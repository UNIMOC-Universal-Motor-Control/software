/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

	This file is part of UNIMOC.

	UNIMOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <limits>
#include <cstdint>
#include <cstring>
#include "uavcan.hpp"
#include "main.hpp"
#include "hardware.hpp"
#include "bxcan.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/pnp/NodeIDAllocationData_1_0.h"
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/node/ExecuteCommand_1_1.h"
#include "uavcan/_register/List_1_0.h"
#include "uavcan/_register/Access_1_0.h"
#include "reg/udral/service/actuator/common/Feedback_0_1.h"
#include "reg/udral/service/actuator/common/Status_0_1.h"
#include "reg/udral/service/actuator/servo/__0_1.h"
#include "reg/udral/service/actuator/esc/__0_1.h"
#include "reg/udral/physics/dynamics/rotation/PlanarTs_0_1.h"
#include "reg/udral/physics/electricity/PowerTs_0_1.h"

///< Node uses hardware name for identification
#define NODE_NAME HARDWARE_NAME

///< heap size for the o1heap instance for UAVCAN frames
static constexpr size_t O1HEAP_SIZE = 4096;

static constexpr std::uint16_t FRAMES_PER_ITER = 1000;

///< heap buffer for o1heap instance
static char o1heap[O1HEAP_SIZE];

///< o1heap allocator instance pointer
static O1HeapInstance* o1allocator;

///> libcanard instance for UAVCAN communication
static CanardInstance canard;

///> uptime counter in seconds
static std::uint32_t uptime;

/// A transfer-ID is an integer that is incremented whenever a new message is published on a given subject.
/// It is used by the protocol for deduplication, message loss detection, and other critical things.
/// For CAN, each value can be of type uint8_t, but we use larger types for genericity and for statistical purposes,
/// as large values naturally contain the number of times each subject was published to.
static struct next_transfer_id_s
{
	uint32_t node_heartbeat;
	uint32_t node_port_list;
	uint32_t pnp_allocation;
	uint32_t servo_fast_loop;
} next_transfer_id;

///< can handling thread
static uavcan::rx_tx can_thread;

///< heartbeat thread
static uavcan::heartbeat heartbeat_thread;

/**
 * @fn void O1Allocate*(CanardInstance* const, const size_t)
 * @brief o1heap allocation wrapper for libcanard
 *
 * @param ins		libcanard instance
 * @param amount	size to allocate
 */
static void* O1Allocate(CanardInstance* const ins, const size_t amount)
{
    (void) ins;
    return o1heapAllocate(o1allocator, amount);
}

/**
 * @fn void O1Free(CanardInstance* const, void* const)
 * @brief o1heap free memory wrapper for libcanard
 *
 * @param ins		libcanard instance
 * @param pointer	size to allocate
 */
static void O1Free(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    o1heapFree(o1allocator, pointer);
}

/**
 * @fn CanardMicrosecond getMonotonicMicroseconds(void)
 * @brief get Systemsticks from start in micro seconds
 *
 * @return micro secounds from startup
 */
static CanardMicrosecond getMonotonicMicroseconds(void)
{
	const uint64_t us_per_tick = 1000000ULL / OSAL_ST_FREQUENCY;
	return (CanardMicrosecond)osalOsGetSystemTimeX() * us_per_tick;
}

/**
 * @fn void ProcessGetNodeInfo(const CanardTransfer*)
 * @brief asseamble GetInfo response
 *
 * @param transfer 		GetInfo Request
 */
static void ProcessGetNodeInfo(const CanardTransfer* transfer)
{

	// The request object is empty so we don't bother deserializing it. Just send the response.
	uavcan_node_GetInfo_Response_1_0 resp;
	resp.protocol_version.major           = CANARD_UAVCAN_SPECIFICATION_VERSION_MAJOR;
	resp.protocol_version.minor           = CANARD_UAVCAN_SPECIFICATION_VERSION_MINOR;

	// The hardware version is not populated in this demo because it runs on no specific hardware.
	// An embedded node would usually determine the version by querying the hardware.
	char unique_id[16];
	snprintf(unique_id, 16, "%ld", DBGMCU->IDCODE);
	resp.software_version.major   = VERSION_MAJOR;
	resp.software_version.minor   = VERSION_MINOR;
	resp.software_vcs_revision_id = 0xDEADBEEFDEADBEEF; //VCS_REVISION_ID;
	memcpy(resp.unique_id, unique_id, sizeof(unique_id));


	// The node name is the name of the product like a reversed Internet domain name (or like a Java package).
	resp.name.count = strlen(NODE_NAME);
	memcpy(&resp.name.elements, NODE_NAME, resp.name.count);

	uint8_t      serialized[uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
	size_t       serialized_size = sizeof(serialized);
	const int8_t res = uavcan_node_GetInfo_Response_1_0_serialize_(&resp, &serialized[0], &serialized_size);
	if (res >= 0)
	{
		CanardTransfer rt = *transfer;  // Response transfers are similar to their requests.
		rt.timestamp_usec = transfer->timestamp_usec + 1000000ULL;
		rt.transfer_kind  = CanardTransferKindResponse;
		rt.payload_size   = serialized_size;
		rt.payload        = &serialized[0];
		(void) canardTxPush(&canard, &rt);
	}
	else
	{
		osalDbgAssert(false, "GetInfo resonse failure");
	}
}

/**
 * @fn const uavcan::register_ts GetRegisterByName*(char*)
 * @brief get uavcan register by name
 *
 * @param name 		register name for search
 * @return			pointer to register with given name. (nullptr if not found)
 */
const uavcan::register_ts* GetRegisterByName(char* name)
{
	const uavcan::register_ts* reg = uavcan::register_table;

	while(std::strcmp(name, reg->name) != 0)
	{
		reg++;

		// end of list, reg not found
		if(reg->value != nullptr)
		{
			reg = nullptr;
			break;
		}
	}

	return reg;
}

/**
 * @fn void ProcessRequestRegisterAccess(const CanardTransfer*)
 * @brief asseamble Register access response
 *
 * @param transfer		Register Access Request
 */
void ProcessRequestRegisterAccess(const CanardTransfer* transfer)
{
	uavcan_register_Access_Request_1_0 req;
	size_t                             size = transfer->payload_size;
	if (uavcan_register_Access_Request_1_0_deserialize_(&req, (const uint8_t*)transfer->payload, &size) >= 0)
	{
		char name[uavcan_register_Name_1_0_name_ARRAY_CAPACITY_ + 1] = {0};
		osalDbgAssert(req.name.name.count < sizeof(name), "Register Name to Long");
		memcpy(&name[0], req.name.name.elements, req.name.name.count);
		name[req.name.name.count] = '\0';

		uavcan_register_Access_Response_1_0 resp;

		// If we're asked to write a new value, do it now:
		if (!uavcan_register_Value_1_0_is_empty_(&req.value))
		{
			uavcan_register_Value_1_0_select_empty_(&resp.value);
			const uavcan::register_ts* reg = GetRegisterByName(name);

			switch (reg->type)
			{
			case uavcan::UNSIGNED:
				if(uavcan_register_Value_1_0_is_natural32_(&req.value))
				{
					memcpy(reg->value, req.value.natural32.value.elements, 4);
				}
				uavcan_register_Value_1_0_select_natural32_(&resp.value);
				resp.value.natural32.value.elements[0] = *(std::uint32_t*)reg->value;
				resp.value.natural32.value.count = 0;
				break;
			case uavcan::SIGNED:
				if(uavcan_register_Value_1_0_is_integer32_(&req.value))
				{
					memcpy(reg->value, req.value.integer32.value.elements, 4);
				}
				uavcan_register_Value_1_0_select_integer32_(&resp.value);
				resp.value.integer32.value.elements[0] = *(std::int32_t*)reg->value;
				resp.value.integer32.value.count = 0;
				break;
			case uavcan::FLOATING:
				if(uavcan_register_Value_1_0_is_real32_(&req.value))
				{
					memcpy(reg->value, req.value.real32.value.elements, 4);
				}
				uavcan_register_Value_1_0_select_real32_(&resp.value);
				resp.value.real32.value.elements[0] = *(float*)reg->value;
				resp.value.real32.value.count = 0;
				break;
			}


			resp._mutable   = reg->_mutable;
			resp.persistent = reg->persistant;
		}

		// Our node does not synchronize its time with the network so we can't populate the timestamp.
		resp.timestamp.microsecond = uavcan_time_SynchronizedTimestamp_1_0_UNKNOWN;

		uint8_t serialized[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
		size_t  serialized_size = sizeof(serialized);
		if (uavcan_register_Access_Response_1_0_serialize_(&resp, &serialized[0], &serialized_size) >= 0)
		{
			CanardTransfer rt = *transfer;  // Response transfers are similar to their requests.
			rt.timestamp_usec = transfer->timestamp_usec + 1000000ULL;
			rt.transfer_kind  = CanardTransferKindResponse;
			rt.payload_size   = serialized_size;
			rt.payload        = &serialized[0];
			(void) canardTxPush(&canard, &rt);
		}
	}
}

void ProcessRequestList(const CanardTransfer* transfer)
{
	uavcan_register_List_Request_1_0 req  = {0};
	size_t                           size = transfer->payload_size;
	if (uavcan_register_List_Request_1_0_deserialize_(&req, (const uint8_t*)transfer->payload, &size) >= 0)
	{
		const char* name = uavcan::register_table[req.index].name;
	    uavcan_register_Name_1_0 out;
	    uavcan_register_Name_1_0_initialize_(&out);

	    out.name.count = nunavutChooseMin(strlen(name), uavcan_register_Name_1_0_name_ARRAY_CAPACITY_);
	    std::memcpy(out.name.elements, name, out.name.count);

		const uavcan_register_List_Response_1_0 resp = {.name = out};
		uint8_t serialized[uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
		size_t  serialized_size = sizeof(serialized);
		if (uavcan_register_List_Response_1_0_serialize_(&resp, &serialized[0], &serialized_size) >= 0)
		{
			CanardTransfer rt = *transfer;  // Response transfers are similar to their requests.
			rt.timestamp_usec = transfer->timestamp_usec + 1000000ULL;
			rt.transfer_kind  = CanardTransferKindResponse;
			rt.payload_size   = serialized_size;
			rt.payload        = &serialized[0];
			(void) canardTxPush(&canard, &rt);
		}
	}
}

void ProcessRequestExecuteCommand(const CanardTransfer* transfer)
{
	uavcan_node_ExecuteCommand_Request_1_1 req;
	size_t                                 size = transfer->payload_size;
	if (uavcan_node_ExecuteCommand_Request_1_1_deserialize_(&req, (const uint8_t*)transfer->payload, &size) >= 0)
	{
		uavcan_node_ExecuteCommand_Response_1_1 resp;
		switch (req.command)
		{
		case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_BEGIN_SOFTWARE_UPDATE:
		{
			char file_name[uavcan_node_ExecuteCommand_Request_1_1_parameter_ARRAY_CAPACITY_ + 1] = {0};
			std::memcpy(file_name, req.parameter.elements, req.parameter.count);
			file_name[req.parameter.count] = '\0';
			// TODO: invoke the bootloader with the specified file name. See https://github.com/Zubax/kocherga/
//			printf("Firmware update request; filename: '%s' \n", &file_name[0]);
			resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_STATE;  // This is a stub.
			break;
		}
		case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_FACTORY_RESET:
		{
//			registerDoFactoryReset();
			resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
			break;
		}
		case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_RESTART:
		{
//			g_restart_required = true;
			resp.status        = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
			break;
		}
		case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_STORE_PERSISTENT_STATES:
		{
			// If your registers are not automatically synchronized with the non-volatile storage, use this command
			// to commit them to the storage explicitly. Otherwise, it is safe to remove it.
			// In this demo, the registers are stored in files, so there is nothing to do.
			resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
			break;
		}
		// You can add vendor-specific commands here as well.
		default:
		{
			resp.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
			break;
		}
		}

		uint8_t serialized[uavcan_node_ExecuteCommand_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
		size_t  serialized_size = sizeof(serialized);
		if (uavcan_node_ExecuteCommand_Response_1_1_serialize_(&resp, &serialized[0], &serialized_size) >= 0)
		{
			CanardTransfer rt = *transfer;  // Response transfers are similar to their requests.
			rt.timestamp_usec = transfer->timestamp_usec + 1000000ULL;
			rt.transfer_kind  = CanardTransferKindResponse;
			rt.payload_size   = serialized_size;
			rt.payload        = &serialized[0];
			(void) canardTxPush(&canard, &rt);
		}
	}
}

/**
 * @fn void Init(void)
 * @brief Initialize the uavan and all its threads
 *
 */
void uavcan::Init(void)
{
	// initialize heap for lib canard
	o1allocator = o1heapInit(o1heap, O1HEAP_SIZE);

	// map o1heap to canard instance
	canard = canardInit(O1Allocate, O1Free);
	canard.mtu_bytes = CANARD_MTU_CAN_CLASSIC;  // Defaults to 64 (CAN FD); here select Classic CAN.
	canard.node_id   = CANARD_NODE_ID_MAX;      // Defaults to anonymous; can be set up later at any point.

	// bxCAN driver initialisation
	BxCANTimings timings;
	// fix it rate to 500k FIXME add autobaud algo in listen only
	bxCANComputeTimings(STM32_PCLK1, 500000, &timings);

	if(bxCANConfigure(0, timings, false)!=true)
	{
		osalSysHalt("uavcan");
	}

    // Subscriptions:
    // (none in this application)

    // Set up subject subscriptions and RPC-service servers.
    // Message subscriptions:
    if (canard.node_id > CANARD_NODE_ID_MAX)
    {
        static CanardRxSubscription rx;
        const int8_t                res =  //
            canardRxSubscribe(&canard,
                              CanardTransferKindMessage,
                              uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_,
                              uavcan_pnp_NodeIDAllocationData_1_0_EXTENT_BYTES_,
                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                              &rx);
        if (res < 0)
        {
//            return -res;
        }
    }
    // Service servers:
    {
        static CanardRxSubscription rx;
        // set the request handler
        rx.user_reference = (void*)ProcessGetNodeInfo;
        const int8_t                res =
            canardRxSubscribe(&canard,
                              CanardTransferKindRequest,
                              uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
                              uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_,
                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                              &rx);
        if (res < 0)
        {
//            return -res;
        }
    }
    {
        static CanardRxSubscription rx;
        rx.user_reference = (void*)ProcessRequestExecuteCommand;
        const int8_t                res =
            canardRxSubscribe(&canard,
                              CanardTransferKindRequest,
                              uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_,
                              uavcan_node_ExecuteCommand_Request_1_1_EXTENT_BYTES_,
                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                              &rx);
        if (res < 0)
        {
//            return -res;
        }
    }
    {
        static CanardRxSubscription rx;
        rx.user_reference = (void*)ProcessRequestRegisterAccess;
        const int8_t                res =  //
            canardRxSubscribe(&canard,
                              CanardTransferKindRequest,
                              uavcan_register_Access_1_0_FIXED_PORT_ID_,
                              uavcan_register_Access_Request_1_0_EXTENT_BYTES_,
                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                              &rx);
        if (res < 0)
        {
//            return -res;
        }
    }
    {
        static CanardRxSubscription rx;
        rx.user_reference = (void*)ProcessRequestList;
        const int8_t                res =  //
            canardRxSubscribe(&canard,
                              CanardTransferKindRequest,
                              uavcan_register_List_1_0_FIXED_PORT_ID_,
                              uavcan_register_List_Request_1_0_EXTENT_BYTES_,
                              CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                              &rx);
        if (res < 0)
        {
//            return -res;
        }
    }

	can_thread.start(NORMALPRIO + 10);
	heartbeat_thread.start(NORMALPRIO);
}

/**
 * @fn void SendFeedback(void)
 * @brief Send activated Feedback messages from fast loop (eg. 1kHz)
 *
 */
void uavcan::SendFeedback(void)
{
    const bool     anonymous         = canard.node_id > CANARD_NODE_ID_MAX;
    const uint64_t servo_transfer_id = next_transfer_id.servo_fast_loop++;
    const CanardMicrosecond deadline_time = getMonotonicMicroseconds() + 10000ULL;

    // Publish feedback if the subject is enabled and the node is non-anonymous.
    if (!anonymous && (settings.uavcan.servo_feedback <= CANARD_SUBJECT_ID_MAX))
    {
        reg_udral_service_actuator_common_Feedback_0_1 msg;
        msg.heartbeat.readiness.value =
        		hardware::pwm::output::Active() ? reg_udral_service_common_Readiness_0_1_ENGAGED
        										: reg_udral_service_common_Readiness_0_1_STANDBY;
        // If there are any hardware or configuration issues, report them here:
        msg.heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
        // Serialize and publish the message:
        uint8_t      serialized[reg_udral_service_actuator_common_Feedback_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
        size_t       serialized_size = sizeof(serialized);
        const int8_t err =
            reg_udral_service_actuator_common_Feedback_0_1_serialize_(&msg, &serialized[0], &serialized_size);
        assert(err >= 0);
        if (err >= 0)
        {
            const CanardTransfer transfer = {
                .timestamp_usec = deadline_time,
                .priority       = CanardPriorityHigh,
                .transfer_kind  = CanardTransferKindMessage,
                .port_id        = settings.uavcan.servo_feedback,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id    = (CanardTransferID) servo_transfer_id,
                .payload_size   = serialized_size,
                .payload        = &serialized[0],
            };
            (void) canardTxPush(&canard, &transfer);
        }
    }

    // Publish dynamics if the subject is enabled and the node is non-anonymous.
    if (!anonymous && (settings.uavcan.servo_dynamics <= CANARD_SUBJECT_ID_MAX))
    {
        reg_udral_physics_dynamics_rotation_PlanarTs_0_1 msg;
        // Our node does not synchronize its clock with the network, so we cannot timestamp our publications:
        msg.timestamp.microsecond = uavcan_time_SynchronizedTimestamp_1_0_UNKNOWN;
        msg.value.kinematics.angular_position.radian                  = values::motor::rotor::angle;
        msg.value.kinematics.angular_velocity.radian_per_second       = values::motor::rotor::omega;
        msg.value.kinematics.angular_acceleration.radian_per_second_per_second = 0.0f;
        msg.value._torque.newton_meter                                = values::motor::torque::electric;
        // Serialize and publish the message:
        uint8_t serialized[reg_udral_physics_dynamics_rotation_PlanarTs_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
        size_t  serialized_size = sizeof(serialized);
        const int8_t err =
            reg_udral_physics_dynamics_rotation_PlanarTs_0_1_serialize_(&msg, &serialized[0], &serialized_size);
        assert(err >= 0);
        if (err >= 0)
        {
            const CanardTransfer transfer = {
                .timestamp_usec = deadline_time,
                .priority       = CanardPriorityHigh,
                .transfer_kind  = CanardTransferKindMessage,
                .port_id        = settings.uavcan.servo_dynamics,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id    = (CanardTransferID) servo_transfer_id,
                .payload_size   = serialized_size,
                .payload        = &serialized[0],
            };
            (void) canardTxPush(&canard, &transfer);
        }
    }

    // Publish power if the subject is enabled and the node is non-anonymous.
    if (!anonymous && (settings.uavcan.servo_power <= CANARD_SUBJECT_ID_MAX))
    {
        reg_udral_physics_electricity_PowerTs_0_1 msg;
        // Our node does not synchronize its clock with the network, so we cannot timestamp our publications:
        msg.timestamp.microsecond = uavcan_time_SynchronizedTimestamp_1_0_UNKNOWN;
        // TODO populate real values:
        msg.value.current.ampere = systems::Length(values::motor::rotor::i);
        msg.value.voltage.volt   = systems::Length(values::motor::rotor::u);
        // Serialize and publish the message:
        uint8_t serialized[reg_udral_physics_electricity_PowerTs_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
        size_t  serialized_size = sizeof(serialized);
        const int8_t err = reg_udral_physics_electricity_PowerTs_0_1_serialize_(&msg, &serialized[0], &serialized_size);
        assert(err >= 0);
        if (err >= 0)
        {
            const CanardTransfer transfer = {
                .timestamp_usec = deadline_time,
                .priority       = CanardPriorityHigh,
                .transfer_kind  = CanardTransferKindMessage,
                .port_id        = settings.uavcan.servo_power,
                .remote_node_id = CANARD_NODE_ID_UNSET,
                .transfer_id    = (CanardTransferID) servo_transfer_id,
                .payload_size   = serialized_size,
                .payload        = &serialized[0],
            };
            (void) canardTxPush(&canard, &transfer);
        }
    }
}


/**
 * @brief Thread main function
 */
void uavcan::rx_tx::main(void)
{
	setName("CANARD");

	while(TRUE)
	{
		deadline = chibios_rt::System::getTime();

		// Process received frames by feeding them from bxCAN driver to libcanard.
		// This function will invoke the "process received" handler specified during init.
		CanardFrame frame             = {0,0,0, nullptr};
		uint8_t     buffer[CANARD_MTU_CAN_CLASSIC] = {0};
		frame.payload = &buffer;
		for (uint16_t i = 0; i < FRAMES_PER_ITER; ++i)
		{
			const bool result = bxCANPop(
					0,
					&(frame.extended_can_id),
					&(frame.payload_size),
					buffer);
			if (result == false)  // The read operation has timed out with no frames, nothing to do here.
			{
				break;
			}
			// The bxCAN driver doesn't give a timestamp
			frame.timestamp_usec = getMonotonicMicroseconds();

			CanardTransfer transfer;
			CanardRxSubscription *subscription = NULL;
			const int8_t   canard_result = canardRxAccept2(&canard, &frame, 0, &transfer, &subscription);
			if (canard_result > 0)
			{
				osalDbgAssert(subscription != NULL, "No Subscriotion");
				//If a handler was assigned as user reference, call it and pass the transfer.
				if(subscription->user_reference!=NULL)
				{
					((void (*)(const CanardTransfer* transfer))subscription->user_reference)(&transfer);
				}
				else
				{
					if (transfer.transfer_kind == CanardTransferKindMessage)
					{
						size_t size = transfer.payload_size;
						if (transfer.port_id == uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_)
						{
							uavcan_pnp_NodeIDAllocationData_1_0 msg;
							if (uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(&msg, (uint8_t*)transfer.payload, &size) >= 0)
							{
								std::uint64_t uid = DBGMCU->IDCODE;
								std::uint16_t node_id = msg.allocated_node_id.elements[0].value;

							    if ((node_id <= CANARD_NODE_ID_MAX) && (std::memcmp(&uid, &msg.unique_id_hash, sizeof(uid)) == 0))
							    {
							    	settings.uavcan.node_id = node_id;
							    }
							}
						}
						else
						{
							osalDbgAssert(false, "Sub without Handler");  // Seems like we have set up a port subscription without a handler -- bad implementation.
						}
					}
				}

				canard.memory_free(&canard, (void*) transfer.payload);
			}
			else if ((canard_result == 0) || (canard_result == -CANARD_ERROR_OUT_OF_MEMORY))
			{
				;  // Zero means that the frame did not complete a transfer so there is nothing to do.
				// OOM should never occur if the heap is sized correctly. We track OOM errors via heap API.
			}
			else
			{
				osalDbgAssert(false, "undefined Error");  // No other error can possibly occur at runtime.
			}
		}

		// Transmit pending frames from the prioritized TX queue managed by libcanard
		const CanardFrame* pframe = canardTxPeek(&canard);  // Take the highest-priority frame from TX queue.
		uint64_t us = getMonotonicMicroseconds();
		while (pframe != NULL)
		{
			// Attempt transmission only if the frame is not yet timed out while waiting in the TX queue.
			// Otherwise just drop it and move on to the next one.
			if ((pframe->timestamp_usec == 0) || (pframe->timestamp_usec > us))
			{
				const bool result = bxCANPush(
						0,
						us,
						pframe->timestamp_usec,
						pframe->extended_can_id,
						pframe->payload_size,
						pframe->payload);
				if (result == false)
				{
					break;  //TODO: Handle errors properly
				}

			}
			//Remove the frame from the transmission queue and free its memory.
			canardTxPop(&canard);
			canard.memory_free(&canard, (void*) pframe);

			//Check if there are any more frames to transmit.
			pframe = canardTxPeek(&canard);
		}

		sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
	}
}

/**
 * @brief constructor of UAVCAN receive and transmit thread
 */
uavcan::rx_tx::rx_tx(void):deadline(0){};


/**
 * @brief Thread main function
 */
void uavcan::heartbeat::main(void)
{
	setName("UAVCAN HEARTBEAT");

	trngStart(&TRNGD1, NULL);

	/*
	 * Normal main() thread activity, sleeping in a loop.
	 */
	while (TRUE)
	{
		deadline = chibios_rt::System::getTime();
		// saturated uptime
		if(uptime < std::numeric_limits<std::uint32_t>::max()) uptime++;

		const bool anonymous = canard.node_id > CANARD_NODE_ID_MAX;
		const CanardMicrosecond deadline_time = getMonotonicMicroseconds() + 250000ULL;

		// Publish heartbeat every second unless the local node is anonymous. Anonymous nodes shall not publish heartbeat.
		if (!anonymous)
		{
			uavcan_node_Heartbeat_1_0 heartbeat = {
				.uptime                      = uptime,
				.health                      = {uavcan_node_Health_1_0_NOMINAL},
				.mode                        = {uavcan_node_Mode_1_0_OPERATIONAL},
				.vendor_specific_status_code = 0,
			};

			const O1HeapDiagnostics heap_diag   = o1heapGetDiagnostics(o1allocator);
			if (heap_diag.oom_count > 0)
			{
				heartbeat.health.value = uavcan_node_Health_1_0_CAUTION;
			}


			uint8_t      serialized[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
			size_t       serialized_size                                                        = sizeof(serialized);
			const int8_t err = uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, &serialized[0], &serialized_size);
			osalDbgAssert(err >= 0, "UAVCAN Heartbeat serialisation error!");
			if (err >= 0)
			{
				const CanardTransfer transfer = {
						.timestamp_usec = deadline_time,
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindMessage,
						.port_id        = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
						.remote_node_id = CANARD_NODE_ID_UNSET,
						.transfer_id    = (CanardTransferID)(next_transfer_id.node_heartbeat++),
						.payload_size   = serialized_size,
						.payload        = &serialized[0],
				};
				(void) canardTxPush(&canard, &transfer);
			}
		}
		else  // If we don't have a node-ID, obtain one by publishing allocation request messages until we get a response.
		{
			std::uint8_t random;
			// The Specification says that the allocation request publication interval shall be randomized.
			// We implement randomization by calling rand() at fixed intervals and comparing it against some threshold.
			// There are other ways to do it, of course. See the docs in the Specification or in the DSDL definition here:
			// https://github.com/UAVCAN/public_regulated_data_types/blob/master/uavcan/pnp/8165.NodeIDAllocationData.2.0.uavcan
			// Note that a high-integrity/safety-certified application is unlikely to be able to rely on this feature.
			bool err = trngGenerate(&TRNGD1, 1, &random);
			if (!err && random > 127)  // NOLINT
			{
				// Note that this will only work over CAN FD. If you need to run PnP over Classic CAN, use message v1.0.
				uavcan_pnp_NodeIDAllocationData_1_0 msg = {
					.unique_id_hash = DBGMCU->IDCODE,
					.allocated_node_id = {0,0},
				};
				uint8_t      serialized[uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
				size_t       serialized_size = sizeof(serialized);
				const int8_t err = uavcan_pnp_NodeIDAllocationData_1_0_serialize_(&msg, &serialized[0], &serialized_size);

				if (err >= 0)
				{
					const CanardTransfer transfer = {
							.timestamp_usec = deadline_time,
							.priority       = CanardPrioritySlow,
							.transfer_kind  = CanardTransferKindMessage,
							.port_id        = uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_,
							.remote_node_id = CANARD_NODE_ID_UNSET,
							.transfer_id    = (CanardTransferID)(next_transfer_id.pnp_allocation++),
							.payload_size   = serialized_size,
							.payload        = &serialized[0],
					};
					(void) canardTxPush(&canard, &transfer);  // The response will arrive asynchronously eventually.
				}
			}
		}
		sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
	}

}

/**
 * @brief constructor of UAVCAN Heartbeat thread
 */
uavcan::heartbeat::heartbeat(void):deadline(0){};


/** \} **/ /* end of doxygen group */
/*-- EOF --*/
