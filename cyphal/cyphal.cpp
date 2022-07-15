/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2022 Alexander <tecnologic86@gmail.com> Evers

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
#include "cyphal.hpp"
#include "main.hpp"
#include "hardware.hpp"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/pnp/NodeIDAllocationData_1_0.h"
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/node/ExecuteCommand_1_1.h"
#include "uavcan/_register/List_1_0.h"
#include "uavcan/_register/Access_1_0.h"
#include <reg/udral/service/common/Readiness_0_1.h>
#include <reg/udral/service/actuator/common/_0_1.h>
#include "reg/udral/service/actuator/common/Feedback_0_1.h"
#include "reg/udral/service/actuator/common/Status_0_1.h"
#include "reg/udral/service/actuator/servo/_0_1.h"
#include "reg/udral/service/actuator/esc/_0_1.h"
#include "reg/udral/physics/dynamics/rotation/PlanarTs_0_1.h"
#include "reg/udral/physics/electricity/PowerTs_0_1.h"

namespace cyphal
{
	/**
	 * this thread handles the heartbeat of the node or
	 * if anonymous it sends the node allocation requests
	 */
	class can_heartbeat : public chibios_rt::BaseStaticThread<1024>
	{
	private:
		static constexpr systime_t 	CYCLE_TIME = OSAL_MS2I(1000);
		systime_t 					deadline;
	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:

		can_heartbeat(void):deadline(0) { };
	};

	/**
	 * this thread takes care of sending the queued frames
	 */
	class can_tx : public chibios_rt::BaseStaticThread<256>
	{
	private:
		static std::uint8_t count;
		const std::uint8_t interface;

		event_listener_t el_tx_empty;
		event_listener_t el_tx_needed;
	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		can_tx(void): interface(count) {count++;};
	};

	/**
	 * this thread takes care of sending the queued frames
	 */
	class can_rx : public chibios_rt::BaseStaticThread<256>
	{
	private:
		static std::uint8_t count;
		const std::uint8_t interface;

		event_listener_t el_rx_full;

    	CANRxFrame rxmsg;
		CanardRxTransfer transfer;
		CanardRxSubscription* subscription;
	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		can_rx(void): interface(count) {count++;};
	};

	typedef enum register_type_e
	{
		SIGNED,
		UNSIGNED,
		FLOATING
	} register_type_te;

	/**
	 * @struct cyphal_register_s
	 * @brief definition of one register in the register table for LIST and ACCESS requests
	 *
	 */
	typedef struct register_s
	{
		///< name of the register in the style of unimoc.motor.rotor.r
		char name[50U];
		///< pointer to the internal value represented by the register
		void*	value;
		///< type of the value this register is pointing on, note 32bit wide values is expected.
		register_type_te type;
		///< register variable is writable
		bool _mutable;
		///< register variable is saved in non-volatile memory
		bool persistant;
	} register_ts;

	constexpr register_ts register_table[] = {
			/* register name							pointer to value	type		mutable		persistent */
			{"cyphal.node.id", 					&settings.cyphal.node_id,	UNSIGNED, 	true, 		true		},
			{"", 												nullptr,	UNSIGNED, 	false, 		false		},
	};
}

///< Node uses hardware name for identification
#define NODE_NAME HARDWARE_NAME

///< heap size for the o1heap instance for cyphal frames
static constexpr size_t O1HEAP_SIZE = 8192;

static constexpr std::uint16_t FRAMES_PER_ITER = 1000;

///< heap buffer for o1heap instance
static char o1heap[O1HEAP_SIZE];

///< o1heap allocator instance pointer
static O1HeapInstance* o1allocator;

///< libcanard instance for cyphal communication
static CanardInstance canard;

///< canard transmission queues for each interface.
static CanardTxQueue tx_queues[HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES];

///< interface counter for the constructors of the can threads
std::uint8_t cyphal::can_tx::count = 0;
std::uint8_t cyphal::can_rx::count = 0;

///< can transmission handling thread
static cyphal::can_tx can_send[HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES];

///< can receive handling thread
static cyphal::can_rx can_recv[HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES];

///< can transmission needed event.
static event_source_t es_tx_needed[HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES];

///> uptime counter in seconds
static std::uint32_t uptime;

///< deadline for arming timeout
static systime_t arm_deadline = 0;

///< heartbeat thread
static cyphal::can_heartbeat heartbeat_thread;

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
 * @fn void ProcessGetNodeInfo(const CanardTransfer*)
 * @brief asseamble GetInfo response
 *
 * @param transfer 		GetInfo Request
 */
static void ProcessGetNodeInfo(const CanardRxTransfer* transfer)
{

	// The request object is empty so we don't bother deserializing it. Just send the response.
	uavcan_node_GetInfo_Response_1_0 resp;
	resp.protocol_version.major           = CANARD_CYPHAL_SPECIFICATION_VERSION_MAJOR;
	resp.protocol_version.minor           = CANARD_CYPHAL_SPECIFICATION_VERSION_MINOR;

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
	const int8_t err = uavcan_node_GetInfo_Response_1_0_serialize_(&resp, &serialized[0], &serialized_size);

	osalDbgAssert(err >= 0, "cyphal Get Node Info Response serialize error");

	if (err >= 0)
	{
		const CanardTransferMetadata transfer_metadata = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
				.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = transfer->metadata.transfer_id,
		};

		for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
		{
			int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
					&canard,
					0,			     	       // Zero if transmission deadline is not limited.
					&transfer_metadata,
					serialized_size,           // Size of the message payload (see Nunavut transpiler).
					&serialized[0]);

			// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
			// It is possible to statically prove that an out-of-memory will never occur for a given application if the
			// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
			osalDbgAssert(result < 0, "cyphal Servo Feedback tx push error");
		}
	}
	else
	{
		osalDbgAssert(false, "GetInfo resonse failure");
	}
}

/**
 * @fn const cyphal::register_ts GetRegisterByName*(char*)
 * @brief get cyphal register by name
 *
 * @param name 		register name for search
 * @return			pointer to register with given name. (nullptr if not found)
 */
static const cyphal::register_ts* GetRegisterByName(char* name)
{
	const cyphal::register_ts* reg = cyphal::register_table;

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
static void ProcessRequestRegisterAccess(const CanardRxTransfer* transfer)
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
			const cyphal::register_ts* reg = GetRegisterByName(name);

			switch (reg->type)
			{
			case cyphal::UNSIGNED:
				if(uavcan_register_Value_1_0_is_natural32_(&req.value))
				{
					memcpy(reg->value, req.value.natural32.value.elements, 4);
				}
				uavcan_register_Value_1_0_select_natural32_(&resp.value);
				resp.value.natural32.value.elements[0] = *(std::uint32_t*)reg->value;
				resp.value.natural32.value.count = 0;
				break;
			case cyphal::SIGNED:
				if(uavcan_register_Value_1_0_is_integer32_(&req.value))
				{
					memcpy(reg->value, req.value.integer32.value.elements, 4);
				}
				uavcan_register_Value_1_0_select_integer32_(&resp.value);
				resp.value.integer32.value.elements[0] = *(std::int32_t*)reg->value;
				resp.value.integer32.value.count = 0;
				break;
			case cyphal::FLOATING:
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
		const std::int8_t err = uavcan_register_Access_Response_1_0_serialize_(&resp, &serialized[0], &serialized_size);

		osalDbgAssert(err >= 0, "cyphal Access Response serialize error");

		if (err >= 0)
		{
			const CanardTransferMetadata transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindMessage,
					.port_id        = uavcan_register_Access_1_0_FIXED_PORT_ID_,
					.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
					.transfer_id    = transfer->metadata.transfer_id,
			};

			for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
			{
				int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
						&canard,
						0,			     	       // Zero if transmission deadline is not limited.
						&transfer_metadata,
						serialized_size,           // Size of the message payload (see Nunavut transpiler).
						&serialized[0]);

				// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
				// It is possible to statically prove that an out-of-memory will never occur for a given application if the
				// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
				osalDbgAssert(result < 0, "cyphal Access tx push error");
			}
		}
		else
		{
			osalDbgAssert(false, "Access response failure");
		}
	}
}

static void ProcessRequestList(const CanardRxTransfer* transfer)
{
	uavcan_register_List_Request_1_0 req  = {0};
	size_t                           size = transfer->payload_size;
	if (uavcan_register_List_Request_1_0_deserialize_(&req, (const uint8_t*)transfer->payload, &size) >= 0)
	{
		const char* name = cyphal::register_table[req.index].name;
	    uavcan_register_Name_1_0 out;
	    uavcan_register_Name_1_0_initialize_(&out);

	    out.name.count = nunavutChooseMin(strlen(name), uavcan_register_Name_1_0_name_ARRAY_CAPACITY_);
	    std::memcpy(out.name.elements, name, out.name.count);

		const uavcan_register_List_Response_1_0 resp = {.name = out};
		uint8_t serialized[uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
		size_t  serialized_size = sizeof(serialized);
		const std::int8_t err = uavcan_register_List_Response_1_0_serialize_(&resp, &serialized[0], &serialized_size);

		osalDbgAssert(err >= 0, "cyphal List Response serialize error");

		if (err >= 0)
		{
			const CanardTransferMetadata transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindMessage,
					.port_id        = uavcan_register_List_1_0_FIXED_PORT_ID_,
					.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
					.transfer_id    = transfer->metadata.transfer_id,
			};

			for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
			{
				int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
						&canard,
						0,			     	       // Zero if transmission deadline is not limited.
						&transfer_metadata,
						serialized_size,           // Size of the message payload (see Nunavut transpiler).
						&serialized[0]);

				// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
				// It is possible to statically prove that an out-of-memory will never occur for a given application if the
				// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
				osalDbgAssert(result < 0, "cyphal List tx push error");
			}
		}
		else
		{
			osalDbgAssert(false, "List response failure");
		}
	}
}

static void ProcessRequestExecuteCommand(const CanardRxTransfer* transfer)
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
		const std::int8_t err = uavcan_node_ExecuteCommand_Response_1_1_serialize_(&resp, &serialized[0], &serialized_size);

		osalDbgAssert(err >= 0, "cyphal Execute Response serialize error");

		if (err >= 0)
		{
			const CanardTransferMetadata transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindMessage,
					.port_id        = uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_,
					.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
					.transfer_id    = transfer->metadata.transfer_id,
			};

			for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
			{
				int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
						&canard,
						0,			     	       // Zero if transmission deadline is not limited.
						&transfer_metadata,
						serialized_size,           // Size of the message payload (see Nunavut transpiler).
						&serialized[0]);

				// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
				// It is possible to statically prove that an out-of-memory will never occur for a given application if the
				// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
				osalDbgAssert(result < 0, "cyphal Execute tx push error");
			}
		}
		else
		{
			osalDbgAssert(false, "Execute response failure");
		}
	}
}

/// https://github.com/cyphal/public_regulated_data_types/blob/master/reg/udral/service/actuator/servo/_.0.1.uavcan
static void ProcessMessageServoSetpoint(const CanardRxTransfer* transfer)
{
	size_t size = transfer->payload_size;
	reg_udral_physics_dynamics_rotation_Planar_0_1 msg;
	if (reg_udral_physics_dynamics_rotation_Planar_0_1_deserialize_(&msg, (const uint8_t*)transfer->payload, &size) >= 0)
	{
		values::motor::rotor::setpoint::phi 	= msg.kinematics.angular_position.radian;
		values::motor::rotor::setpoint::omega   = msg.kinematics.angular_velocity.radian_per_second;
//		values::motor::rotor::setpoint::acceleration = msg.kinematics.angular_acceleration.radians_per_second_per_second;
		values::motor::rotor::setpoint::torque   = msg._torque.newton_meter;
	}
}

/// https://github.com/cyphal/public_regulated_data_types/blob/master/reg/udral/service/common/Readiness.0.1.uavcan
static void ProcessMessageServiceReadiness(const CanardRxTransfer* transfer)
{
	size_t size = transfer->payload_size;
	reg_udral_service_common_Readiness_0_1 msg;
	if (reg_udral_service_common_Readiness_0_1_deserialize_(&msg, (const uint8_t*)transfer->payload, &size) >= 0)
	{
		if(msg.value >= reg_udral_service_common_Readiness_0_1_ENGAGED)
		{
			hardware::pwm::output::Enable();
		}
		else
		{
			hardware::pwm::output::Disable();
		}
		/// set the deadline to some future time if that time is overdue we disarm
		arm_deadline = osalOsGetSystemTimeX() + OSAL_MS2I(1000);
	}
}

/**
 * @fn void main(void)
 * @brief task for handing frames from libcanard to CAN HAL
 *
 */
void cyphal::can_tx::main(void)
{
	setName("CAN Send");

	chEvtRegister(&es_tx_needed[interface], &el_tx_needed, 0);
	chEvtRegister(&hardware::can::event[interface].txempty, &el_tx_empty, 0);

	while(TRUE)
	{
		for (const CanardTxQueueItem* ti = NULL;
				(ti = canardTxPeek(&tx_queues[interface])) != NULL;)  // Peek at the top of the queue.
		{
			if (		(0U == ti->tx_deadline_usec)
					||  (ti->tx_deadline_usec > hardware::GetMonotonicMicroseconds()))  // Check the deadline.
			{
				if (hardware::can::Transmit(interface, ti->frame)) // Send the frame over this redundant CAN iface.
				{
					break;                             // If the driver is busy, break and retry later.
				}
			}
			// After the frame is transmitted or if it has timed out while waiting, pop it from the queue and deallocate:
			canard.memory_free(&canard, canardTxPop(&tx_queues[interface], ti));
		}
		// wait for either tx empty or new stuff to be transmitted
		chEvtWaitAnyTimeout(ALL_EVENTS, OSAL_MS2I(1000));
	}
}


void cyphal::can_rx::main(void)
{
	setName("CAN Receive");

	chEvtRegister(&hardware::can::event[interface].rxfull, &el_rx_full, 0);

	while(TRUE)
	{
		// wait for any frame received
		const msg_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, OSAL_MS2I(1000));

		if (evt == MSG_OK)
		{
			CanardFrame received_frame;

			while (hardware::can::Receive(interface, received_frame))
			{
				CanardMicrosecond rs_timestamp = hardware::GetMonotonicMicroseconds();

				const std::int8_t result = canardRxAccept(&canard,
						rs_timestamp,               // When the frame was received, in microseconds.
						&received_frame,            // The CAN frame received from the bus.
						interface,                  // If the transport is not redundant, use 0.
						&transfer,
						&subscription);

				if (result < 0)
				{
					// An error has occurred: either an argument is invalid or we've ran out of memory.
					// It is possible to statically prove that an out-of-memory will never occur for a given application if
					// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
					// Reception of an invalid frame is NOT an error.
				}
				else if (result == 1)
				{
					if (transfer.metadata.transfer_kind == CanardTransferKindMessage)
					{
						size_t size = transfer.payload_size;
						if (transfer.metadata.port_id == uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_)
						{
							uavcan_pnp_NodeIDAllocationData_1_0 msg;
							if (uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(&msg, (uint8_t*)transfer.payload, &size) >= 0)
							{
								std::uint64_t uid = DBGMCU->IDCODE;
								std::uint16_t node_id = msg.allocated_node_id.elements[0].value;

								if ((node_id <= CANARD_NODE_ID_MAX) && (std::memcmp(&uid, &msg.unique_id_hash, sizeof(uid)) == 0))
								{
									settings.cyphal.node_id = node_id;
								}
							}
						}
						else
						{
							osalDbgAssert(false, "Sub without Handler");  // Seems like we have set up a port subscription without a handler -- bad implementation.
						}
					}
					else
					{
						//If a handler was assigned as user reference, call it and pass the transfer.
						if(subscription->user_reference!=NULL)
						{
							((void (*)(const CanardRxTransfer* transfer))subscription->user_reference)(&transfer);
						}
					}
					canard.memory_free(&canard, transfer.payload);                  // Deallocate the dynamic memory afterwards.
				}
				else
				{
					// Nothing to do.
					// The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
					// Reception of an invalid frame is NOT reported as an error because it is not an error.
				}
			}
		}
	}
}
/**
 * @fn void Init(void)
 * @brief Initialize the uavan and all its threads
 *
 */
void cyphal::Init(void)
{
	// initialize heap for lib canard
	o1allocator = o1heapInit(o1heap, O1HEAP_SIZE);

	// map o1heap to canard instance
	canard = canardInit(O1Allocate, O1Free);

	for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
	{
		if(settings.cyphal.fd && HARDWARE_CAPABIITY_CAN_FD)
		{
			tx_queues[i] = canardTxInit(O1HEAP_SIZE / HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES, CANARD_MTU_CAN_FD);
		}
		else
		{
			tx_queues[i] = canardTxInit(O1HEAP_SIZE / HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES, CANARD_MTU_CAN_CLASSIC);
		}
		// init event source for send event.
		osalEventObjectInit(&es_tx_needed[i]);
		// start the frame handling tasks
		can_recv[i].start(NORMALPRIO + 11);
		can_send[i].start(NORMALPRIO + 10);
	}
	canard.node_id   = settings.cyphal.node_id; // Defaults to anonymous; can be set up later at any point.

	// set the arm deadline to an sure unarmed value
	arm_deadline = osalOsGetSystemTimeX();

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

	heartbeat_thread.start(NORMALPRIO);
}

/**
 * @fn void SendFeedback(void)
 * @brief Send activated Feedback messages from fast loop (eg. 1kHz)
 *
 */
void cyphal::SendFeedback(void)
{
    const bool     anonymous         = canard.node_id > CANARD_NODE_ID_MAX;
    static uint8_t servo_transfer_id;
    const CanardMicrosecond deadline_time = hardware::GetMonotonicMicroseconds() + 10000ULL;

    ++servo_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.

    // Publish feedback if the subject is enabled and the node is non-anonymous.
    if (!anonymous && (settings.cyphal.subject.servo.feedback <= CANARD_SUBJECT_ID_MAX))
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
        osalDbgAssert(err >= 0, "cyphal Servo Feedback serialisation error");

        if (err >= 0)
        {
        	const CanardTransferMetadata transfer_metadata = {
        	    .priority       = CanardPriorityHigh,
        	    .transfer_kind  = CanardTransferKindMessage,
        	    .port_id        = settings.cyphal.subject.servo.feedback,
        	    .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
        	    .transfer_id    = servo_transfer_id,
        	};

        	for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
        	{
        		int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
        				&canard,
						deadline_time,     		 // Zero if transmission deadline is not limited.
						&transfer_metadata,
						serialized_size,           // Size of the message payload (see Nunavut transpiler).
						&serialized[0]);

        		// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
        		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
        		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
        		osalDbgAssert(result < 0, "cyphal Servo Feedback tx push error");
        	}
        }
    }

    // Publish dynamics if the subject is enabled and the node is non-anonymous.
    if (!anonymous && (settings.cyphal.subject.servo.dynamics <= CANARD_SUBJECT_ID_MAX))
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

        osalDbgAssert(err >= 0, "cyphal Servo Dynamics serialisation error");

        if (err >= 0)
        {
        	const CanardTransferMetadata transfer_metadata = {
        	    .priority       = CanardPriorityHigh,
        	    .transfer_kind  = CanardTransferKindMessage,
        	    .port_id        = settings.cyphal.subject.servo.dynamics,
        	    .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
        	    .transfer_id    = servo_transfer_id,
        	};

        	for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
        	{
        		int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
        				&canard,
						deadline_time,     		 // Zero if transmission deadline is not limited.
						&transfer_metadata,
						serialized_size,           // Size of the message payload (see Nunavut transpiler).
						&serialized[0]);

        		// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
        		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
        		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
        		osalDbgAssert(result < 0, "cyphal Servo Dynamics tx push error");
        	}
        }
    }

    // Publish power if the subject is enabled and the node is non-anonymous.
    if (!anonymous && (settings.cyphal.subject.servo.power <= CANARD_SUBJECT_ID_MAX))
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

        osalDbgAssert(err >= 0, "cyphal Servo Power serialisation error");

        if (err >= 0)
        {
        	const CanardTransferMetadata transfer_metadata = {
        	    .priority       = CanardPriorityNominal,
        	    .transfer_kind  = CanardTransferKindMessage,
        	    .port_id        = settings.cyphal.subject.servo.power,
        	    .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
        	    .transfer_id    = servo_transfer_id,
        	};

        	for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
        	{
        		int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
        				&canard,
						deadline_time,     		 // Zero if transmission deadline is not limited.
						&transfer_metadata,
						serialized_size,           // Size of the message payload (see Nunavut transpiler).
						&serialized[0]);

        		// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
        		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
        		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
        		osalDbgAssert(result < 0, "cyphal Servo Power tx push error");
        	}
        }
    }

    for (std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
    {
    	eventflags_t flags = i + 1;
    	osalEventBroadcastFlags(&es_tx_needed[i], flags);
    }

    // Timeout for Readiness Messages Arming state.
    if (arm_deadline <= osalOsGetSystemTimeX())
    {
    	hardware::pwm::output::Disable();
    }
}

/**
 * @brief Thread main function
 */
void cyphal::can_heartbeat::main(void)
{
	static std::uint8_t heartbeat_transfer_id;

	setName("cyphal HEARTBEAT");

	/*
	 * Normal main() thread activity, sleeping in a loop.
	 */
	while (TRUE)
	{
		deadline = chibios_rt::System::getTime();
		// saturated uptime
		if(uptime < std::numeric_limits<std::uint32_t>::max()) uptime++;

		static bool was_anonymous = true;
		static std::uint16_t old_servo_readiness = CANARD_SUBJECT_ID_MAX;
		static std::uint16_t old_servo_setpoint = CANARD_SUBJECT_ID_MAX;
		const bool anonymous = canard.node_id > CANARD_NODE_ID_MAX;
		const CanardMicrosecond deadline_time = hardware::GetMonotonicMicroseconds() + 250000ULL;

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

			osalDbgAssert(err >= 0, "cyphal Heartbeat serialisation error!");

			if (err >= 0)
			{
				heartbeat_transfer_id++;

				const CanardTransferMetadata transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindMessage,
						.port_id        = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
						.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
						.transfer_id    = heartbeat_transfer_id,
				};

				for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
				{
					int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
							&canard,
							deadline_time,     		 // Zero if transmission deadline is not limited.
							&transfer_metadata,
							serialized_size,           // Size of the message payload (see Nunavut transpiler).
							&serialized[0]);

					// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
					// It is possible to statically prove that an out-of-memory will never occur for a given application if the
					// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
					osalDbgAssert(result < 0, "cyphal Heartbeat tx push error");
				}
			}

			// Subscriptions:
			// (none in this application)
			// Service servers:
			if(was_anonymous)
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

			if(was_anonymous)
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

			if(was_anonymous)
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

			if(was_anonymous)
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
			was_anonymous = false;


			if (   settings.cyphal.subject.servo.setpoint <= CANARD_SUBJECT_ID_MAX
				&& settings.cyphal.subject.servo.setpoint != old_servo_setpoint)  // Do not subscribe if not configured.
			{
				static CanardRxSubscription rx;
				// set the request handler
				rx.user_reference = (void*)ProcessMessageServoSetpoint;
				const int8_t                res =  //
						canardRxSubscribe(&canard,
								CanardTransferKindMessage,
								settings.cyphal.subject.servo.setpoint,
								reg_udral_physics_dynamics_rotation_Planar_0_1_EXTENT_BYTES_,
								100000ULL,
								&rx);
				if (res < 0)
				{
					//    		return -res;
				}
				else
				{
					 old_servo_setpoint = settings.cyphal.subject.servo.setpoint;
				}
			}

			if (	settings.cyphal.subject.servo.readiness <= CANARD_SUBJECT_ID_MAX
				&&  settings.cyphal.subject.servo.readiness != old_servo_readiness)  // Do not subscribe if not configured.
			{
				static CanardRxSubscription rx;
				// set the request handler
				rx.user_reference = (void*)ProcessMessageServiceReadiness;
				const int8_t                res =  //
						canardRxSubscribe(&canard,
								CanardTransferKindMessage,
								settings.cyphal.subject.servo.readiness,
								reg_udral_service_common_Readiness_0_1_EXTENT_BYTES_,
								100000ULL,
								&rx);
				if (res < 0)
				{
					//    		return -res;
				}
				else
				{
					 old_servo_readiness = settings.cyphal.subject.servo.readiness;
				}
			}
		}
#if HARDWARE_CAPABIITY_RANDOM == TRUE
		else // If we don't have a node-ID, obtain one by publishing allocation request messages until we get a response.
		{
			std::uint8_t random;

			was_anonymous = true;

			// The Specification says that the allocation request publication interval shall be randomized.
			// We implement randomization by calling rand() at fixed intervals and comparing it against some threshold.
			// There are other ways to do it, of course. See the docs in the Specification or in the DSDL definition here:
			// https://github.com/cyphal/public_regulated_data_types/blob/master/cyphal/pnp/8165.NodeIDAllocationData.2.0.cyphal
			// Note that a high-integrity/safety-certified application is unlikely to be able to rely on this feature.
			bool err = hardware::random::Generate(&random, sizeof(random));
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


				osalDbgAssert(err >= 0, "cyphal Node Alloc serialisation error!");

				if (err >= 0)
				{
					heartbeat_transfer_id++;

					const CanardTransferMetadata transfer_metadata = {
							.priority       = CanardPrioritySlow,
							.transfer_kind  = CanardTransferKindMessage,
							.port_id        = uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_,
							.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
							.transfer_id    = heartbeat_transfer_id,
					};

					for(std::uint8_t i = 0; i < HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES; i++)
					{
						int32_t result = canardTxPush(&tx_queues[i], // Call this once per redundant CAN interface (queue).
								&canard,
								0,     		 			   // Zero if transmission deadline is not limited.
								&transfer_metadata,
								serialized_size,           // Size of the message payload (see Nunavut transpiler).
								&serialized[0]);

						// An error has occurred: either an argument is invalid, the TX queue is full, or we've run out of memory.
						// It is possible to statically prove that an out-of-memory will never occur for a given application if the
						// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
						osalDbgAssert(result < 0, "cyphal Node Alloc tx push error");
					}
				}

			}
		}
#endif
		sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
	}
}



/** \} **/ /* end of doxygen group */
/*-- EOF --*/
