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
#include "Heartbeat_1_0.h"
#include "NodeIDAllocationData_1_0.h"
#include "GetInfo_1_0.h"
#include "ExecuteCommand_1_1.h"
#include "List_1_0.h"
#include "Access_1_0.h"

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
	uint32_t uavcan_node_heartbeat;
	uint32_t uavcan_node_port_list;
	uint32_t uavcan_pnp_allocation;
	// Messages published synchronously can share the same transfer-ID:
	uint32_t servo_fast_loop;
	uint32_t servo_1Hz_loop;
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
		rt.timestamp_usec = transfer->timestamp_usec + (std::uint64_t)1e6;
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
//			registerRead(&name[0], &resp.value);
//			// If such register exists and it can be assigned from the request value:
//			if (!uavcan_register_Value_1_0_is_empty_(&resp.value) && registerAssign(&resp.value, &req->value))
//			{
//				registerWrite(&name[0], &resp.value);
//			}
		}

		// Regardless of whether we've just wrote a value or not, we need to read the current one and return it.
		// The client will determine if the write was successful or not by comparing the request value with response.
		uavcan_register_Value_1_0_select_empty_(&resp.value);
//		registerRead(&name[0], &resp.value);

		// Currently, all registers we implement are mutable and persistent. This is an acceptable simplification,
		// but more advanced implementations will need to differentiate between them to support advanced features like
		// exposing internal states via registers, perfcounters, etc.
		resp._mutable   = true;
		resp.persistent = true;

		// Our node does not synchronize its time with the network so we can't populate the timestamp.
		resp.timestamp.microsecond = uavcan_time_SynchronizedTimestamp_1_0_UNKNOWN;

		uint8_t serialized[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
		size_t  serialized_size = sizeof(serialized);
		if (uavcan_register_Access_Response_1_0_serialize_(&resp, &serialized[0], &serialized_size) >= 0)
		{
			CanardTransfer rt = *transfer;  // Response transfers are similar to their requests.
			rt.timestamp_usec = transfer->timestamp_usec + (uint64_t)1e6;
			rt.transfer_kind  = CanardTransferKindResponse;
			rt.payload_size   = serialized_size;
			rt.payload        = &serialized[0];
			(void) canardTxPush(&canard, &rt);
		}
	}
}

/**
 * Initialize the uavan and all its threads
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

    // Load the port-IDs from the registers. You can implement hot-reloading at runtime if desired.
    // Publications:
	/// FIXME add the specific IDs
//    state.port_id.pub.differential_pressure =
//        getPublisherSubjectID("airspeed.differential_pressure",
//                              uavcan_si_unit_temperature_Scalar_1_0_FULL_NAME_AND_VERSION_);
//    state.port_id.pub.static_air_temperature =
//        getPublisherSubjectID("airspeed.static_air_temperature",
//                              uavcan_si_unit_temperature_Scalar_1_0_FULL_NAME_AND_VERSION_);
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
        const int8_t                res =  //
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
        const int8_t                res =  //
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
//								processMessagePlugAndPlayNodeIDAllocation(state, &msg);
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
						.timestamp_usec = (CanardMicrosecond)osalOsGetSystemTimeX() * OSAL_ST_FREQUENCY / 1000000ULL,
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindMessage,
						.port_id        = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
						.remote_node_id = CANARD_NODE_ID_UNSET,
						.transfer_id    = (CanardTransferID)(next_transfer_id.uavcan_node_heartbeat++),
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
							.timestamp_usec = (CanardMicrosecond)osalOsGetSystemTimeX() * OSAL_ST_FREQUENCY / 1000000ULL,
							.priority       = CanardPrioritySlow,
							.transfer_kind  = CanardTransferKindMessage,
							.port_id        = uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_,
							.remote_node_id = CANARD_NODE_ID_UNSET,
							.transfer_id    = (CanardTransferID)(next_transfer_id.uavcan_pnp_allocation++),
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
