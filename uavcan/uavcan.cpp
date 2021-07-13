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
#include "uavcan.hpp"
#include "Heartbeat_1_0.h"
#include "NodeIDAllocationData_1_0.h"

///< heap buffer for o1heap instance
char uavcan::thread::o1heap[O1HEAP_SIZE];

///< o1heap allocator instance pointer
O1HeapInstance* uavcan::thread::o1allocator;

///> libcanard instance for UAVCAN communication
CanardInstance uavcan::thread::canard;

///> uptime counter in seconds
std::uint32_t uavcan::thread::uptime = 0;

/// A transfer-ID is an integer that is incremented whenever a new message is published on a given subject.
/// It is used by the protocol for deduplication, message loss detection, and other critical things.
/// For CAN, each value can be of type uint8_t, but we use larger types for genericity and for statistical purposes,
/// as large values naturally contain the number of times each subject was published to.
struct uavcan::thread::next_transfer_id_s uavcan::thread::next_transfer_id;

/**
 * @fn void O1Allocate*(CanardInstance* const, const size_t)
 * @brief o1heap allocation wrapper for libcanard
 *
 * @param ins		libcanard instance
 * @param amount	size to allocate
 */
void* uavcan::thread::O1Allocate(CanardInstance* const ins, const size_t amount)
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
void uavcan::thread::O1Free(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    o1heapFree(o1allocator, pointer);
}

/**
 * @brief Thread main function
 */
void uavcan::thread::main(void)
{
	setName("UAVCAN");

	trngStart(&TRNGD1, NULL);

	// initialize heap for lib canard
	o1allocator = o1heapInit(o1heap, O1HEAP_SIZE, chSysLock, chSysUnlock);

	// map o1heap to canard instance
	canard = canardInit(O1Allocate, O1Free);
	canard.mtu_bytes = CANARD_MTU_CAN_CLASSIC;  // Defaults to 64 (CAN FD); here select Classic CAN.
	canard.node_id   = 42;                      // Defaults to anonymous; can be set up later at any point.

	/*
	 * Normal main() thread activity, sleeping in a loop.
	 */
	while (TRUE)
	{
		// saturated uptime
		if(uptime < std::numeric_limits<std::uint32_t>::max()) uptime++;

		const bool anonymous = canard.node_id > CANARD_NODE_ID_MAX;
		// Publish heartbeat every second unless the local node is anonymous. Anonymous nodes shall not publish heartbeat.
		if (!anonymous)
		{
			uavcan_node_Heartbeat_1_0 heartbeat = {0};
			heartbeat.uptime                    = uptime;
			heartbeat.mode.value                = uavcan_node_Mode_1_0_OPERATIONAL;
			const O1HeapDiagnostics heap_diag   = o1heapGetDiagnostics(o1allocator);
			if (heap_diag.oom_count > 0)
			{
				heartbeat.health.value = uavcan_node_Health_1_0_CAUTION;
			}
			else
			{
				heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
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
				uavcan_pnp_NodeIDAllocationData_1_0 msg = {0};
				msg.unique_id_hash = DBGMCU->IDCODE;
				uint8_t      serialized[uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
				size_t       serialized_size = sizeof(serialized);
				const int8_t err = uavcan_pnp_NodeIDAllocationData_1_0_serialize_(&msg, &serialized[0], &serialized_size);
				assert(err >= 0);
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
		sleep(1000);
	}

}

/**
 * @brief constructor of UAVCAN thread
 */
uavcan::thread::thread(void){}


/** \} **/ /* end of doxygen group */
/*-- EOF --*/
