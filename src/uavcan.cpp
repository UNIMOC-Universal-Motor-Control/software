/*
    UNIMOC - Universal Motor Control  2020 Alexander <tecnologic86@gmail.com> Brand

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
#include <uavcan.hpp>
#include "hal.h"

/**
 * @namespace uavcan handling thread and tools
 */
namespace uavcan
{
	///< The Libcanard library instance
	CanardInstance thread::canard;

	///< Arena for memory allocation, used by the libcanard library
	std::uint8_t thread::canard_memory_pool[1024];

	/**
	 * @brief initialize CAN Bus and LibCanard
	 */
	static void thread::Init(void)
	{

		// 1000KBaud, automatic wakeup, automatic recover from abort mode.
		// See section 22.7.7 on the STM32 reference manual.
		CANConfig cancfg = {
				CAN_MCR_ABOM | CAN_MCR_AWUM,
				CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
				CAN_BTR_TS1(14) | CAN_BTR_BRP(5)
		};

		canStart(&CAND1, &cancfg);

		canardInit(&canard,                           // Uninitialized library instance
				   canard_memory_pool,                // Raw memory chunk used for dynamic allocation
				   sizeof(canard_memory_pool),        // Size of the above, in bytes
				   onTransferReceived,                // Callback, see CanardOnTransferReception
				   shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
				   NULL);

		canardSetLocalNodeID(&canard, 100);
	}

	/**
	 * this callback is called every time a transfer is received
	 * to determine if it should be passed further to the library
	 * or ignored. Here we should filter out all messages
	 * that are not needed for our particular task.
	 *
	 * @param ins
	 * @param out_data_type_signature
	 * @param data_type_id
	 * @param transfer_type
	 * @param source_node_id
	 * @return
	 */
	bool thread::shouldAcceptTransfer(const CanardInstance* ins,
			uint64_t* out_data_type_signature,
			uint16_t data_type_id,
			CanardTransferType transfer_type,
			uint8_t source_node_id)
	{
		if ((transfer_type == CanardTransferTypeRequest) &&
				(data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
		{
			*out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
			return true;
		}

		return false;
	}

	/**
	 *  this callback is called every time a transfer is received
	 *  and accepted in shouldAcceptTransfer.
	 *  It is a good idea to put incoming data handlers here.
	 * @param ins
	 * @param transfer
	 */
	void thread::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
	{
		if ((transfer->transfer_type == CanardTransferTypeRequest) &&
				(transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
		{
			canardGetNodeInfoHandle(transfer);
		}
	}

	/**
	 * uavcan thread
	 */
	void main(void)
	{
		Init();

		while(1)
		{
			sleep(TIME_MS2I(1000));

			std::uint32_t spin_time = System::getTime();;

			std::uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
			// This variable MUST BE STATIC; refer to the libcanard documentation for the background
			static std::uint8_t transfer_id = 0;

			makeNodeStatusMessage(buffer);

			canardBroadcast(&canard,
					UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
					UAVCAN_NODE_STATUS_DATA_TYPE_ID,
					&transfer_id,
					CANARD_TRANSFER_PRIORITY_LOW,
					buffer,
					UAVCAN_NODE_STATUS_MESSAGE_SIZE);
		}
	}

}/* namespace uavcan */



