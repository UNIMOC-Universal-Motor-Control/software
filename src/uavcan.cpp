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
#include <cstring>
#include <cstdlib>
#include "uavcan.hpp"
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
	void thread::Init(void)
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
				   OnTransferReceived,                // Callback, see CanardOnTransferReception
				   ShouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
				   NULL);

		canardSetLocalNodeID(&canard, 100);
	}

	/**
	 * @brief send perodic messages like node status
	 */
	void thread::Spin(void)
	{
		std::uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
		// This variable MUST BE STATIC; refer to the libcanard documentation for the background
		static std::uint8_t transfer_id = 0;

		MakeNodeStatusMessage(buffer);

		canardBroadcast(&canard,
				UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
				UAVCAN_NODE_STATUS_DATA_TYPE_ID,
				&transfer_id,
				CANARD_TRANSFER_PRIORITY_LOW,
				buffer,
				UAVCAN_NODE_STATUS_MESSAGE_SIZE);
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
	bool thread::ShouldAcceptTransfer(const CanardInstance* ins,
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
	 * @brief handle accepted incomming messages
	 *
	 * this callback is called every time a transfer is received
	 * and accepted in shouldAcceptTransfer.
	 * It is a good idea to put incoming data handlers here.
	 * @param ins
	 * @param transfer
	 */
	void thread::OnTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
	{
		if ((transfer->transfer_type == CanardTransferTypeRequest) &&
				(transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
		{
			canardGetNodeInfoHandle(transfer);
		}
	}

	/**
	 * @brief prepare node status message
	 *
	 * To make a node status message we will have to compose it manually. For that we will need three values:
	 *    - Uptime in seconds.
	 *    - Node health. Our node will always be 100% healthy.
	 *    - Node mode. Our node will always be in the operational mode.
	 *
	 * @param buffer
	 */
	void thread::MakeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
	{
		const std::uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
		const std::uint8_t node_mode   = UAVCAN_NODE_MODE_OPERATIONAL;
		std::memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
		const std::uint32_t uptime_sec = TIME_I2S(chibios_rt::System::getTime());

		canardEncodeScalar(buffer,  0, 32, &uptime_sec);
		canardEncodeScalar(buffer, 32,  2, &node_health);
		canardEncodeScalar(buffer, 34,  3, &node_mode);
	}

	/**
	 * @brief answer for the Node Info request.
	 *
	 * When the UAVCAN GUI Tool receives this message for the first time,
	 * it will attempt to get more info about the new node,
	 * so we also have to implement a handler that will form a GetNodeInfo response
	 * and send it back to the requesting node (client)
	 *
	 * @param buffer
	 * @return
	 */
	uint16_t thread::MakeNodeInfoMessage(std::uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE])
	{
		std::memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
		MakeNodeStatusMessage(buffer);

		buffer[7] = APP_VERSION_MAJOR;
		buffer[8] = APP_VERSION_MINOR;
		buffer[9] = 1;  // Optional field flags, VCS commit is set
		canardEncodeScalar(buffer, 80, 32, &GIT_HASH);

		ReadUniqueID(&buffer[24]);

		std::memcpy(&buffer[41], APP_NODE_NAME, std::strlen(APP_NODE_NAME));
		return (41 + std::strlen(APP_NODE_NAME));
	}


	/**
	 * @brief handle a GetNodeInfo request
	 * @param transfer
	 */
	void thread::GetNodeInfoHandleCanard(CanardRxTransfer* transfer)
	{
		uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
		memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
		const uint16_t len = MakeNodeInfoMessage(buffer);
		int result = canardRequestOrRespond(&canard,
				transfer->source_node_id,
				UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
				UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
				&transfer->transfer_id,
				transfer->priority,
				CanardResponse,
				&buffer[0],
				(uint16_t)len);

		if (result < 0)
		{
			// TODO: handle the error
		}
	}

	/**
	 * Generates a unique 24byte number out of the 96bit unique id of the mcu
	 * @param out_uid unique id output buffer
	 */
	void thread::ReadUniqueID(std::uint8_t* out_uid)
	{
		///< Pointer to 96bit unique id of the MCU, see ref man page 1397
		const std::uint8_t* const UID = (std::uint8_t*)0x1FF07A10;
		constexpr std::uint8_t UID_LEN = UNIQUE_ID_LENGTH_BYTES/2;

		for (uint8_t i = 0; i < UID_LEN; i++)
		{
			out_uid[2*i] = UID[i];
			out_uid[2*i + 1] = UID[UID_LEN - i];
		}
	}

	/**
	 * uavcan thread
	 */
	void thread::main(void)
	{

		chRegSetThreadName("UAVCAN main thread");

		Init();

		while(1)
		{
			sleep(TIME_S2I(1));

			Spin();


		}
	}

	/**
	 * uavcan thread
	 */
	void thread::rx::main(void)
	{
		event_listener_t el_can_rx;
		event_listener_t el_can_err;
		CANRxFrame rxmsg;

		chRegSetThreadName("UAVCAN receive thread");

		/*
		 * register thread to the CAN device driver to be
		 * notified on received CAN Frames
		 */
		chEvtRegister(&CAND1.rxfull_event, &el_can_rx, 0);

		while (true)
		{
			// sleep until next CAN Frame received
			if (chEvtWaitAny(ALL_EVENT) == 0)
				continue;

			// handle all pending messages
			while (canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX,&rxmsg, TIME_IMMEDIATE) == MSG_OK)
			{
				CanardCANFrame rx_frame;

				rx_frame.id = rxmsg.SID;
				rx_frame.data_len = rxmsg.DLC;
				std::memcpy(rx_frame.data, rxmsg.data8, CANARD_CAN_FRAME_MAX_DATA_LEN);

				canardHandleRxFrame(&canard, &rx_frame, TIME_I2US(chibios_rt::System::getTime()));
			}
		}
	}

}/* namespace uavcan */



