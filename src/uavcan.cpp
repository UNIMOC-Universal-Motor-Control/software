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
#include "values.hpp"
#include "drivers/stm32/canard_stm32.h"


///< The Libcanard library instance
CanardInstance uavcan::canard;

///< Arena for memory allocation, used by the libcanard library
std::uint8_t uavcan::canard_memory_pool[1024] = {0};

constexpr char uavcan::APP_NODE_NAME[];
constexpr std::uint32_t uavcan::GIT_HASH;


/**
 * @brief initialize CAN Bus and LibCanard
 */
void uavcan::Init(void)
{

	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	CanardSTM32CANTimings timings;
	int result = canardSTM32ComputeCANTimings(STM32_PCLK1, 1000000, &timings);
	if (result)
	{
		asm volatile("BKPT #01");
	}
	result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
	if (result)
	{
		asm volatile("BKPT #01");
	}

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
void uavcan::Spin(void)
{
	static sysinterval_t spin_time = 0;
	static uint8_t transfer_id = 0;           // This variable MUST BE STATIC; refer to the libcanard documentation for the background

	// rate limiting
	if(chibios_rt::System::isSystemTimeWithin(spin_time, spin_time + UAVCAN_SPIN_PERIOD))
	{
		return;
	}
	else
	{
		uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
		spin_time = chibios_rt::System::getTime();


		MakeNodeStatusMessage(buffer);

		canardBroadcast(&canard,
				UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
				UAVCAN_NODE_STATUS_DATA_TYPE_ID,
				&transfer_id,
				CANARD_TRANSFER_PRIORITY_LOW,
				buffer,
				UAVCAN_NODE_STATUS_MESSAGE_SIZE);
	}
}

/**
 * @brief send messages from the tx queue
 */
void uavcan::Send(void)
{
	const CanardCANFrame* txf = canardPeekTxQueue(&canard);
	while(txf)
	{
		const int tx_res = canardSTM32Transmit(txf);
		if (tx_res < 0)         // Failure - drop the frame and report
		{
			asm volatile("BKPT #01");   // TODO: handle the error properly
		}
		if(tx_res > 0)
		{
			canardPopTxQueue(&canard);
		}
		txf = canardPeekTxQueue(&canard);
	}
}

/**
 * @brief get messages from the mailboxes and handle them
 */
void uavcan::Receive(void)
{
	CanardCANFrame rx_frame;
	int res = canardSTM32Receive(&rx_frame);
	if(res)
	{
		canardHandleRxFrame(&canard, &rx_frame, TIME_I2US(chibios_rt::System::getTime()));
	}
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
bool uavcan::ShouldAcceptTransfer(const CanardInstance* ins,
		uint64_t* out_data_type_signature,
		uint16_t data_type_id,
		CanardTransferType transfer_type,
		uint8_t source_node_id)
{
	(void)source_node_id;
	(void)ins;

	if ((transfer_type == CanardTransferTypeRequest) &&
			(data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
	{
		*out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
		return true;
	}

	if(data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)
	{
		*out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
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
void uavcan::OnTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
	(void)ins;

	if ((transfer->transfer_type == CanardTransferTypeRequest) &&
			(transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
	{
		GetNodeInfoHandleCanard(transfer);
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
void uavcan::MakeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
	std::memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
	const std::uint32_t uptime_sec = TIME_I2S(chibios_rt::System::getTime());

	canardEncodeScalar(buffer,  0, 32, &uptime_sec);
	canardEncodeScalar(buffer, 32,  2, &values.uavcan.health);
	canardEncodeScalar(buffer, 34,  3, &values.uavcan.mode);
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
uint16_t uavcan::MakeNodeInfoMessage(std::uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE])
{
	std::uint32_t hash = GIT_HASH;
	std::memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
	MakeNodeStatusMessage(buffer);

	buffer[7] = APP_VERSION_MAJOR;
	buffer[8] = APP_VERSION_MINOR;
	buffer[9] = 1;  // Optional field flags, VCS commit is set
	canardEncodeScalar(buffer, 80, 32, &hash);

	ReadUniqueID(&buffer[24]);

	std::memcpy(&buffer[41], APP_NODE_NAME, std::strlen(APP_NODE_NAME));
	return (41 + std::strlen(APP_NODE_NAME));
}


/**
 * @brief handle a GetNodeInfo request
 * @param transfer
 */
void uavcan::GetNodeInfoHandleCanard(CanardRxTransfer* transfer)
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
void uavcan::ReadUniqueID(std::uint8_t* out_uid)
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
 * uavcan handling
 */
void uavcan::Run(void)
{
	while(1)
	{
		Send();
		Receive();
		Spin();
	}
}




