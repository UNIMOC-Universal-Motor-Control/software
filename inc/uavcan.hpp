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
#ifndef INC_UAVCAN_HPP_
#define INC_UAVCAN_HPP_

#include <cinttypes>
#include <cmath>
#include <climits>
#include "ch.hpp"
#include "canard.h"



/**
 * uavcan handling functions class
 */
class uavcan
{
private:
	static constexpr sysinterval_t UAVCAN_SPIN_PERIOD = TIME_S2I(1);

	static constexpr std::uint8_t APP_VERSION_MAJOR = 0;
	static constexpr std::uint8_t APP_VERSION_MINOR = 1;
	static constexpr char APP_NODE_NAME[] = "org.unimoc.hardware.4850";
	static constexpr std::uint32_t GIT_HASH = 0xBADC0FFE;            // Normally this should be queried from the VCS when building the firmware

	static constexpr std::size_t UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE = ((3015 + 7) / 8);
	static constexpr std::uint64_t UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE = 0xee468a8121c46a9e;
	static constexpr std::uint32_t UAVCAN_GET_NODE_INFO_DATA_TYPE_ID = 1;


	static constexpr std::uint32_t UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID  			= 1030;
	static constexpr std::uint64_t UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE	= 0x217f5c87d7ec951d;
	static constexpr std::size_t UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_VALUE		= 8192;

	static constexpr std::size_t UNIQUE_ID_LENGTH_BYTES                        = 16;

	static constexpr std::size_t UAVCAN_NODE_STATUS_MESSAGE_SIZE               = 7;
	static constexpr std::uint32_t UAVCAN_NODE_STATUS_DATA_TYPE_ID             = 341;
	static constexpr std::uint64_t UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE      = 0x0f0868d0c1a7c6f1;

	static constexpr std::uint32_t UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID           = 16370;
	static constexpr std::uint64_t UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE    = 0xe02f25d6e0c98ae0;
	static constexpr std::size_t UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MESSAGE_SIZE   = 62;

	static constexpr std::uint32_t UAVCAN_PROTOCOL_PARAM_GETSET_ID             = 11;
	static constexpr std::uint64_t UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE      = 0xa7b622f939d1a4d5;

	///< The Libcanard library instance
	static CanardInstance canard;

	///< Arena for memory allocation, used by the libcanard library
	static uint8_t canard_memory_pool[1024];

	/**
	 * @brief send perodic messages like node status
	 */
	static void Spin(void);

	/**
	 * @brief send messages from the tx queue
	 */
	static void Send(void);

	/**
	 * @brief get messages from the mailboxes and handle them
	 */
	static void Receive(void);

	/**
	 * @brief on reveive callback to determine if the message is handled or not
	 *
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
	static bool ShouldAcceptTransfer(const CanardInstance* ins,
			uint64_t* out_data_type_signature,
			uint16_t data_type_id,
			CanardTransferType transfer_type,
			uint8_t source_node_id);

	/**
	 * @brief handle accepted incomming messages
	 *
	 * this callback is called every time a transfer is received
	 * and accepted in shouldAcceptTransfer.
	 * It is a good idea to put incoming data handlers here.
	 * @param ins
	 * @param transfer
	 */
	static void OnTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);


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
	static void MakeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE]);

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
	static uint16_t MakeNodeInfoMessage(uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE]);

	/**
	 * @brief handle a GetNodeInfo request
	 * @param transfer
	 */
	static void GetNodeInfoHandleCanard(CanardRxTransfer* transfer);

	/**
	 * Generates a unique 24byte number out of the 96bit unique id of the mcu
	 * @param out_uid unique id output buffer
	 */
	static void ReadUniqueID(std::uint8_t* out_uid);

public:
	enum struct health_e: std::uint8_t
	{
		UAVCAN_NODE_HEALTH_OK                                      = 0,
				UAVCAN_NODE_HEALTH_WARNING                                 = 1,
				UAVCAN_NODE_HEALTH_ERROR                                   = 2,
				UAVCAN_NODE_HEALTH_CRITICAL                                = 3,
	};

	enum struct mode_e: std::uint8_t
	{
		UAVCAN_NODE_MODE_OPERATIONAL                               = 0,
				UAVCAN_NODE_MODE_INITIALIZATION                            = 1,
	};

	/**
	 * @brief initialize CAN Bus and LibCanard
	 */
	static void Init(void);

	/**
	 * @brief run lib canard functions
	 */
	static void Run(void);
};

#endif /* INC_UAVCAN_HPP_ */

