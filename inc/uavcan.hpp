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
 * @namespace uavcan handling thread and tools
 */
namespace uavcan
{

	/**
	 * uavcan handling thread
	 */
	class thread : public chibios_rt::BaseStaticThread<256>
	{
	private:

		///< The Libcanard library instance
		static CanardInstance canard;

		///< Arena for memory allocation, used by the libcanard library
		static uint8_t canard_memory_pool[1024];

		/**
		 * @brief initialize CAN Bus and LibCanard
		 */
		static void Init(void);

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
		static bool shouldAcceptTransfer(const CanardInstance* ins,
				uint64_t* out_data_type_signature,
				uint16_t data_type_id,
				CanardTransferType transfer_type,
				uint8_t source_node_id);

		/**
		 *  this callback is called every time a transfer is received
		 *  and accepted in shouldAcceptTransfer.
		 *  It is a good idea to put incoming data handlers here.
		 * @param ins
		 * @param transfer
		 */
		static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);

	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		/**
		 * generic constructor
		 */
		thread();
	};
}/* namespace uavcan */

#endif /* INC_UAVCAN_HPP_ */

