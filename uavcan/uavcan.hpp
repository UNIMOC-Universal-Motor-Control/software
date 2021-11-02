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
#ifndef UNIMOC_UAVCAN_THREAD_H_
#define UNIMOC_UAVCAN_THREAD_H_


#include <stdint.h>
#include <stdarg.h>
#include "ch.hpp"
#include "hal.h"
#include "canard.h"
#include "o1heap.h"
#include "settings.hpp"
#include "values.hpp"

#define NUNAVUT_ASSERT(x) osalDbgAssert(x, "NUNAVUT");

namespace uavcan
{
	/**
	 * Initialize the uavan and all its threads
	 */
	void Init(void);

	/**
	 * this thread handles the heartbeat of the node or
	 * if anonymous it sends the node allocation requests
	 */
	class heartbeat : public chibios_rt::BaseStaticThread<1024>
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
		heartbeat();
	};

	/**
	 * this thread takes care of sending the queued frames
	 * and handles the frames received in the can mailboxes
	 */
	class rx_tx : public chibios_rt::BaseStaticThread<1024>
	{
	private:
		static constexpr systime_t 	CYCLE_TIME = OSAL_US2I(201);
		systime_t 					deadline;

		void ProcessGetNodeInfo(const CanardTransfer* transfer);
	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		rx_tx();
	};


	typedef enum register_type_e
	{
		SIGNED,
		UNSIGNED,
		FLOATING
	} register_type_te;

	/**
	 * @struct uavcan_register_s
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
			{"uavcan.node.id", 					&settings.uavcan.node_id,	UNSIGNED, 	true, 		true		},
			{"", 												nullptr,	UNSIGNED, 	false, 		false		},
	};


} // namespace uavcan

#endif /* UNIMOC_UAVCAN_THREAD_H_ */

