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

    Credits to stancecoke's EBiCS/EBiCS_Firmware
 */
#ifndef INC_DISPLAY_HPP_
#define INC_DISPLAY_HPP_

#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "hal.h"
#include "controller.hpp"


namespace  display
{

	/**
	 * Display Thread
	 */
	class thread : public chibios_rt::BaseStaticThread<2048>
	{
	private:
		typedef struct config_s
		{
			uint8_t assist_level;
			bool    light;
			uint8_t motor_characteristic;
			uint8_t wheel_size;
			uint8_t max_speed;
			uint8_t power_assist_control_mode;
			uint8_t controller_max_current;
			uint8_t p1;
			uint8_t p2;
			uint8_t p3;
			uint8_t p4;
			uint8_t p5;
			uint8_t c1;
			uint8_t c2;
			uint8_t c4;
			uint8_t c5;
			uint8_t c12;
			uint8_t c13;
			uint8_t c14;
		} config_ts;

		static constexpr float BATTERY_LI_ION_CELLS_NUMBER = 10.0f;
		static constexpr float COMMUNICATIONS_BATTERY_VOLTAGE	= (BATTERY_LI_ION_CELLS_NUMBER * 3.45f); // example: 7S battery, should be = 24

		// Considering the follow voltage values for each li-ion battery cell
		// State of charge 		| voltage
		static constexpr float LI_ION_CELL_VOLTS_MAX 	= 4.20f;
		static constexpr float LI_ION_CELL_VOLTS_100 	= 4.20f;
		static constexpr float LI_ION_CELL_VOLTS_80 	= 3.86f;// 4.02
		static constexpr float LI_ION_CELL_VOLTS_60 	= 3.68f;// 3.87
		static constexpr float LI_ION_CELL_VOLTS_40		= 3.46f;// 3.80
		static constexpr float LI_ION_CELL_VOLTS_20		= 3.28f;// 3.73
		static constexpr float LI_ION_CELL_VOLTS_0		= 3.10f;// 3.27
		static constexpr float LI_ION_CELL_VOLTS_MIN 	= 3.10f;

		static constexpr float  BATTERY_PACK_VOLTS_100	= (LI_ION_CELL_VOLTS_100 * BATTERY_LI_ION_CELLS_NUMBER) * 256.0f;
		static constexpr float  BATTERY_PACK_VOLTS_80 	= (LI_ION_CELL_VOLTS_80  * BATTERY_LI_ION_CELLS_NUMBER) * 256.0f;
		static constexpr float  BATTERY_PACK_VOLTS_60	= (LI_ION_CELL_VOLTS_60  * BATTERY_LI_ION_CELLS_NUMBER) * 256.0f;
		static constexpr float  BATTERY_PACK_VOLTS_40	= (LI_ION_CELL_VOLTS_40  * BATTERY_LI_ION_CELLS_NUMBER) * 256.0f;
		static constexpr float  BATTERY_PACK_VOLTS_20	= (LI_ION_CELL_VOLTS_20  * BATTERY_LI_ION_CELLS_NUMBER) * 256.0f;
		static constexpr float  BATTERY_PACK_VOLTS_0	= (LI_ION_CELL_VOLTS_0   * BATTERY_LI_ION_CELLS_NUMBER) * 256.0f;

		uint8_t tx_buffer[12];
		uint8_t error;
		uint8_t rx_buffer[13];

		SerialDriver *sdp;
		config_ts config;

		/**
		 * initialize thread
		 */
		void init(void);

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

} /* namespace display */

#endif /* INC_DISPLAY_HPP_ */

