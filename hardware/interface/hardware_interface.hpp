/*
    UNIMOC - Universal Motor Control  2019 Alexander <tecnologic86@gmail.com>

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
#ifndef HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_
#include <cstdint>
#include "ch.hpp"
#include "hal.h"

namespace unimoc {
	namespace hardware {
		/**
		 * Motor Phases. Normally fixed to 3
		 */
		constexpr uint8_t PHASES = 3;

		namespace pwm {
			/*
			 * This interface defines the functions for pwm handling
			 * which all hardware variants need to implement.
			 */

			///< PWM Timer clock in Hz
			constexpr uint32_t TIMER_CLOCK = STM32_TIMCLK2;


			///< deadtime in nano seconds
			constexpr uint32_t DEADTIME = 300;

			/**
			 * PWM frequency in Hz
			 *
			 * The ADC sampling relies on the PWM frequency!
			 * Each ADC samples 16 samples with a sample time of 15clk + 12clk for conversion
			 * + 1clk for triggering and 1 hole cyle more for the new setup of the adc.
			 * This gives 17*28=476clk. ADC clock is divided by 8 from PWM clock this means
			 * The Period of the PWM needs to be greater than 3808clk. With center aligned
			 * PWM this is the period of PWM half period.
			 */
			constexpr uint32_t PERIOD = 3900;

			/**
			 * Initialize PWM hardware with outputs disabled!
			 */
			extern void Init();

			/**
			 * Enable PWM Outputs.
			 * @note Power output is active after this call
			 */
			extern void EnableOutputs(void);

			/**
			 * Disable PWM Outputs.
			 */
			extern void DisableOutputs(void);

			/**
			 * Get pwm output state
			 * @return pwm output state, true = pwm active
			 */
			extern bool OutputActive(void);

			/**
			 * Set the normalized duty cycles for each phase
			 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
			 */
			extern void SetDutys(float dutys[PHASES]);

		} /* namespace pwm */

		///< Control cycle time
		constexpr float Tc = (float)(pwm::PERIOD + 1) / (float)pwm::TIMER_CLOCK;

		///< Control cycle frequency
		constexpr float Fc = (float)pwm::TIMER_CLOCK / (float)(pwm::PERIOD + 1);

		namespace adc
		{
			/*
			 * This interface defines the functions for adc handling
			 * which all hardware variants need to implement.
			 */
			/**
			 * Initialize ADC hardware with outputs disabled!
			 */
			extern void Init();

		} /* namespace adc */
	} /* namespace hardware */
} /* namespace unimoc */

#endif /* HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_ */

