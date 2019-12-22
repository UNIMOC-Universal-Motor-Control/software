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

namespace unimoc {
	namespace hardware {
		namespace pwm {
			/*
			 * This interface defines the functions for pwm handling
			 * which all hardware variants need to implement.
			 */

			/**
			 * Motor Phases. Normally fixed to 3
			 */
			constexpr uint8_t PHASES = 3;

			/**
			 * Initialize PWM hardware with outputs disabled!
			 * @param freq_hz PWM frequency in Hz
			 */
			extern void Init(uint32_t freq_hz);

			/**
			 * Set PWM Freqency
			 * @param freq_hz PWM frequency in Hz
			 */
			extern void SetFreqency(uint32_t freq_hz);

			/**
			 * Get PWM Frequency
			 * @return PWM frequency in Hz
			 */
			extern uint32_t GetFreqency();

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
			 * Set the normalized duty cycles for each phase
			 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
			 */
			extern void SetDutys(float dutys[PHASES]);

		} /* namespace pwm */
	} /* namespace hardware */
} /* namespace unimoc */

#endif /* HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_ */

