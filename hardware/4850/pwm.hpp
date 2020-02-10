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
#ifndef HARDWARE_INTERFACE_4850_PWM_HPP_
#define HARDWARE_INTERFACE_4850_PWM_HPP_
#include <cstdint>
#include "hardware_interface.hpp"
#include "ch.hpp"
#include "hal.h"


namespace hardware {
	namespace pwm {

		///< PWM driver instance
		extern PWMDriver* PWMP;

		///< PWM duty counts
		extern uint16_t duty_counts[DUTY_BUFFER_CYCLES][INJECTION_CYCLES][PHASES];

		///< PWM duty buffer cycle
		uint8_t duty_buffer_cycle;
	} /* namespace pwm */
} /* namespace hardware */


#endif /* HARDWARE_INTERFACE_4850_PWM_HPP_ */
