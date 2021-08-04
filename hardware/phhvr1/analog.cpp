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
#include <cstring>
#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "hardware_interface.hpp"
#include "hal.h"

using namespace hardware::analog;




///< absolute maximum current
constexpr float hardware::analog::current::MAX = 1.65f/(20.0f*(0.002f/3.0f));

///< reference to thread to be woken up in the hardware control cycle.
chibios_rt::ThreadReference hardware::control_thread = nullptr;


/**
 * Initialize ADC hardware
 */
void hardware::analog::Init(void)
{

}

/**
 * Get the current values in the last control cycle
 * @param currents references to the current samples
 */
void hardware::analog::current::Value(std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& currents)
{

}

/**
 * Get current derivatives in the last control
 * cycles
 * @param derivatives
 */
void hardware::analog::current::Derivative(std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& derivatives)
{

}

/**
 * set dc current offsets
 */
void hardware::analog::current::SetOffset(void)
{

}

/**
 * Read the DC Bus voltage
 * @return DC Bus voltage in Volts
 */
float hardware::analog::voltage::DCBus(void)
{
	return 0.0f;
}

/**
 * Get the temperature of the power electronics
 * @return Temperature of the power electronics in °C
 */
float hardware::analog::temperature::Bridge(void)
{
	return 0.0f;
}

/**
 * Get the temperature of the motor
 * @return Temperature of the Motor in °C
 */
float hardware::analog::temperature::Motor(void)
{
	return 0.0f;
}


/**
 * get analog input
 * @return analog in put 0 - 1
 */
float hardware::analog::input(void)
{
	return 0.0f;
}


