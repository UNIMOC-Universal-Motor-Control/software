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
 */
#include <cstring>
#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "systems.hpp"
#include "hardware_interface.hpp"
#include "hal.h"


/**
 * @fn void hardware_cordic_Init(void)
 * @brief Initialise the Cordig just for Sin/Cos operation
 *
 */
void hardware_cordic_Init(void)
{

	/*
	 * Function: 0 = Cosine
	 * Precision: 6 = 24 Cycles (highest Precision)
	 * Results: 2
	 */
	CORDIC->CSR = CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_1 | CORDIC_CSR_NRES;
}

/**
  @brief         Floating-point sine and cosine function.
  @param 	     theta   input value in q31
  @retval	     out     points to processed sine cosine output
 */
systems::sin_cos hardware::SinCos(const std::int32_t theta)
{
	using namespace systems;
	constexpr float scale = 1.0f/(float)std::numeric_limits<std::int32_t>::max();
	sin_cos out;

	// Write Theta to Cordic to start Calculation
	CORDIC->WDATA = theta;
	// When Calculation is finished the results can be read
	out.cos = (float)CORDIC->RDATA * scale;
	out.sin = (float)CORDIC->RDATA * scale;

	return out;
}
