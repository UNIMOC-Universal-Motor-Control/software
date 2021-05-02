/*
    UNIMOC - Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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
#include <stdint.h>
#include <array>
#include "ch.hpp"
#include "hal.h"
#include "hardware_interface.hpp"
#include "values.hpp"
#include "pas.hpp"

using namespace chibios_rt;

/**
 * @namespace pedal assist system classes
 */
namespace pas
{

	/**
	 * generic constructor
	 */
	thread::thread(): deadline(0) {};

	/**
	 * @brief Thread main function
	 */
	void thread::main(void)
	{
		setName("PAS");

		deadline = chibios_rt::System::getTime();
		/*
		 * Normal main() thread activity
		 */
		while (TRUE)
		{
			deadline = chibios_rt::System::getTime();

			values.crank.angle = hardware::crank::Angle(settings.crank.pas.counts);
			values.crank.torque = hardware::crank::Torque(settings.crank.offset, settings.crank.gain);

			if(settings.crank.enable)
			{
				// pas mode releases torque only on pedal movement
				if(settings.crank.pas.enable)
				{
					if(std::fabs(values.crank.cadence) > 1.0f)
					{
						values.motor.rotor.setpoint.torque = (values.crank.power / values.motor.rotor.omega);
					}
					else
					{
						values.motor.rotor.setpoint.torque = 0.0f;
					}
				}
				else
				{
					palSetLineMode(LINE_AIN_MOT_TEMP, PAL_MODE_INPUT_PULLUP);
					// reverse on Cadence input
					if(palReadLine(LINE_AIN_MOT_TEMP))
					{
						values.motor.rotor.setpoint.torque = -values.crank.torque;
					}
					else
					{
						values.motor.rotor.setpoint.torque = values.crank.torque;
					}
				}
			}

			sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
		}
	}

} /* namespace pas */

