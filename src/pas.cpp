/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2023 Alexander <tecnologic86@gmail.com> Evers

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
#include "observer.hpp"

using namespace chibios_rt;


/**
 * @namespace pedal assist system classes
 */
namespace pas
{

	///< observer for cadence
	observer::mechanic mech;
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

			if(settings.crank.enable)
			{
		        const float ts = hardware::Tc();
		        const float tsj = ts/settings.crank.observer.J;

				values::crank::angle = hardware::crank::angle(settings.crank.pas.counts);
				values::crank::torque = hardware::crank::torque(settings.crank.offset, settings.crank.gain);

		        // omega
				values::crank::cadence += tsj * (values::crank::torque - torque);

		        if(values::crank::cadence > 20.0f)	// 200rpm
		        {
		        	values::crank::cadence > 20.0f;
		        }
		        else if(values::crank::cadence < -20.0f)
		        {
		        	values::crank::cadence > -20.0f;
		        }

		        if(!std::isfinite(alues::crank::cadence ))
		        {
		        	values::crank::cadence = 0.0f;
		        }

		        // integrate omega for phi
		        rotor::phi += unit::Q31R(alues::crank::cadence * ts);

				angle_error = angle - values::crank::angle;

				mech.Update(settings.crank.observer.Q, settings.crank.observer.R, angle_error, out_error);

				// pas mode releases torque only on pedal movement
				if(settings.crank.pas.enable)
				{
					if(std::fabs(values::crank::cadence) > 1.0f)
					{
						values::motor::rotor::setpoint::torque = (values::crank::power / values::motor::rotor::omega);
					}
					else
					{
						values::motor::rotor::setpoint::torque = 0.0f;
					}
				}
				sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
			}
			else
			{
				sleepUntilWindowed(deadline, deadline + TIME_MS2I(2000));
			}
		}
	}

} /* namespace pas */

