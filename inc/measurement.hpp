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
#ifndef INC_MEASUREMENT_HPP_
#define INC_MEASUREMENT_HPP_


#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"


/**
 * @namespace measurement data
 */
namespace measurement
{

	/**
	 * run any of the measurement functions with stop handling
	 * @param enable	true if measurement is allowed to be active
	 * @param func		function pointer to one of the run functions of the measurements
	 * @return true if measurement finished regardless if measuement was a success or stop was true
	 */
	bool Run(const bool enable, bool (*func)(bool));

	/**
	 * @namespace resistance measurement values
	 */
	namespace r
	{
		/**
		 * Measure the stators winding resistance.
		 *
		 * The function will slowly increase current in each of the 3 phases direction
		 * until current reaches 75% of current limit.
		 *
		 * @param stop	if true the measurement stops in a clean manner
		 * @return true if measurement finished regardless if measuement was a success or stop was true
		 */
		bool Run(bool stop);
	}

	/**
	 * @namespace inductance measurement values
	 */
	namespace l
	{
		/**
		 * Measure the stators winding inductance with axial components.
		 *
		 * The function will slowly increase current in a fast rotating field
		 * until current reaches the set current limit.
		 *
		 * @param stop	if true the measurement stops in a clean manner
		 * @return true if measurement finished regardless if measuement was a success or stop was true
		 */
		bool Run(bool stop);
	}

	/**
	 * @namespace flux measurement values
	 */
	namespace psi
	{
		/**
		 * Measure the rotor flux.
		 *
		 * The function will turn on current control and set 50% max current then slowly increases omega
		 * until the voltages needed for the current control exceed 25% battery voltage
		 *
		 * @param stop	if true the measurement stops in a clean manner
		 * @return true if measurement finished regardless if measuement was a success or stop was true
		 */
		bool Run(bool stop);
	}
}



#endif /* INC_MEASUREMENT_HPP_ */
