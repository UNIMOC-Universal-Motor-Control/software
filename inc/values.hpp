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
#ifndef INC_VALUES_HPP_
#define INC_VALUES_HPP_

#include <cstdint>
#include "systems.hpp"


/**
 * @namespace system values
 */
namespace values
{
	/**
	 * @namespace motor values
	 */
	namespace motor
	{
		///< Admittance mean
		extern systems::alpha_beta y;

		///< electric torque
		extern float m_el;

		///< external load torque
		extern float m_l;

		/**
		 * @namespace motor rotor system values
		 */
		namespace rotor
		{
			///< Current in rotor frame
			extern systems::dq i;

			///< Voltage in rotor frame
			extern systems::dq u;

			///< Admittance deviation in rotor frame
			extern systems::dq y;

			///< angular velocity in rotor frame
			extern float omega;

			///< rotor angle
			extern float phi;

			///< sine and cosine of phi
			extern systems::sin_cos sin_cos;

			/**
			 * @namespace motor rotor system setpoints
			 */
			namespace setpoint
			{
				///< Current setpoint in rotor frame
				extern systems::dq i;

				///< angular velocity setpoint in rotor frame
				extern float omega;

				///< rotor angle setpoint
				extern float phi;

			} /* namespace setpoint */
		} /* namespace rotor */
	} /* namespace motor */

	/**
	 * @namespace battery values
	 */
	namespace battery
	{
		///< Battery voltage
		extern float u;

		///< Battery current
		extern float i;

	} /* namespace battery */

	/**
	 * @namespace converter values
	 */
	namespace converter
	{
		///< phase temperatures
		extern float temp;

	} /* namespace converter */
} /* namespace values */

#endif /* INC_VALUES_HPP_ */

