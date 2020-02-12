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
#include "values.hpp"

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
		systems::alpha_beta y = {0.0f, 0.0f};

		///< electric torque
		float m_el = 0.0f;

		///< external load torque
		float m_l = 0.0f;

		/**
		 * @namespace motor rotor system values
		 */
		namespace rotor
		{
			///< Current in rotor frame
			systems::dq i = {0.0f, 0.0f};

			///< Voltage in rotor frame
			systems::dq u = {0.0f, 0.0f};

			///< Admittance vector in rotor frame
			systems::dq y = {0.0f, 0.0f};

			///< angular velocity in rotor frame
			float omega = 0.0f;

			///< rotor angle
			float phi = 0.0f;

			/**
			 * @namespace motor rotor system setpoints
			 */
			namespace setpoint
			{
				///< Current setpoint in rotor frame
				systems::dq i = {0.0f, 0.0f};

				///< angular velocity setpoint in rotor frame
				float omega = 0.0f;

				///< rotor angle setpoint
				float phi = 0.0f;
			} /* namespace setpoint */
		} /* namespace rotor */
	} /* namespace motor */

	/**
	 * @namespace battery values
	 */
	namespace battery
	{
		///< Battery voltage
		float u = 10.0f;

		///< Battery current
		float i = 0.0f;

	} /* namespace battery */

	/**
	 * @namespace converter values.
	 */
	namespace converter
	{
		///< temperature
		float temp = 0.0f;
	} /* namespace converter */
} /* namespace setting */

