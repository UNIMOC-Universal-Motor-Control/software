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
 * system global values
 */
typedef struct values_s
{
	/**
	 * motor values
	 */
	struct motor_s
	{
		///< Admittance mean
		systems::alpha_beta y;

		///< electric torque
		float m_el;

		///< external load torque
		float m_l;

		///< motor temperature
		float temp;

		/**
		 * motor rotor system values
		 */
		struct rotor_s
		{
			///< Current in rotor frame
			systems::dq i;

			///< Voltage in rotor frame
			systems::dq u;

			///< Admittance deviation in rotor frame
			systems::dq y;

			///< angular velocity in rotor frame
			float omega;

			///< rotor angle
			float phi;

			/**
			 * motor rotor system setpoints
			 */
			struct setpoint_s
			{
				///< Current setpoint in rotor frame
				systems::dq i;

				///< angular velocity setpoint in rotor frame
				float omega;

				///< rotor angle setpoint
				float phi;

			} setpoint;
		} rotor;
	} motor;

	/**
	 * battery values
	 */
	struct battery_s
	{
		///< Battery voltage
		float u;

		///< Battery current
		float i;
	} battery;

	/**
	 * converter values
	 */
	struct converter_s
	{
		///< powerstage temperature
		float temp;
	} converter;
} values_ts;

extern values_ts values;

#endif /* INC_VALUES_HPP_ */

