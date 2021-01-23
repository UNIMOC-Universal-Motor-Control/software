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
#include "filter.hpp"
#include "as5048b.hpp"

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
		///< electric torque
		float m_el;

		///< external load torque
		float m_l;

		///< motor temperature
		float temp;

		///< motor phase current
		systems::abc i;

		///< motor phase voltage
		systems::abc u;

		/**
		 * motor rotor system values
		 */
		struct rotor_s
		{
			///< Current in rotor frame
			systems::dq i;

			///< Goertzel Frequency analysis instance for direct current
			filter::goertzel<128> gid;

			///< Goertzel Frequency analysis instance for quadrature current
			filter::goertzel<128> giq;

			///< Voltage in rotor frame
			systems::dq u;

			///< angular velocity in rotor frame
			float omega;

			///< rotor angle
			float phi;

			///< rotor hall sensor states
			std::uint8_t hall;

			///< rotor hall sensor angle error
			float phi_err;

			///< rotor full rotation from start
			std::int32_t rotation;

			///< hfi currents
			systems::dq i_hfi;

			/**
			 * motor rotor system setpoints
			 */
			struct setpoint_s
			{
				///< Current setpoint in rotor frame
				systems::dq i;

				///< electrical angular velocity setpoint in rotor frame
				float omega;

				///< rotor angle setpoint
				float phi;

				///< motor electrical torque in Nm
				float torque;

				/**
				 * motor rotor system setpoint limits
				 */
				struct limit_s
				{
					/**
					 * motor rotor system setpoint limits current
					 */
					struct i_s
					{
						float min;
						float max;
					}i;
				} limit;
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

		///< phase dutys
		systems::abc dutys;
	} converter;

	/**
	 * external sensor values
	 */
	struct sense_s
	{
		///< feedback sensor position value
		std::uint16_t position;

		///< feedback sensor rotor angle
		float angle;
	} sense;
} values_ts;

extern values_ts values;
#endif /* INC_VALUES_HPP_ */

