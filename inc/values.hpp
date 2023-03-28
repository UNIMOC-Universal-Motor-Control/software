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
#ifndef INC_VALUES_HPP_
#define INC_VALUES_HPP_

#include <cstdint>
#include "systems.hpp"
#include "filter.hpp"

/**
 * system global values
 */
namespace values
{
	/**
	 * motor values
	 */
	namespace motor
	{
		/**
		 * motor torque values
		 */
		namespace torque
		{
			///< electric torque
			extern float electric;

			///< external load torque
			extern float load;
		}

		///< motor temperature
		extern float temp;

		/**
		 * motor hall values
		 */
		namespace hall
		{
			///< hall sensor states
			extern std::uint8_t state;

			///< hall sensor flux estimate
			extern systems::dq flux;
		} /* namespace hall */

		/**
		 * motor phase system values
		 */
		namespace phase
		{
			///< Current in phases
			extern systems::abc i;

			///< motor phase voltage
			extern systems::abc u;

			///< motor phase voltage output
			extern systems::abc u_out;

			///< motor phase voltage output normalized
			extern systems::abc duty;
		} /* namespace phase */

		/**
		 * motor stator system values
		 */
		namespace stator
		{
			///< Current in stator frame
			extern systems::alpha_beta i;

			///< motor stator voltage
			extern systems::alpha_beta u;

			///< motor stator voltage output
			extern systems::alpha_beta u_out;

		} /* namespace stator */

		/**
		 * motor rotor system values
		 */
		namespace rotor
		{
			///< Current in rotor frame
			extern systems::dq i;

			///< Goertzel Frequency analysis instance for direct current
			extern filter::goertzel<128> gid;

			///< Goertzel Frequency analysis instance for quadrature current
			extern filter::goertzel<128> giq;

			///< Voltage in rotor frame
			extern systems::dq u;

			///< Output voltage in rotor frame
			extern systems::dq u_out;

			///< angular velocity in rotor frame
			extern float omega;

			///< rotor angle in q31
			extern std::int32_t phi;

			///< sine cosine values of phi
			extern systems::sin_cos sc;

			/**
			 * rotor flux values
			 */
			namespace flux
			{
				///< rotor flux vector setpoint
				extern systems::dq set;

				///< rotor flux vector actual
				extern systems::dq act;

				///< rotor flux observer feedback
				extern systems::dq C;
			}

			/**
			 * motor rotor system setpoints
			 */
			namespace setpoint
			{
				///< Current setpoint in rotor frame
				extern systems::dq i;

				///< electrical angular velocity setpoint in rotor frame
				extern float omega;

				///< rotor angle setpoint
				extern float phi;

				///< motor electrical torque in Nm
				extern float torque;

				/**
				 * motor rotor system setpoint limits
				 */
				namespace limit
				{
					/**
					 * motor rotor system setpoint limits current
					 */
					namespace i
					{
						extern float min;
						extern float max;
					} /* namespace i */
				} /* namespace limit */
			} /* namespace setpoint */
		} /* namespace rotor */
	} /* namespace stator */

	/**
	 * battery values
	 */
	namespace battery
	{
		///< Battery voltage
		extern float u;

		///< Battery current
		extern float i;
	} /* namespace battery */

	/**
	 * converter values
	 */
	namespace converter
	{
		///< powerstage temperature
		extern float temp;
	} /* namespace converter */

	/**
	 * external sensor values
	 */
	namespace sense
	{
		///< feedback sensor position value
		extern std::uint32_t position;

		///< feedback sensor rotor angle
		extern std::uint32_t angle;
	} /* namespace sense */

	/**
	 * pedal assist system values
	 */
	namespace crank
	{
		///< crank angle in rad
		extern float angle;

		///< pedal cadence in rad/s
		extern float cadence;

		///< pedal torque in Nm
		extern float torque;

		///< pedal power in W
		extern float power;
	} /* namespace crank */
} /* namespace values */

#endif /* INC_VALUES_HPP_ */

