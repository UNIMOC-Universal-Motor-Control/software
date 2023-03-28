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
#include "values.hpp"
#include "hardware_interface.hpp"

#pragma GCC push_options
#pragma GCC optimize ("-O0")

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
			float electric = 0.0f;

			///< external load torque
			float load = 0.0f;
		}

		///< motor temperature
		float temp = 0.0f;

		/**
		 * motor hall values
		 */
		namespace hall
		{
			///< hall sensor states
			std::uint8_t state = 0;

			///< hall sensor flux estimate
			systems::dq flux = {0.0f, 0.0f};
		}

		/**
		 * motor phase system values
		 */
		namespace phase
		{
			///< Current in phases
			systems::abc i = {0.0f, 0.0f, 0.0f};

			///< motor phase voltage
			systems::abc u = {0.0f, 0.0f, 0.0f};

			///< motor phase voltage output
			systems::abc u_out = {0.0f, 0.0f, 0.0f};

			///< motor phase voltage output normalized
			systems::abc duty = {0.0f, 0.0f, 0.0f};
		} /* namespace phase */

		/**
		 * motor stator system values
		 */
		namespace stator
		{
			///< phase current in stator frame
			systems::alpha_beta i = {0.0f, 0.0f};

			///< phase voltage in stator frame
			systems::alpha_beta u = {0.0f, 0.0f};

			///< motor stator voltage for pwm output
			systems::alpha_beta u_out = {0.0f, 0.0f};
		} /* namespace stator */

		/**
		 * motor rotor system values
		 */
		namespace rotor
		{
			///< Current in rotor frame
			systems::dq i = {0.0f, 0.0f};

			///< Goertzel Frequency analysis instance for direct current
			filter::goertzel<128> gid;

			///< Goertzel Frequency analysis instance for quadrature current
			filter::goertzel<128> giq;

			///< Voltage in rotor frame
			systems::dq u = {0.0f, 0.0f};

			///< Voltage in rotor frame for pwm output
			systems::dq u_out = {0.0f, 0.0f};

			///< angular velocity in rotor frame
			float omega = 0.0f;

			///< rotor angle in q31
			std::int32_t phi = 0;

			///< sine cosine values of phi
			systems::sin_cos sc = {0.0f, 0.0f};

			/**
			 * motor rotor flux values
			 */
			namespace flux
			{
				///< motor rotor flux vector setpoint
				systems::dq set = {0.0f, 0.0f};

				///< motor rotor flux vector actual
				systems::dq act = {0.0f, 0.0f};

				///< rotor flux observer feedback
				systems::dq C = {500.0f, 500.0f};

			}

			/**
			 * motor rotor system setpoints
			 */
			namespace setpoint
			{
				///< Current setpoint in rotor frame
				systems::dq i = {0.0f, 0.0f};

				///< electrical angular velocity setpoint in rotor frame
				float omega = 0.0f;

				///< rotor angle setpoint
				float phi  = 0.0f;

				///< motor electrical torque in Nm
				float torque  = 0.0f;

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
						float min  = 0.0f;
						float max  = 0.0f;
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
		float u  = 0.0f;

		///< Battery current
		float i = 0.0f;
	} /* namespace battery */

	/**
	 * converter values
	 */
	namespace converter
	{
		///< powerstage temperature
		float temp = 0.0f;
	} /* namespace converter */

	/**
	 * external sensor values
	 */
	namespace sense
	{
		///< feedback sensor position value
		std::uint32_t position = 0;

		///< feedback sensor rotor angle
		std::uint32_t angle = 0;
	} /* namespace sense */

	/**
	 * pedal assist system values
	 */
	namespace crank
	{
		///< crank angle in rad
		float angle = 0.0f;

		///< pedal cadence in rad/s
		float cadence = 0.0f;

		///< pedal torque in Nm
		 float torque = 0.0f;

		///< pedal power in W
		float power = 0.0f;
	} /* namespace crank */
} /* namespace values */

#pragma GCC pop_options

