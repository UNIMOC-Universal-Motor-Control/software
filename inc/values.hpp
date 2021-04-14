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
namespace values
{
	/**
	 * motor values
	 */
	namespace motor
	{
		///< electric torque
		extern float m_el;

		///< external load torque
		extern float m_l;

		///< motor temperature
		extern float temp;

		/**
		 * motor hall values
		 */
		namespace hall
		{
			///< hall sensor states
			extern std::uint8_t state;

			///< hall sensor sine and cosine values
			extern systems::sin_cos sc;
		} /* namespace hall */

		/**
		 * motor phase system values
		 */
		namespace phase
		{
			///< Current in phases
			extern systems::abc i;

			///< motor phase current derivatives
			extern systems::abc di;

			///< motor phase voltage
			extern systems::abc u;
		} /* namespace phase */

		/**
		 * motor stator system values
		 */
		namespace stator
		{
			///< Current in stator frame
			extern systems::alpha_beta i;

			///< motor stator current derivatives
			extern systems::alpha_beta di;

			///< motor stator voltage
			extern systems::alpha_beta u;

			///< motor stator admittance
			extern systems::alpha_beta y;

			///< motor stator admittance vector
			extern systems::alpha_beta yd;

			/**
			 * motor stator flux values
			 */
			namespace flux
			{
				///< motor stator flux vector setpoint
				extern systems::alpha_beta set;

				///< motor stator flux vector actual
				extern systems::alpha_beta act;
			}
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

			///< angular velocity in rotor frame
			extern float omega;

			///< rotor angle in q31
			extern std::int32_t phi;

			///< rotor angle in deg
			extern float angle;

			///< sine cosine values of phi
			extern systems::sin_cos sc;

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
		extern std::uint16_t position;

		///< feedback sensor rotor angle
		extern float angle;
	} /* namespace sense */
} /* namespace values */

#endif /* INC_VALUES_HPP_ */

