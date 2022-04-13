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
 * RTT Scope
 */
namespace scope
{
	/**
	 * SEGGER RTT J-Scope definitions
	 */
	constexpr std::uint32_t NUM_OF_RTT_SCOPE_CHANNELS = 8;

	constexpr std::uint32_t BUFFER_SIZE = 4096;


	typedef struct scope_s
	{
		//std::uint32_t timestamp;
		float ch[NUM_OF_RTT_SCOPE_CHANNELS];
	}scope_ts;

	typedef struct string_s
	{
		static constexpr char start[] = "JScope_";

		constexpr string_s(void) : str()
		{
			for(std::uint32_t i = 0; i < sizeof(start); i++)
			{
				str[i] = start[i];
			}

			for(std::uint32_t i = 0; i < NUM_OF_RTT_SCOPE_CHANNELS; i++)
			{
				str[sizeof(start) - 1 + i *2] = 'f';
				str[sizeof(start) + i *2] = '4';
			}
			str[sizeof(start) - 1 + NUM_OF_RTT_SCOPE_CHANNELS *2] = 0;
		}
		char str[NUM_OF_RTT_SCOPE_CHANNELS * 2 + sizeof(start)];
	}string_ts;


	extern scope_ts sample;

	extern const string_ts string;

	extern std::uint8_t buffer[BUFFER_SIZE];
}

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

			///< rotor angle in deg
			extern float angle;

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
		extern std::uint16_t position;

		///< feedback sensor rotor angle
		extern float angle;
	} /* namespace sense */
} /* namespace values */





#endif /* INC_VALUES_HPP_ */

