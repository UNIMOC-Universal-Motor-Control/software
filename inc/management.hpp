/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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
#ifndef INC_MANAGEMENT_HPP_
#define INC_MANAGEMENT_HPP_


#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"


/**
 * @namespace controller management classes
 */
namespace management
{
	/**
	 * @namespace observer flags
	 */
	namespace observer
	{
		///< release flux observer
		extern bool flux;

		///< release high frequency injection observer
		extern bool hfi;

		///< release mechanic observer
		extern bool hall;
	}

	/**
	 * @namespace controller flags
	 */
	namespace control
	{
		///< release current control
		extern bool current;

		///< feedforward omega
		extern bool feedforward;

		///< release speed control
		extern bool speed;

		///< release position control
		extern bool position;
	}

	/**
	 * @namespace measurement data
	 */
	namespace measure
	{
		///< measure all parameters
		extern bool all;

		///< measure stator resistance
		extern bool resitance;

		///< measure stator inductance
		extern bool inductance;

		///< measure rotor flux
		extern bool flux;

		///< measure hall sensor state positions
		extern bool hall;
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
			extern filter::low_pass electric;

			///< external load torque
			extern filter::low_pass load;
		}

		///< motor temperature
		extern filter::low_pass temp;

		/**
		 * motor rotor system values
		 */
		namespace rotor
		{
			///< Current in rotor frame
			extern filter::low_pass_dq i;

			///< Voltage in rotor frame
			extern filter::low_pass_dq u;

			///< angular velocity in rotor frame
			extern filter::low_pass omega;

			///< rotor flux vector actual
			extern filter::low_pass_dq flux;
		} /* namespace rotor */
	} /* namespace motor */

	/**
	 * battery values
	 */
	namespace battery
	{
		///< Battery voltage
		extern filter::low_pass u;

		///< Battery current
		extern filter::low_pass i;
	} /* namespace battery */

	/**
	 * converter values
	 */
	namespace converter
	{
		///< powerstage temperature
		extern filter::low_pass temp;

		///< analog input  filter
		extern filter::low_pass input;
	} /* namespace converter */

	/**
	 * controller management thread
	 */
	class thread : public chibios_rt::BaseStaticThread<1024>
	{
	private:
		static constexpr systime_t CYCLE_TIME = TIME_MS2I(1);

		///< time constant for control value filters (time 10 of the CYCLE_TIME
		static constexpr float Ts = 10e-3f;

		systime_t 				deadline;
		std::uint32_t			delay;

		enum state_e
		{
			STARTUP,
			CURRENT_OFFSETS,
			RUN,
			MEASURE_RS,
			MEASURE_LS,
			MEASURE_PSI,
		} sequencer;

		/**
		 * derate control input envelope
		 * @param limit	value to end derating
		 * @param envelope positive value sets the envelope below the limit, negative above the limit
		 * @param actual actual value
		 * @return 1 when no derating active and 1 to 0 when in envelope and 0 when above limit
		 */
		float Derate(const float limit, const float envelope, const float actual);

		/**
		 * limit the input value
		 * @param[in/out] in input value
		 * @param min minimal value
		 * @param max maximal value
		 * @return true when value is out of limits
		 */
		bool Limit(float& in, const float min, const float max);

		/**
		 * Limit the current setpoint according to temp, voltage, and current limits
		 * @param setpoint[in/out] current setpoint
		 */
		void LimitCurrentSetpoint(systems::dq& setpoint);
		/**
		 * compensate the commanded voltage for the error introduced by PWM deadtime
		 */
		void DeadtimeCompensation(void);

		/**
		 * get the mapped throttle input signal
		 * @param setpoint
		 */
		void SetThrottleSetpoint(systems::dq& setpoint);

		/**
		 * calculate analog input signal with dead zones in bidirectional manner
		 * @param input 0-1 input
		 * @return	-1-1 output with dead zones at 0 and +-1
		 */
		float BiAnalogThrottleDeadzone(const float input);

		/**
		 * calculate analog input signal with deadzones
		 * @param input 0-1 input
		 * @return	0-1 output with dead zones
		 */
		float UniAnalogThrottleDeadzone(const float input);

		/**
		 * Switch Flag with hysteresis
		 * @param value	current values
		 * @param limit limit with hysteresis around
		 * @param hysteresis corridor of hysteresis +-
		 * @param flag current state
		 * @return new state of the flag
		 */
		bool Hysteresis(const float value, const float limit, const float hysteresis, const bool flag);


	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		/**
		 * generic constructor
		 */
		thread();
	};

} /* namespace management */


#endif /* INC_MANAGEMENT_HPP_ */
