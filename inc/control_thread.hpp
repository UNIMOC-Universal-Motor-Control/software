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
#ifndef INC_CONTROL_THREAD_HPP_
#define INC_CONTROL_THREAD_HPP_

#include <cstdint>
#include <cmath>
#include <climits>
#include "ch.hpp"
#include "filter.hpp"
#include "systems.hpp"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"
#include "observer.hpp"
#include "controller.hpp"
#include "management.hpp"
#include "hardware_interface.hpp"

/**
 * @namespace controller classes
 */
namespace control
{
	/**
	 * generic FOC controller thread
	 */
	class thread : public chibios_rt::BaseStaticThread<2048>
	{
	private:
		static constexpr float _3by2 = 3.0f/2.0f;
		observer::flux       	flux;
		observer::hall       	hall;
		observer::mechanic 		mech;
		control::foc      		foc;
		sensor::as5048b 		as5048;
		std::array<float, 3>   	correction;
		filter::low_pass		uq;
		filter::low_pass		ubat;
		filter::low_pass		w;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> i_abc;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> i_abc_ac;
		std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES> i_ab;
		std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES> i_ab_ac;
		std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES> u_ab;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> dutys;

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
} /* namespace control */

#endif /* INC_CONTROL_THREAD_HPP_ */

