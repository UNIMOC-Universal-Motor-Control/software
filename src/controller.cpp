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
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "controller.hpp"
#include "filter.hpp"
#include "values.hpp"
#include "settings.hpp"
#include "freemaster_wrapper.hpp"
#include "hardware_interface.hpp"

/**
 * @namespace controller classes
 */
namespace control
{

	/**
	 * @brief Pi controller constructor with all essential parameters.
	 *
	 * @param new_kp				proportional gain.
	 * @param new_tn				integral action time.
	 * @param new_positive_limit	positive output limit.
	 * @param new_negative_limit	negative output limit.
	 * @param ts					sampling time
	 */
	pi::pi(const float new_kp, const float new_tn,
			const float new_positive_limit, const float new_negative_limit, const float ts):
			error_sum(0.0f), output_unlimited(0.0f), output(0.0f), kp(new_kp),
			ki(SetKi(kp, ts, new_tn)), positive_limit(new_positive_limit), negative_limit(new_negative_limit)
	{}


	/**
	 * @brief calculate regulator equation with feed forward and anti windup.
	 *
	 * @param error			control loop error.
	 * @param feed_forward 	feed forward control input.
	 * @retval controller output
	 */
	float pi::Calculate(const float setpoint, const float actual, const float feedforward)
	{
		float error = setpoint - actual;

		output_unlimited = error_sum + error * kp + feedforward;  	// regulator equation

		if(output_unlimited > positive_limit)				// upper saturation
		{
			output = positive_limit;
		}
		else if(output_unlimited < negative_limit)			// lower saturation
		{
			output = negative_limit;
		}
		else												// normal operation
		{
			output = output_unlimited;
		}

		// anti windup strategy is only to limit if
		// integration of the error takes the output
		// more over the limit.
		// if not we are free to integrate
		if((output == output_unlimited) || ((error * output_unlimited) <= 0.0F))
		{
			error_sum += error * ki;   // ki already takes sampling time ts into account
		}
		return output;
	}

	/**
	 * @brief constructor of the foc with all essential parameters.
	 */
	foc::foc(void):
			ctrl_d(0.0f, 0.0f, 0.0f, 0.0f, hardware::Tc()),
			ctrl_q(0.0f, 0.0f, 0.0f, 0.0f, hardware::Tc())
	{}


	/**
	 * @brief calculate FOC current controller
	 * @param setpoint current vector
	 */
	void foc::Calculate(systems::dq& setpoint)
	{
		using namespace values;
		using namespace values::motor;

		float limit = _1bysqrt3 * battery::u*0.95f;

		// Current controller is limited to 1/sqrt(3)*DC Bus Voltage due to SVM
		// d current controller is master for voltage limit
		ctrl_d.positive_limit = limit;
		ctrl_d.negative_limit = -limit;

		float length = systems::Length(rotor::u);

		if(length > ctrl_d.positive_limit)									// saturation
		{
			float rest = std::sqrt(length*length - rotor::u.d * rotor::u.d);
			// q current controller is only fully free if we are not in voltage limit
			ctrl_q.positive_limit = rest;
			ctrl_q.negative_limit = -rest;
		}
		else																// normal operation
		{
			// q current controller is only fully free if we are not in voltage limit
			ctrl_q.positive_limit = limit;
			ctrl_q.negative_limit = -limit;
		}

		systems::dq feedforward = {0.0f, 0.0f};

		if(settings.control.current.feedforward)
		{
			feedforward.d = settings.motor.rs*setpoint.d - rotor::omega * setpoint.q * settings.motor.l.q;
			feedforward.q = settings.motor.rs*setpoint.q + rotor::omega * setpoint.d * settings.motor.l.d + rotor::omega*settings.motor.psi;
		}

		rotor::u.d = ctrl_d.Calculate(setpoint.d, rotor::i.d, feedforward.d);
		rotor::u.q = ctrl_q.Calculate(setpoint.q, rotor::i.q, feedforward.q);
	}

}/* namespace control */





