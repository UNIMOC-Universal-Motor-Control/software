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
#include "controller.hpp"
#include "values.hpp"
#include "settings.hpp"

/**
 * @namespace controller classes
 */
namespace controller
{
	/**
	 * @brief Pi controller constructor with all essential parameters.
	 *
	 * @param new_ts				set sample time.
	 * @param new_kp				proportional gain.
	 * @param new_tn				integral action time.
	 * @param new_positive_limit	positive output limit.
	 * @param new_negative_limit	negative output limit.
	 */
	pi::pi(const float new_ts, const float new_kp, const float new_tn,
			const float new_positive_limit, const float new_negative_limit):
			ts(new_ts), error_sum(0.0f), output_unlimited(0.0f), output(0.0f), kp(new_kp),
			ki(SetKi(kp, ts, new_tn)), positive_limit(new_positive_limit), negative_limit(new_negative_limit)
	{}


	/**
	 * @brief calculate regulator equation with feed forward and anti windup.
	 *
	 * @param error			control loop error.
	 * @param feed_forward 	feed forward control input.
	 * @retval controller output
	 */
	float pi::Calculate(float  error, float feed_forward)
	{
		output_unlimited = error_sum + error * kp + feed_forward;  	// regulator equation

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
	 * @brief FOC controller constructor with all essential parameters.
	 *
	 * @param new_ts                set sample time.
	 * @param new_kp                proportional gain.
	 * @param new_tn                integral action time.
	 */
	foc::foc(const float new_ts, const float new_kp, const float new_tn):
			d(new_ts, new_kp, new_tn, 0.0f, 0.0f), q(new_ts, new_kp, new_tn, 0.0f, 0.0f)
	{}
	/**
	 * @brief calculate foc current controller
	 */
	void foc::Calculate(void)
	{
		const float _1bysqrt3 = 1.0f / sqrt(3.0f);
		float limit = 0.0;

		// leave limits at zero if current control is inactive.
		if(settings::control::current)
		{
			limit = _1bysqrt3 * values::battery::u;
		}

		// Current controller is limited to 1/sqrt(3)*DC Bus Voltage due to SVM
		d.positive_limit = q.positive_limit = limit;
		d.negative_limit = q.negative_limit = -limit;


		// calculate feedforward
		systems::dq feedforward = {0.0f, 0.0f};
		feedforward.d = settings::motor::Rs * values::motor::rotor::setpoint::i.d
				- values::motor::rotor::setpoint::omega * settings::motor::L.q * values::motor::rotor::setpoint::i.q;

		feedforward.q = settings::motor::Rs * values::motor::rotor::setpoint::i.q
				+ values::motor::rotor::setpoint::omega * settings::motor::L.d * values::motor::rotor::setpoint::i.d
				+ values::motor::rotor::setpoint::omega * settings::motor::Psi;

		// only set voltages with active current control
		if(settings::control::current)
		{
			// direct current control
			values::motor::rotor::u.d = d.Calculate(values::motor::rotor::setpoint::i.d - values::motor::rotor::i.d, feedforward.d);

			// quadrature current control
			values::motor::rotor::u.q = q.Calculate(values::motor::rotor::setpoint::i.q - values::motor::rotor::i.q, feedforward.q);
		}
	}
}/* namespace controller */



