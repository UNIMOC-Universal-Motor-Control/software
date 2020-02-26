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
#include "controller.hpp"
#include "filter.hpp"
#include "values.hpp"
#include "settings.hpp"
#include "hardware_interface.hpp"

/**
 * @namespace controller classes
 */
namespace control
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
	 * @brief Pi controller constructor with all essential parameters.
	 *
	 * The motor in rotor frame is G_m = 1/(L_s (s + j w) + R_s)
	 * The controller in rotor frame is G_c = (K_p (s + j w) + K_i)/s
	 *
	 * so to get G_o = 1/s for the open loop, K_p = L_s and K_i = R_s
	 *
	 * @param new_ts                set sample time.
	 * @param rs	                series resistance of the winding
	 * @param l                		series inductance of the winding
	 * @param psi                	rotor flux constant
	 * @param new_limit   			output limit.
	 */
	complex_current::complex_current(const float new_ts, const float rs, const systems::dq l, const float new_psi, const float new_limit):
						ts(new_ts), error_sum{0.0f, 0.0f}, output_unlimited{0.0f, 0.0f}, output{0.0f, 0.0f}, kp{l.d, l.q},
						ki(rs), psi(new_psi), limit(new_limit)
	{}

	/**
	 * @brief calculate regulator equation with feed forward and anti windup.
	 * @param setpoint				setpoint vector
	 * @param act					actual current vector
	 * @param omega					angular velocity in rad/s of the motor
	 * @return						controllers output voltage vector
	 */
	systems::dq complex_current::Calculate(const systems::dq setpoint, const systems::dq act, const float omega)
	{
		systems::dq error = {setpoint.d - act.d, setpoint.q -act.q};

		output_unlimited.d = error_sum.d + error.d * kp.d;  				// regulator equation
		output_unlimited.q = error_sum.q + error.q * kp.q + omega * psi;	// with feed forward

		float length = systems::Length(output_unlimited);

		if(length > limit)													// saturation
		{
			output.d = limit/length * output_unlimited.d;					// scale to fit the limit
			output.q = limit/length * output_unlimited.q;
		}
		else																// normal operation
		{
			std::memcpy(&output, &output_unlimited, sizeof(systems::dq));
		}

		// anti windup strategy is only to limit if
		// integration of the error takes the output
		// more over the limit.
		// if not we are free to integrate
		if(std::memcmp(&output, &output_unlimited, sizeof(systems::dq))
				|| ((error.d * output_unlimited.d) <= 0.0f) || ((error.q * output_unlimited.q) <= 0.0f))
		{
			// (K_p * j w + K_i)/s
			error_sum.d += (error.d * ki - error.q * omega * kp.d) * ts;
			error_sum.q += (error.q * ki + error.d * omega * kp.q) * ts;
		}
		return output;
	}

	/**
	 * @brief smith predictor with complex current controller
	 * 		  constructor with all essential parameters.
	 *
	 * The motor in rotor frame is G_m = 1/(L_s (s + j w) + R_s)
	 * The controller in rotor frame is G_c = (K_p (s + j w) + K_i)/s
	 *
	 * so to get G_o = 1/s for the open loop, K_p = L_s and K_i = R_s
	 *
	 * @param new_ts                set sample time.
	 * @param rs	                series resistance of the winding
	 * @param l                		series inductance of the winding
	 * @param psi                	rotor flux constant
	 * @param new_limit   			output limit.
	 */
	smith_predictor_current::smith_predictor_current(const float new_ts, const float rs, const systems::dq l,
			const float psi, const float new_limit, const float new_hwQ, const float new_hwFc,
			const float new_fQ, const float new_fFc): ts(new_ts), hwQ(new_hwQ), hwFc(new_hwFc), fQ(new_fQ),
			fFc(new_fFc), ctrl(new_ts, rs, l , psi, new_limit),
			fd(filter::lowpass, fFc, fQ, 0.0f), fq(filter::lowpass, fFc, fQ, 0.0f),
			fomega(filter::lowpass, fFc, fQ, 0.0f),
			hwd(filter::lowpass, hwFc, hwQ, 0.0f), hwq(filter::lowpass, hwFc, hwQ, 0.0f),
			sd(filter::lowpass, fFc, fQ, 0.0f), sq(filter::lowpass, fFc, fQ, 0.0f)
	{}

	/**
	 * @brief calculate regulator equation with feed forward and anti windup.
	 * @param setpoint				setpoint vector
	 * @param act					actual current vector
	 * @param omega					angular velocity in rad/s of the motor
	 * @return						controllers output voltage vector
	 */
	systems::dq smith_predictor_current::Calculate(const systems::dq setpoint, const systems::dq act, const float omega)
	{
		systems::dq u;

		return u;
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
	 * @brief calculate FOC current controller
	 */
	void foc::Calculate(void)
	{
		const float _1bysqrt3 = 1.0f / sqrt(3.0f);
		float limit = 0.0;

		// leave limits at zero if current control is inactive.
		if(settings.control.current)
		{
			limit = _1bysqrt3 * values.battery.u;
		}

		// Current controller is limited to 1/sqrt(3)*DC Bus Voltage due to SVM
		d.positive_limit = q.positive_limit = limit;
		d.negative_limit = q.negative_limit = -limit;


		// calculate feedforward
		systems::dq feedforward = {0.0f, 0.0f};
		feedforward.d = settings.motor.Rs * values.motor.rotor.filtered.i.d
				- values.motor.rotor.filtered.omega * settings.motor.L.q * values.motor.rotor.filtered.i.q;

		feedforward.q = settings.motor.Rs * values.motor.rotor.filtered.i.q
				+ values.motor.rotor.filtered.omega * settings.motor.L.d * values.motor.rotor.filtered.i.d
				+ values.motor.rotor.filtered.omega * settings.motor.Psi;

		// only set voltages with active current control
		if(settings.control.current)
		{
			// direct current control
			values.motor.rotor.u.d = d.Calculate(values.motor.rotor.setpoint.i.d, values.motor.rotor.i.d, feedforward.d);

			// quadrature current control
			values.motor.rotor.u.q = q.Calculate(values.motor.rotor.setpoint.i.q, values.motor.rotor.i.q, feedforward.q);
		}
	}
}/* namespace control */



