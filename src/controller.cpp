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
	 * @param new_kp				proportional gain.
	 * @param new_tn				integral action time.
	 * @param new_positive_limit	positive output limit.
	 * @param new_negative_limit	negative output limit.
	 */
	pi::pi(const float new_kp, const float new_tn,
			const float new_positive_limit, const float new_negative_limit):
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
	 * @brief Pi controller constructor with all essential parameters.
	 *
	 * The motor in rotor frame is G_m = 1/(L_s (s + j w) + R_s)
	 * The controller in rotor frame is G_c = (K_p (s + j w) + K_i)/s
	 *
	 * so to get G_o = 1/s for the open loop, K_p = L_s and K_i = R_s
	 *
	 * @param rs	                series resistance of the winding
	 * @param l                		series inductance of the winding
	 * @param psi                	rotor flux constant
	 * @param new_limit   			output limit.
	 */
	complex_current::complex_current(const float rs, const systems::dq l, const float new_psi, const float new_limit):
						error_sum{0.0f, 0.0f}, output_unlimited{0.0f, 0.0f}, output{0.0f, 0.0f}, kp{l.d, l.q},
						ki(rs), psi(new_psi), limit(new_limit)
	{}

	/**
	 * @brief calculate regulator equation with feed forward and anti windup.
	 * @param setpoint				setpoint vector
	 * @param act					actual current vector
	 * @param omega					angular velocity in rad/s of the motor
	 * @param gain					controller gain factor
	 * @return						controllers output voltage vector
	 */
	systems::dq complex_current::Calculate(const systems::dq setpoint, const systems::dq act, const float omega, const float gain)
	{
		systems::dq error = {gain * (setpoint.d - act.d), gain * (setpoint.q -act.q)};

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
		if(0 == std::memcmp(&output, &output_unlimited, sizeof(systems::dq))
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
	 * @param new_rs                series resistance of the winding
	 * @param new_l            		series inductance of the winding
	 * @param new_psi              	rotor flux constant
	 * @param new_limit   			output limit.
	 * @param hwf					coefficients for hardware filter model
	 * @param swf					coefficients for software filter and its model
	 */
	smith_predictor_current::smith_predictor_current(const float new_rs, const systems::dq new_l, const float new_psi,
			const float new_limit, const filter::biquad_coefficients_ts hwf, const filter::biquad_coefficients_ts swf):
			rs(new_rs), l(new_l), psi(new_psi), hwf(hwf), swf(swf), ctrl(rs, l , psi, new_limit),
			fd(swf), fq(swf), fomega(swf), hwd(hwf), hwq(hwf), sd(swf), sq(swf)
	{}

	/**
	 * @brief calculate regulator equation with feed forward and anti windup.
	 * @param setpoint				setpoint vector
	 * @param act					actual current vector
	 * @param omega					angular velocity in rad/s of the motor
	 * @param gain					controller gain factor
	 * @return						controllers output voltage vector
	 */
	systems::dq smith_predictor_current::Calculate(const systems::dq setpoint, const systems::dq act, const float omega, const float gain)
	{
		values.motor.rotor.filtered.omega = fomega.Process(omega);
		values.motor.rotor.filtered.i.d = fd.Process(act.d);
		values.motor.rotor.filtered.i.q = fq.Process(act.q);

		// calculate the model output delayed to the filtered current.
		// using the old i values to take the deadtime of one controlcycle into account!
		i_filtered.d = sd.Process(hwd.Process(i.d));
		i_filtered.q = sq.Process(hwq.Process(i.q));

		// current prediction model
		i.d += (u.d - rs*i.d - l.q*values.motor.rotor.filtered.omega*i.q)*ts;
		i.q += (u.q - rs*i.q - l.d*values.motor.rotor.filtered.omega*i.d - psi*values.motor.rotor.filtered.omega)*ts;

		// correct the current prediction with the error of the filtered
		systems::dq i_feedback =
		{
				i.d + values.motor.rotor.filtered.i.d - i_filtered.d,
				i.q + values.motor.rotor.filtered.i.q - i_filtered.q
		};

		// update the limit for the controller
		ctrl.limit = limit;
		u = ctrl.Calculate(setpoint, i_feedback , values.motor.rotor.filtered.omega, gain);

		return u;
	}

	/**
	 * @brief constructor of the foc with all essential parameters.
	 *
	 * @param hwf					coefficients for hardware filter model
	 * @param swf					coefficients for software filter and its model
	 *
	 */
	foc::foc(const filter::biquad_coefficients_ts hwf, const filter::biquad_coefficients_ts swf):
			ctrl(settings.motor.rs, settings.motor.l, settings.motor.psi, 0.0f,	hwf, swf)
	{}


	/**
	 * @brief calculate FOC current controller
	 */
	void foc::Calculate(void)
	{
		// only set voltages with active current control
		if(settings.control.current.active)
		{
			constexpr float _1bysqrt3 = 1.0f / std::sqrt(3.0f);
			// Current controller is limited to 1/sqrt(3)*DC Bus Voltage due to SVM
			ctrl.limit = _1bysqrt3 * values.battery.u;

			values.motor.rotor.u = ctrl.Calculate(values.motor.rotor.setpoint.i, values.motor.rotor.i, values.motor.rotor.omega, settings.control.current.gain);
		}
	}

	/**
	 * R1 = 4k7, R2 = 47k, R3 = 4k7, C1 = 10n, C2 = 1n
	 * Q = 0.47619047619048, Fc = 3386.2753849339
	 */
	constexpr filter::biquad_coefficients_ts hw_coeff = filter::BiquadCalc(filter::biquad_type_et::lowpass, 0.47619047619048f, 3386.2753849339f/hardware::Fc, 0.0f);
	constexpr filter::biquad_coefficients_ts sw_coeff = filter::BiquadCalc(filter::biquad_type_et::lowpass, 0.5f, 0.01f, 0.0f);

	/**
	 * generic constructor
	 */
	thread::thread():flux(), mech(settings.observer.Q, settings.observer.R), foc(hw_coeff, sw_coeff)
	{}

	/**
	 * @brief Thread main function
	 */
	void thread::main(void)
	{
		setName("Control");


		/*
		 * Normal main() thread activity
		 */
		while (TRUE)
		{
			systems::sin_cos sin_cos;
			float angle;

			/* Checks if an IRQ happened else wait.*/
			chEvtWaitAny((eventmask_t)1);

			values.battery.u = hardware::adc::voltage::DCBus();

			hardware::adc::current::Value(values.motor.i);

			// calculate the sine and cosine of the new angle
			angle = values.motor.rotor.phi
					+ values.motor.rotor.omega * hardware::Tc;

			// calculate new
			systems::SinCos(angle, sin_cos);

			// convert 3 phase system to ortogonal
			i_ab = systems::transform::Clark(values.motor.i);
			// convert current samples from clark to rotor frame;
			values.motor.rotor.i = systems::transform::Park(i_ab, sin_cos);


			if(management::observer::flux)
			{
				// calculate the flux observer
				float flux_error = flux.Calculate(sin_cos);
				mech.Update(flux_error, correction);

				// predict motor behavior
				observer::mechanic::Predict();
				// correct the prediction
				observer::mechanic::Correct(correction);
			}
			else
			{
				values.motor.rotor.phi += values.motor.rotor.omega * hardware::Tc;
			}

			if(management::control::current)
			{
				// calculate the field orientated controllers
				foc.Calculate();
			}

			// transform the voltages to stator frame
			u_ab = systems::transform::InversePark(values.motor.rotor.u, sin_cos);

			// transform to ab system
			values.motor.u = systems::transform::InverseClark(u_ab);

			//scale the voltages
			if(values.battery.u > 10.0f)
			{
				values.motor.u.a /= values.battery.u;
				values.motor.u.b /= values.battery.u;
				values.motor.u.c /= values.battery.u;
			}
			else
			{
				std::memset(&values.motor.u, 0, sizeof(systems::abc));
			}

			hardware::pwm::Dutys(values.motor.u);
		}

	}
}/* namespace control */



