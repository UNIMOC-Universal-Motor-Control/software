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
	 * derate control input envelope
	 * @param limit	value to end derating
	 * @param envelope positive value sets the envelope below the limit, negative above the limit
	 * @param actual actual value
	 * @return 1 when no derating active and 1 to 0 when in envelope and 0 when above limit
	 */
	float Derate(const float limit, const float envelope, const float actual)
	{
		const float start = limit - envelope;

		float derating = (actual - start)/envelope;

		if(derating < 0.0f) derating = 0.0f;
		if(derating > 1.0f) derating = 1.0f;

		// cut off the derating value from the maximum
		return (1.0f - derating);
	}

	/**
	 * limit the input value
	 * @param[in/out] in input value
	 * @param min minimal value
	 * @param max maximal value
	 * @return true when value is out of limits
	 */
	bool Limit(float& in, const float min, const float max)
	{
		bool did_trunc = false;

		if (in > max)
		{
			in = max;
			did_trunc = true;

		}
		else if (in < min)
		{
			in = min;
			did_trunc = true;
		}

		return did_trunc;
	}

	/**
	 * @brief Pi controller constructor with all essential parameters.
	 *
	 * @param new_kp				proportional gain.
	 * @param new_tn				integral action time.
	 * @param new_positive_limit	positive output limit.
	 * @param new_negative_limit	negative output limit.
	 */
	pi::pi(const float new_kp, const float new_tn,
			const float new_positive_limit, const float new_negative_limit, const float ts):
			ts(ts), error_sum(0.0f), output_unlimited(0.0f), output(0.0f), kp(new_kp),
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
	foc::foc(void):Rs(settings.motor.rs), Ls((settings.motor.l.d + settings.motor.l.q) * 0.5f), PsiM(settings.motor.psi),
			ctrl_d(0.0f, 0.0f, 0.0f, 0.0f, hardware::Tc),
			ctrl_q(0.0f, 0.0f, 0.0f, 0.0f, hardware::Tc)
	{}


	/**
	 * @brief calculate FOC current controller
	 */
	void foc::Calculate(void)
	{
		float limit = _1bysqrt3 * values.battery.u*0.95f;

		// Current controller is limited to 1/sqrt(3)*DC Bus Voltage due to SVM
		// d current controller is master for voltage limit
		ctrl_d.positive_limit = limit;
		ctrl_d.negative_limit = -limit;

		float length = systems::Length(values.motor.rotor.u);

		if(length > ctrl_d.positive_limit)									// saturation
		{
			float rest = std::sqrt(length*length - values.motor.rotor.u.d * values.motor.rotor.u.d);
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
			feedforward.d = Rs*values.motor.rotor.setpoint.i.d - values.motor.rotor.omega * values.motor.rotor.setpoint.i.q * Ls;
			feedforward.q = Rs*values.motor.rotor.setpoint.i.q + values.motor.rotor.omega * values.motor.rotor.setpoint.i.d * Ls
					+ values.motor.rotor.omega*PsiM;
		}

		values.motor.rotor.u.d = ctrl_d.Calculate(values.motor.rotor.setpoint.i.d, values.motor.rotor.i.d, feedforward.d);
		values.motor.rotor.u.q = ctrl_q.Calculate(values.motor.rotor.setpoint.i.q, values.motor.rotor.i.q, feedforward.q);
	}

	/**
	 * generic constructor
	 */
	thread::thread():flux(), hfi(),foc(),
			omega_max(0.01f, 1.0f, 0.0f, 0.0f, hardware::Tc),
			omega_min(0.01f, 1.0f, 0.0f, 0.0f, hardware::Tc)
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
			systems::sin_cos phi_sc;
			systems::sin_cos cur_sc;
			float angle;

			/* Checks if an IRQ happened else wait.*/
			chEvtWaitAny((eventmask_t)1);

			values.battery.u = hardware::adc::voltage::DCBus();

			hardware::adc::current::Value(values.motor.i);

			// calculate the sine and cosine of the new angle
			angle = values.motor.rotor.phi
					+ values.motor.rotor.omega * hardware::Tf;

			// calculate new sine and cosine for the reference system
			systems::SinCos(values.motor.rotor.phi, phi_sc);

			// calculate new sine and cosine for the current system delayed be filters
			systems::SinCos(angle, cur_sc);

			// convert 3 phase system to ortogonal
			i_ab = systems::transform::Clark(values.motor.i);
			// convert current samples from clark to rotor frame;
			values.motor.rotor.i = systems::transform::Park(i_ab, cur_sc);

			// calculate battery current from power equality
			values.battery.i = (values.motor.rotor.u.d * values.motor.rotor.i.d
					+ values.motor.rotor.u.q * values.motor.rotor.i.q)/values.battery.u;

			systems::sin_cos hall;
			hardware::adc::hall::Angle(hall);
			values.motor.rotor.hall_err = hall.sin*phi_sc.cos - hall.cos*phi_sc.sin;

			// predict motor behavior
			observer::mechanic::Predict();

			if(management::observer::flux)
			{
				// calculate the flux observer
				flux.Calculate(phi_sc, correction);

				// correct the prediction
				observer::mechanic::Correct(correction);
			}

			if(management::observer::hfi)
			{
				// calculate the flux observer
				hfi.Calculate(values.motor.rotor.i, correction);

				// correct the prediction
				observer::mechanic::Correct(correction);
			}

			if(management::control::current)
			{
				float torque_factor = _3by2 * settings.motor.psi;

				values.motor.rotor.setpoint.i.d = 0.0f;
				values.motor.rotor.setpoint.i.q =
						values.motor.rotor.setpoint.torque/(torque_factor);

				// starting help for traction drives
				if(std::fabs(values.motor.rotor.setpoint.i.q) > settings.observer.mech.i_min
						&& settings.motor.i_start > 0.1f)
				{
					values.motor.rotor.setpoint.i.d +=
							settings.motor.i_start * std::exp(- std::fabs(values.motor.rotor.omega/200.0f));
				}

				float ratio = values.battery.u / uq.Calculate(values.motor.rotor.u.q);
				float min = -settings.motor.limits.i;
				float max =  settings.motor.limits.i;

				// deadzone for battery current limiter of 1W
				if(std::fabs(values.motor.rotor.u.q * values.motor.rotor.setpoint.i.q) > 1.0f)
				{
					if(ratio > 1e-3f)
					{
						min = ratio * -settings.battery.limits.i.charge;
						max = ratio * settings.battery.limits.i.drive;
					}
					else if(ratio < -1e-3f)
					{
						min = ratio * settings.battery.limits.i.drive;
						max = ratio * -settings.battery.limits.i.charge;
					}

					if(min < -settings.motor.limits.i) min = -settings.motor.limits.i;
					if(max > settings.motor.limits.i) max =  settings.motor.limits.i;
				}


				// derate by temperature and voltage
				std::array<float, 2> derate;
				derate[0] = Derate(settings.converter.limits.temperature,
						settings.converter.derating.temprature, values.converter.temp);
				derate[1] = Derate(settings.battery.limits.voltage,
						-settings.converter.derating.voltage, values.battery.u);
				//				derate[1] = Derate(settings.motor.limits.temperature,
				//						settings.converter.derating.temprature, values.motor.temp);

				// always use the minimal derating possible
				float derating = *std::min_element(derate.begin(), derate.end());

				// derate the limits
				min *= derating;
				max *= derating;

				// omega limiter
				omega_min.negative_limit = min;
				omega_min.positive_limit = 0.0f;
				values.motor.rotor.setpoint.limit.i.min = omega_min.Calculate(-settings.motor.limits.omega, values.motor.rotor.omega, min);

				omega_max.negative_limit = 0.0f;
				omega_max.positive_limit = max;
				values.motor.rotor.setpoint.limit.i.max = omega_max.Calculate(settings.motor.limits.omega, values.motor.rotor.omega, max);

				Limit(values.motor.rotor.setpoint.i.q, values.motor.rotor.setpoint.limit.i.min, values.motor.rotor.setpoint.limit.i.max);

				// calculate the field orientated controllers
				foc.Calculate();
			}
			else
			{
				foc.Reset();
				foc.SetParameters(settings.motor.rs, settings.motor.l, settings.motor.psi, hardware::Tf);
			}

			systems::dq u = values.motor.rotor.u;

			if(management::observer::hfi)
			{
				u.d += hfi.Injection();
			}

			// transform the voltages to stator frame
			u_ab = systems::transform::InversePark(u, phi_sc);

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

			values.motor.rotor.u.d = 0.0;

			modules::freemaster::Recorder();
		}

	}
}/* namespace control */



