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
	 * SVPWM Overmodulation modified to flat bottom.
	 * @param u phase voltages
	 * @param ubat battery voltage
	 * @return dutys in 0 to 1
	 */
	void Overmodulation(systems::abc& u, float ubat, systems::abc& dutys)
	{
		const float* min = std::min_element(u.array.begin(), u.array.end());
		const float* max = std::max_element(u.array.begin(), u.array.end());
		float mid = (*min + *max)*0.5f;

		// prevent division by zero
		if(ubat < 10.0f)
		{
			ubat = 10.0f;
		}
		// scale voltage to -1 to 1
		float scale = 1.0f / ubat;

		for (std::uint8_t i = 0; i < hardware::PHASES; ++i)
		{
			dutys.array[i] = (u.array[i] - mid) * scale + 0.5f;
		}
	}

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
			feedforward.d = settings.motor.rs*setpoint.d - values.motor.rotor.omega * setpoint.q * settings.motor.l.q;
			feedforward.q = settings.motor.rs*setpoint.q + values.motor.rotor.omega * setpoint.d * settings.motor.l.d
					+ values.motor.rotor.omega*settings.motor.psi;
		}

		values.motor.rotor.u.d = ctrl_d.Calculate(setpoint.d, values.motor.rotor.i.d, feedforward.d);
		values.motor.rotor.u.q = ctrl_q.Calculate(setpoint.q, values.motor.rotor.i.q, feedforward.q);
	}

	/**
	 * generic constructor
	 */
	thread::thread():flux(), hfi(), foc(),	as5048(hardware::i2c::instance), uq(32e3f, 1.0f, 2e-3f)
	{}

	/**
	 * @brief Thread main function
	 */
	void thread::main(void)
	{
		setName("Control");

		// worker thread for as5048 read
		as5048.start(NORMALPRIO + 3);

		/*
		 * Normal main() thread activity
		 */
		while (TRUE)
		{
			systems::sin_cos phi_sc;
			systems::sin_cos sense_sc;
			float angle;

			// set Run Mode LED
			palClearLine(LINE_LED_RUN);

			/* Checks if an IRQ happened else wait.*/
			chEvtWaitAny((eventmask_t)1);

			// clear Run Mode LED
			palSetLine(LINE_LED_RUN);

			values.battery.u = hardware::adc::voltage::DCBus();

			hardware::adc::current::Value(values.motor.i);

			// Get angle from as5048b
			as5048.SetZero(settings.mechanics.zero_pos);
			values.sense.position = as5048.GetPosition();
			values.sense.angle = as5048.GetPosition(settings.motor.P);


			// calculate the sine and cosine of the new angle
			angle = values.motor.rotor.phi - values.motor.rotor.omega * hardware::Tf();

			// calculate new sine and cosine for the reference system
			systems::SinCos(angle, phi_sc);

			// calculate new sine and cosine for the sensor signal
			systems::SinCos(values.sense.angle, sense_sc);
			// angle error to as5048 signal
			values.motor.rotor.phi_err =  phi_sc.cos*sense_sc.sin - phi_sc.sin*sense_sc.cos;


			// convert 3 phase system to ortogonal
			i_ab = systems::transform::Clark(values.motor.i);
			// convert current samples from clark to rotor frame;
			values.motor.rotor.i = systems::transform::Park(i_ab, phi_sc);

			//sample currents for frequency analysis
			values.motor.rotor.gid = values.motor.rotor.i.d;
			values.motor.rotor.giq = values.motor.rotor.i.q;

			// calculate battery current from power equality
			values.battery.i = (values.motor.rotor.u.d * values.motor.rotor.i.d
					+ values.motor.rotor.u.q * values.motor.rotor.i.q)/values.battery.u;

			if(			management::observer::mechanic
					&& 	std::fabs(values.motor.rotor.i.q) > settings.observer.mech.i_min)
			{
				observer::mechanic::Predict(values.motor.rotor.i);
			}
			else
			{
				systems::dq i = {0.0f, 0.0f};
				observer::mechanic::Predict(i);
			}

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
				// starting help for traction drives
				if(std::fabs(values.motor.rotor.setpoint.i.q) > settings.observer.mech.i_min
						&& settings.motor.i_start > 0.1f && std::fabs(values.motor.rotor.omega) < 200.0f)
				{
					values.motor.rotor.setpoint.i.d =
							settings.motor.i_start * (1.0f - std::fabs(values.motor.rotor.omega) * 0.005f);
				}
				else
				{
					values.motor.rotor.setpoint.i.d = 0.0f;
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
				std::array<float, 4> derate;
				derate[0] = Derate(settings.converter.limits.temperature,
						settings.converter.derating.temprature, values.converter.temp);
				derate[1] = Derate(settings.battery.limits.voltage,
						-settings.converter.derating.voltage, values.battery.u);
				derate[2] = Derate(settings.motor.limits.omega,
										settings.converter.derating.omega, values.motor.rotor.omega);
				derate[3] = Derate(-settings.motor.limits.omega,
										-settings.converter.derating.omega, values.motor.rotor.omega);

				// always use the minimal derating possible
				float derating = *std::min_element(derate.begin(), derate.end());

				// derate the limits
				min *= derating;
				max *= derating;

				values.motor.rotor.setpoint.limit.i.min = min;
				values.motor.rotor.setpoint.limit.i.max = max;

				systems::dq setpoint = values.motor.rotor.setpoint.i;
				Limit(setpoint.q, values.motor.rotor.setpoint.limit.i.min, values.motor.rotor.setpoint.limit.i.max);

				// calculate the field orientated controllers
				foc.Calculate(setpoint);

//				// Deadtime Compensation, runs but needs some more ifs and else
//				float k = 0.0f;
//				if		(values.motor.rotor.phi < 1.0f * math::PI/6.0f) k = 0.0f;
//				else if	(values.motor.rotor.phi < 3.0f * math::PI/6.0f) k = 2.0f * math::PI/6.0f;
//				else if	(values.motor.rotor.phi < 5.0f * math::PI/6.0f) k = 4.0f * math::PI/6.0f;
//				else if	(values.motor.rotor.phi < 7.0f * math::PI/6.0f) k = 6.0f * math::PI/6.0f;
//				else if	(values.motor.rotor.phi < 9.0f * math::PI/6.0f) k = 8.0f * math::PI/6.0f;
//				else if	(values.motor.rotor.phi <11.0f * math::PI/6.0f) k =10.0f * math::PI/6.0f;
//
//
//				systems::sin_cos tmp;
//				systems::SinCos(k - values.motor.rotor.phi, tmp);
//
//				values.motor.rotor.u.d += 4.0f/3.0f*values.battery.u*(settings.converter.deadtime*1e-9f)*hardware::Fc()*tmp.cos;
//				values.motor.rotor.u.q += 4.0f/3.0f*values.battery.u*(settings.converter.deadtime*1e-9f)*hardware::Fc()*tmp.sin;

				if(management::observer::hfi)
				{
					values.motor.rotor.u.d += hfi.Injection();
				}
			}
			else
			{
				foc.Reset();
				foc.SetParameters(settings.control.current.kp, settings.control.current.tn, hardware::Tc());
			}

			// transform the voltages to stator frame
			u_ab = systems::transform::InversePark(values.motor.rotor.u, phi_sc);

			// transform to ab system
			values.motor.u = systems::transform::InverseClark(u_ab);

			// set dutys with overmodulation
			Overmodulation(values.motor.u, values.battery.u, values.converter.dutys);

			hardware::pwm::Duty(values.converter.dutys);

			modules::freemaster::Recorder();

			// read as 5048 every 4th cycle
			static std::uint8_t cnt = 0;
			cnt++;
			if(!(cnt % 4))
			{
				as5048.Read();
			}
		}

	}
}/* namespace control */





