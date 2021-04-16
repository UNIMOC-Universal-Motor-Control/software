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
#include "control_thread.hpp"
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
	 * Clark transform of 4 cycle current measurements
	 * @param[in]  i_abc		phase current samples
	 * @param[out] i_ab 		transformed stator current samples
	 */
	void QuadClark(const std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& i_abc, std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& i_ab)
	{
		for (std::uint_fast8_t i = 0; i < i_abc.size(); ++i)
		{
			i_ab[i] = systems::transform::Clark(i_abc[i]);
		}
	}

	/**
	 * Clark transform of 4 cycle current measurements
	 * @param[in]  i_abc		stator current samples
	 * @retval 					mean of stator current samples
	 */
	systems::alpha_beta MeanAlphaBeta(const std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& i_ab)
	{
		systems::alpha_beta i_ab_mean =
		{
				(i_ab[0].alpha + i_ab[1].alpha + i_ab[2].alpha + i_ab[3].alpha) * 0.25f,
				(i_ab[0].beta + i_ab[1].beta + i_ab[2].beta + i_ab[3].beta) * 0.25f
		};
		return i_ab_mean;
	}

	/**
	 * Inverse Clark transform of 4 cycle voltages that compensates for angle advance due to omega
	 * @param[in]  u_ab			stator voltage vector
	 * @param[out] u_ab_turn	stator voltage vectors with angle advance
	 */
	void QuadInvClark(const systems::alpha_beta u_ab, const float omega, std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& u_ab_turn)
	{
		systems::sin_cos sc;
		systems::dq u_tmp = {u_ab.alpha, u_ab.beta};
		u_ab_turn[0] = {u_tmp.d, u_tmp.q};
		sc = systems::SinCos(unit::Q31(omega*hardware::Tc()/(float)hardware::pwm::INJECTION_CYCLES));

		for (std::uint_fast8_t i = 1; i < u_ab_turn.size(); ++i)
		{
			systems::alpha_beta u_ab_tmp = systems::transform::InversePark(u_tmp, sc);
			u_tmp = {u_ab_tmp.alpha, u_ab_tmp.beta};
			u_ab_turn[i] = u_ab_tmp;
		}
	}

	/**
	 * Quad sample over modulation for svpwm
	 * @param u_ab		input voltage samples vector
	 * @param ubat		current battery voltage
	 * @param dutys		output duty cycles vector
	 */
	void QuadOvermodulation(const std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& u_ab, const float ubat, std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& dutys)
	{
		for (std::uint_fast8_t i = 0; i < dutys.size(); ++i)
		{
			systems::abc u_abc = systems::transform::InverseClark(u_ab[i]);
			Overmodulation(u_abc, ubat, dutys[i]);
		}
	}

	/**
	 * generic constructor
	 */
	thread::thread():flux(), hall(), foc(),	as5048(hardware::i2c::instance), uq(32e3f, 1.0f, 2e-3f)
	{}

#pragma GCC push_options
#pragma GCC optimize ("-O0")
	/**
	 * @brief Thread main function
	 */
	void thread::main(void)
	{
		using namespace values;
		setName("Control");

		// worker thread for as5048 read
		as5048.start(NORMALPRIO + 3);

		/*
		 * Normal main() thread activity
		 */
		while (TRUE)
		{
			std::int32_t angle;

			// set Run Mode LED
			palClearLine(LINE_LED_RUN);

			/* Checks if an IRQ happened else wait.*/
			chEvtWaitAny((eventmask_t)1);

			// clear Run Mode LED
			palSetLine(LINE_LED_RUN);

			battery::u = hardware::adc::voltage::DCBus();

			hardware::adc::current::Value(i_abc);
			hardware::adc::current::Derivative(i_abc_ac);

			// transform the current samples to stator frame
			QuadClark(i_abc, i_ab);
			QuadClark(i_abc_ac, i_ab_ac);

			// calculate the admittance and mean current from the samples
			motor::stator::i = MeanAlphaBeta(i_ab);
			motor::stator::y = observer::hfi::GetMean(i_ab);
			motor::stator::yd = observer::hfi::GetVector(i_ab);

			// Get angle from as5048b
			as5048.SetZero(settings.mechanics.zero_pos);
			sense::position = as5048.GetPosition();
			sense::angle = as5048.GetPosition(settings.motor.P);

			// read hall sensors
			motor::hall::state = hardware::adc::hall::State();

			// calculate the sine and cosine of the new angle
			angle = motor::rotor::phi - unit::Q31(motor::rotor::omega * hardware::Tf());;

			// calculate new sine and cosine for the reference system
			systems::sin_cos sc = systems::SinCos(angle);

			// convert current samples from clark to rotor frame;
			motor::rotor::i = systems::transform::Park(motor::stator::i, sc);

			//sample currents for frequency analysis
			motor::rotor::gid = motor::rotor::i.d;
			motor::rotor::giq = motor::rotor::i.q;

			// calculate battery current from power equality
			battery::i = (motor::rotor::u.d * motor::rotor::i.d
					+ motor::rotor::u.q * motor::rotor::i.q)/battery::u;

			if(management::observer::hall)
			{
				hall.SetOffset(settings.observer.hall.offset);

				// calculate the hall observer
				hall.Calculate(motor::hall::flux);

				// calculate reference flux vector from hall sensors
				motor::rotor::flux::set.d = motor::hall::flux.d * settings.motor.psi;
				motor::rotor::flux::set.q = motor::hall::flux.q * settings.motor.psi;
			}
			else
			{
				// calculate reference flux vector from estimated rotor position
				motor::rotor::flux::set.d = motor::rotor::sc.cos * settings.motor.psi;
				motor::rotor::flux::set.q = motor::rotor::sc.sin * settings.motor.psi;
			}

			if(management::observer::flux)
			{
				std::array<float, 3> correction;
				float error;

				// Predict the new rotor position
				mech.Predict(motor::rotor::i);

				// calculate the flux observer
				flux.Calculate(motor::rotor::flux::set, motor::rotor::flux::act);

		    	if(settings.motor.psi > 1e-8)
		    	{
		    		error = motor::rotor::flux::act.q/settings.motor.psi;
		    	}
		    	else
		    	{
		    		error = motor::rotor::flux::act.q;
		    	}

		    	// Update the
				mech.Update(settings.observer.mech.Q, settings.observer.mech.R, error, correction);

				// correct the prediction
				mech.Correct(correction);
			}
			else
			{
				// start with flux on reference
				motor::rotor::flux::act.d = motor::rotor::flux::set.d;
				motor::rotor::flux::act.q  = motor::rotor::flux::set.q;

				systems::dq i = {0.0f, 0.0f};
				mech.Predict(i);

			}

			if(management::control::current)
			{
				float ratio = battery::u / uq.Calculate(motor::rotor::u.q);
				float min = -settings.motor.limits.i;
				float max =  settings.motor.limits.i;

				// deadzone for battery current limiter of 1W
				if(std::fabs(motor::rotor::u.q * motor::rotor::setpoint::i.q) > 1.0f)
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
						settings.converter.derating.temprature, converter::temp);
				derate[1] = Derate(settings.battery.limits.voltage,
						-settings.converter.derating.voltage, battery::u);
				derate[2] = Derate(settings.motor.limits.omega,
										settings.converter.derating.omega, motor::rotor::omega);
				derate[3] = Derate(-settings.motor.limits.omega,
										-settings.converter.derating.omega, motor::rotor::omega);

				// always use the minimal derating possible
				float derating = *std::min_element(derate.begin(), derate.end());

				// derate the limits
				min *= derating;
				max *= derating;

				motor::rotor::setpoint::limit::i::min = min;
				motor::rotor::setpoint::limit::i::max = max;

				systems::dq setpoint = motor::rotor::setpoint::i;
				Limit(setpoint.q, motor::rotor::setpoint::limit::i::min, motor::rotor::setpoint::limit::i::max);

				// calculate the field orientated controllers
				foc.Calculate(setpoint);

//				// Deadtime Compensation, runs but needs some more ifs and else
//				float k = 0.0f;
//				if		(motor::rotor::phi < 1.0f * math::PI/6.0f) k = 0.0f;
//				else if	(motor::rotor::phi < 3.0f * math::PI/6.0f) k = 2.0f * math::PI/6.0f;
//				else if	(motor::rotor::phi < 5.0f * math::PI/6.0f) k = 4.0f * math::PI/6.0f;
//				else if	(motor::rotor::phi < 7.0f * math::PI/6.0f) k = 6.0f * math::PI/6.0f;
//				else if	(motor::rotor::phi < 9.0f * math::PI/6.0f) k = 8.0f * math::PI/6.0f;
//				else if	(motor::rotor::phi <11.0f * math::PI/6.0f) k =10.0f * math::PI/6.0f;
//
//
//				systems::sin_cos tmp;
//				systems::SinCos(k - motor::rotor::phi, tmp);
//
//				motor::rotor::u.d += 4.0f/3.0f*battery::u*(settings.converter.deadtime*1e-9f)*hardware::Fc()*tmp.cos;
//				motor::rotor::u.q += 4.0f/3.0f*battery::u*(settings.converter.deadtime*1e-9f)*hardware::Fc()*tmp.sin;
			}
			else
			{
				foc.Reset();
				foc.SetParameters(settings.control.current.kp, settings.control.current.tn, hardware::Tc());
			}

			// calculate new sine and cosine for the reference system
			motor::rotor::sc = systems::SinCos(motor::rotor::phi);

			// transform the voltages to stator frame
			motor::stator::u = systems::transform::InversePark(motor::rotor::u, motor::rotor::sc);

			if(management::observer::hfi)
			{
				observer::hfi::Injection(motor::stator::u, u_ab);
			}
			else
			{
				QuadInvClark(motor::stator::u, motor::rotor::omega, u_ab);
			}

			// set dutys with overmodulation
			QuadOvermodulation(u_ab, battery::u, dutys);

			hardware::pwm::Duty(dutys);

			for (std::uint_fast8_t i = 0; i < hardware::pwm::INJECTION_CYCLES; ++i)
			{
				motor::phase::i = i_abc[i];
				motor::phase::di = i_abc_ac[i];
				motor::stator::i = i_ab[i];
				motor::stator::di = i_ab_ac[i];
				motor::stator::u = u_ab[i];
				motor::phase::u = dutys[i];

				modules::freemaster::Recorder();
			}


			// read as 5048 every 4th cycle
			static std::uint8_t cnt = 0;
			cnt++;
			if(!(cnt % 4))
			{
				as5048.Read();
			}
		}

	}
#pragma GCC pop_options
}/* namespace control */





