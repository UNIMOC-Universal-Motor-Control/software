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
	thread::thread():flux(), hall(), foc(),	as5048(hardware::i2c::instance), uq(32e3f, 1.0f, 2e-3f), ubat(32e3f, 1.0f, 2e-3f), w(32e3f, 1.0f, 2e-3f)
	{}


	/**
	 * Limit the current setpoint according to temp, voltage, and current limits
	 * @param setpoint[in/out] current setpoint
	 */
	void thread::LimitCurrentSetpoint(systems::dq& setpoint)
	{
		using namespace values::motor::rotor;
		using namespace values;

		float ratio = ubat.Calculate(battery::u) / uq.Calculate(u.q);
		float min = -settings.motor.limits.i;
		float max =  settings.motor.limits.i;

		// deadzone for battery current limiter of 1W
		if(std::fabs(uq.Get() * setpoint::i.q) > 1.0f)
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
		std::array<float, 5> derate;
		derate[0] = Derate(settings.converter.limits.temperature,
				settings.converter.derating.temprature, converter::temp);

		derate[1] = Derate(settings.battery.limits.voltage.low,
				-settings.converter.derating.voltage, ubat.Get());

		derate[2] = Derate(settings.battery.limits.voltage.high,
						settings.converter.derating.voltage, ubat.Get());

		w.Calculate(omega);
		derate[3] = Derate(settings.motor.limits.omega.forwards,
				settings.converter.derating.omega, w.Get());

		derate[4] = Derate(settings.motor.limits.omega.backwards,
				-settings.converter.derating.omega, w.Get());

		// always use the minimal derating possible
		float derating = *std::min_element(derate.begin(), derate.end());

		// derate the limits
		min *= derating;
		max *= derating;

		motor::rotor::setpoint::limit::i::min = min;
		motor::rotor::setpoint::limit::i::max = max;

		Limit(setpoint.q, motor::rotor::setpoint::limit::i::min, motor::rotor::setpoint::limit::i::max);
	}

	/**
	 * compensate the commanded voltage for the error introduced by PWM deadtime
	 */
	void thread::DeadtimeCompensation(void)
	{
//		using namespace values;
//		using namespace values::motor::rotor;
//		using namespace math;
//
//		// Deadtime Compensation, runs but needs some more ifs and else
//		std::int32_t k = 0;
//		if		(phi < std::numeric_limits<std::int32_t>::max()/6) 		k = 0;
//		else if	(phi < std::numeric_limits<std::int32_t>::max()/2)	 	k = std::numeric_limits<std::int32_t>::max()/3;
//		else if	(phi < std::numeric_limits<std::int32_t>::max()/6*5) 	k = std::numeric_limits<std::int32_t>::max()/3*2;
//		else if	(phi > -std::numeric_limits<std::int32_t>::max()/6*5)	k = std::numeric_limits<std::int32_t>::max();
//		else if	(phi < std::numeric_limits<std::int32_t>::max()/6*9) 	k = std::numeric_limits<std::int32_t>::max()/6*8;
//		else if	(phi < std::numeric_limits<std::int32_t>::max()/6*11) 	k = std::numeric_limits<std::int32_t>::max()/6*11;
//
//
//		systems::sin_cos tmp = systems::SinCos(k - phi);
//
//		u.d += 4.0f/3.0f*battery::u*(settings.converter.deadtime*1e-9f)*hardware::Fc()*tmp.cos;
//		u.q += 4.0f/3.0f*battery::u*(settings.converter.deadtime*1e-9f)*hardware::Fc()*tmp.sin;
	}

	/**
	 * calculate analog input signal with deadzones
	 * @param input 0-1 input
	 * @return	0-1 output with dead zones
	 */
	float thread::UniAnalogThrottleDeadzone(const float input)
	{
		float output = (input - settings.throttle.deadzone.low)/(settings.throttle.deadzone.high - settings.throttle.deadzone.low);
		Limit(output, 0.0f, 1.0f);
		return output;
	}

	/**
	 * calculate analog input signal with dead zones in bidirectional manner
	 * @param input 0-1 input
	 * @return	-1-1 output with dead zones at 0 and +-1
	 */
	float thread::BiAnalogThrottleDeadzone(const float input)
	{
		float bi_input = (input *2.0f) - 1.0f;
		// low dead zone bidirectional
		if(std::fabs(bi_input) < settings.throttle.deadzone.low) bi_input = 0.0f;
		else bi_input -= std::copysign(settings.throttle.deadzone.low, bi_input);
		// scaling for high dead zone
		float output = bi_input/(settings.throttle.deadzone.high - settings.throttle.deadzone.low);
		// cut of high dead zone
		Limit(output, -1.0f, 1.0f);

		return output;
	}

	/**
	 * get the mapped throttle input signal
	 * @param setpoint
	 */
	void thread::SetThrottleSetpoint(systems::dq& setpoint)
	{
		using namespace values::motor::rotor;

		switch(settings.throttle.sel)
		{
		default:
		case settings_ts::throttle_s::NONE:
			break;
		case settings_ts::throttle_s::ANALOG_BIDIRECTIONAL:
		{
			setpoint.d = 0.0f;
			float throttle = BiAnalogThrottleDeadzone(hardware::adc::input());

			if(throttle > 0.0f) setpoint.q = throttle * setpoint::limit::i::max;
			else setpoint.q = throttle * setpoint::limit::i::min;

			break;
		}

		case settings_ts::throttle_s::ANALOG_FORWARDS:
			setpoint.d = 0.0f;
			setpoint.q = UniAnalogThrottleDeadzone(hardware::adc::input()) * setpoint::limit::i::max;
			break;
		case settings_ts::throttle_s::ANALOG_BACKWARDS:
			setpoint.d = 0.0f;
			setpoint.q = UniAnalogThrottleDeadzone(hardware::adc::input()) * setpoint::limit::i::min;
			break;
		case settings_ts::throttle_s::ANALOG_SWITCH:
			break;
		case settings_ts::throttle_s::PAS:
			break;
		}

	}


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

			// Handle obviously wrong hall states as pure flux observer info
			if(management::observer::hall
				&& (motor::hall::state != 0 && motor::hall::state != 7))
			{
				// calculate the hall observer
				hall.Calculate(motor::hall::flux);

				// calculate reference flux vector from hall sensors
				motor::rotor::flux::set.d = motor::hall::flux.d * settings.motor.psi;
				motor::rotor::flux::set.q = motor::hall::flux.q * settings.motor.psi;
			}
			else
			{
				// calculate reference flux vector from estimated rotor position
				motor::rotor::flux::set.d = settings.motor.psi;
				motor::rotor::flux::set.q = 0.0f;
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
				LimitCurrentSetpoint(motor::rotor::setpoint::i);

				// calculate the field orientated controllers
				foc.Calculate(motor::rotor::setpoint::i);
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





