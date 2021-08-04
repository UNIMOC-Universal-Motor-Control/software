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
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "control_thread.hpp"
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
	 * Clark transform of 4 cycle current measurements
	 * @param[in]  i_abc		phase current samples
	 * @param[out] i_ab 		transformed stator current samples
	 */
	void QuadClark(const std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& i_abc, std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& i_ab)
	{
		for (std::uint_fast8_t i = 0; i < i_abc.size(); ++i)
		{
			systems::abc tmp;

			for (std::uint_fast8_t j = 0; j < hardware::PHASES; ++j)
			{
				std::uint_fast8_t p = 0;

				// Map the phase wires to internal phases
		    		 if((1 << j) & settings.converter.map.a) p = 0;
		    	else if((1 << j) & settings.converter.map.b) p = 1;
		    	else if((1 << j) & settings.converter.map.c) p = 2;

		    	tmp.array[p] = i_abc[i].array[j];
			}

			i_ab[i]  = systems::transform::Clark(tmp);
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
		sc = systems::SinCos(unit::Q31R(omega*hardware::Tc()/(float)hardware::pwm::INJECTION_CYCLES));

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
	thread::thread():flux(), mech(), hall(), foc()
	{}

	/**
	 * execute slow management tasks which don't need to run in control loop
	 */
	void thread::Manage(void)
	{
		hall.SetOffset(settings.observer.hall.offset);
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
	 * @brief Thread main function
	 */
	void thread::main(void)
	{
		using namespace values;
		setName("Control");
		/*
		 * Normal main() thread activity
		 */
		while (TRUE)
		{
			std::int32_t angle;

			// set Run Mode LED
//			palClearLine(LINE_LED_RUN);

			/* Checks if an IRQ happened else wait.*/
			chEvtWaitAny((eventmask_t)1);

			// clear Run Mode LED
//			palSetLine(LINE_LED_RUN);

			battery::u = hardware::analog::voltage::DCBus();

			hardware::analog::current::Value(i_abc);
			hardware::analog::current::Derivative(i_abc_ac);

			converter::temp = hardware::analog::temperature::Bridge();
			motor::temp = hardware::analog::temperature::Motor();

			// transform the current samples to stator frame
			QuadClark(i_abc, i_ab);
			QuadClark(i_abc_ac, i_ab_ac);

			// calculate the admittance and mean current from the samples
			motor::stator::i = MeanAlphaBeta(i_ab);
			motor::stator::y = observer::hfi::GetMean(i_ab);
			motor::stator::yd = observer::hfi::GetVector(i_ab);

			// read hall sensors
			motor::hall::state = hardware::digital::hall::State();

			// calculate the sine and cosine of the new angle
			angle = motor::rotor::phi - unit::Q31R(motor::rotor::omega * hardware::Tf());;

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
			if((motor::hall::state != 0 && motor::hall::state != 7))
			{
				// calculate the hall observer
				hall.Calculate(motor::hall::flux);

				if(management::observer::hall)
				{
					// calculate reference flux vector from hall sensors
					motor::rotor::flux::set.d = motor::hall::flux.d;
					motor::rotor::flux::set.q = motor::hall::flux.q;

					// set the observer feedback accorting to observer
					motor::rotor::flux::C = settings.observer.hall.C;
				}
				else
				{
					// calculate reference flux vector from estimated rotor position
					motor::rotor::flux::set.d = settings.motor.psi;
					motor::rotor::flux::set.q = 0.0f;

					// set the observer feedback back to pure flux observer
					motor::rotor::flux::C = settings.observer.flux.C;
				}
			}

			if(management::observer::flux)
			{
				std::array<float, 3> correction;
				float error;

				// Predict the new rotor position
				mech.Predict(motor::rotor::i);

				// calculate the flux observer
				flux.Calculate(motor::rotor::flux::set, motor::rotor::flux::act, motor::rotor::flux::C);

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

			// double pulse test automation
			if(management::double_pulse::enable)
			{
				static bool pause = false;

				for(std::uint8_t i = 0; i < hardware::pwm::INJECTION_CYCLES; i++)
				{
					dutys[i].array.fill(0.0f);

					if(management::double_pulse::trigger)
					{
						if(management::double_pulse::load_time - management::double_pulse::ontime > hardware::Tc()*0.2375f)
						{
							dutys[i].array[management::double_pulse::phase] = 0.95f;

							management::double_pulse::ontime += hardware::Tc()*0.2375f;
						}
						else if(management::double_pulse::ontime == management::double_pulse::load_time)
						{
							if(pause) pause = false;
							else
							{
								dutys[i].array[management::double_pulse::phase] = 0.1f;	// add the test pulse

								management::double_pulse::trigger = false;
								management::double_pulse::ontime = 0.0f;
							}
						}
						else // loading time is nearly finished
						{
							dutys[i].array[management::double_pulse::phase] =
									(management::double_pulse::load_time - management::double_pulse::ontime) / (hardware::Tc()*0.25f);

							management::double_pulse::ontime = management::double_pulse::load_time;
							pause = true;
						}
					}
				}
			}

			hardware::pwm::Duty(dutys);

//			// read as 5048 every 4th cycle
//			static std::uint8_t cnt = 0;
//			cnt++;
//			if(!(cnt % 4))
//			{
//				as5048.Read();
//			}

			// calculate the filters for management Task
			management::motor::torque::electric.Calculate(motor::torque::electric);
			management::motor::torque::load.Calculate(motor::torque::load);
			management::motor::temp.Calculate(motor::temp);
			management::motor::rotor::i.Calculate(motor::rotor::i);
			management::motor::rotor::u.Calculate(motor::rotor::u);
			management::motor::rotor::omega.Calculate(motor::rotor::omega);
			management::motor::rotor::flux.Calculate(motor::rotor::flux::act);
			management::battery::u.Calculate(battery::u);
			management::battery::i.Calculate(battery::i);
			management::converter::temp.Calculate(converter::temp);
		}

	}
}/* namespace control */

control::thread controller;





