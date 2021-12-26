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
		using namespace values;

		systems::sin_cos tmp = hardware::SinCos(motor::rotor::phi);

		motor::rotor::u.d += 4.0f/3.0f*battery::u*(settings.converter.deadtime*1e-9f)*hardware::Fc()*tmp.sin;
		motor::rotor::u.q += 4.0f/3.0f*battery::u*(settings.converter.deadtime*1e-9f)*hardware::Fc()*tmp.cos;
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
			std::int32_t i_angle;
			std::int32_t u_angle;
			systems::sin_cos i_sc;
			systems::sin_cos u_sc;

			/* Checks if an IRQ happened else wait.*/
			chEvtWaitAny((eventmask_t)1);

			battery::u = hardware::analog::voltage::DCBus();

			hardware::analog::current::Phase(motor::phase::i);
			hardware::analog::voltage::Phase(motor::phase::u);

			converter::temp = hardware::analog::temperature::Bridge();
			motor::temp = hardware::analog::temperature::Motor();

			// transform the current and voltage samples to stator frame
			motor::stator::i = systems::transform::Clark(motor::phase::i);
			motor::stator::u = systems::transform::Clark(motor::phase::u);

			// read hall sensors
			motor::hall::state = hardware::digital::hall::State();

			// calculate the sine and cosine of the new angle
			i_angle = motor::rotor::phi
						- unit::Q31R(motor::rotor::omega * hardware::analog::current::Tf());
			u_angle = motor::rotor::phi
						- unit::Q31R(motor::rotor::omega * hardware::analog::voltage::Tf());
			// calculate new sine and cosine for the reference system
			i_sc = hardware::SinCos(i_angle);
			u_sc = hardware::SinCos(u_angle);

			// convert current samples from clark to rotor frame;
			motor::rotor::i = systems::transform::Park(motor::stator::i, i_sc);
			motor::rotor::u = systems::transform::Park(motor::stator::u, u_sc);

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

			// Compensate for Deadtime
			DeadtimeCompensation();

			// calculate new sine and cosine for the reference system
			motor::rotor::sc = hardware::SinCos(motor::rotor::phi);

			// transform the voltages to stator frame
			motor::stator::u_out = systems::transform::InversePark(motor::rotor::u_out, motor::rotor::sc);

			motor::phase::u_out = systems::transform::InverseClark(motor::stator::u_out);

			// set dutys with overmodulation
			Overmodulation(motor::phase::u_out, battery::u, motor::phase::duty);

			// double pulse test automation
			if(management::double_pulse::enable)
			{
				static bool pause = false;

				if(management::double_pulse::trigger)
				{
					if(management::double_pulse::load_time - management::double_pulse::ontime > hardware::Tc()*0.2375f)
					{
						motor::phase::duty.array[management::double_pulse::phase] = 0.95f;

						management::double_pulse::ontime += hardware::Tc()*0.2375f;
					}
					else if(management::double_pulse::ontime == management::double_pulse::load_time)
					{
						if(pause) pause = false;
						else
						{
							motor::phase::duty.array[management::double_pulse::phase] = 0.1f;	// add the test pulse

							management::double_pulse::trigger = false;
							management::double_pulse::ontime = 0.0f;
						}
					}
					else // loading time is nearly finished
					{
						motor::phase::duty.array[management::double_pulse::phase] =
								(management::double_pulse::load_time - management::double_pulse::ontime) / (hardware::Tc()*0.25f);

						management::double_pulse::ontime = management::double_pulse::load_time;
						pause = true;
					}
				}

			}

			hardware::pwm::Duty(motor::phase::duty);

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





