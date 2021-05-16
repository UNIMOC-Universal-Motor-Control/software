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
#include <cstring>
#include <stdint.h>
#include <array>
#include <algorithm>
#include "ch.hpp"
#include "hal.h"
#include "hardware_interface.hpp"
#include "measurement.hpp"
#include "management.hpp"

using namespace chibios_rt;


/**
 * @namespace measurement flags
 */
namespace measurement
{
	/**
	 * calculates the optimal current controller proportional gain
	 * @param inductance of the stator
	 * @param t2 filter time constant
	 * @return kp of the current controller
	 */
	constexpr float CalculateKp(const float inductance, const float t2)
	{
		if(t2 > 1e-9) return (inductance/t2*0.5f);
		else return (0.0f);
	}

	/**
	 * calculates the time constant of the pi current controller to cancel
	 * the electrical time constant
	 * @param inductance of the stator
	 * @param resitance of the stator
	 * @return tn of the current controller
	 */
	constexpr float CalculateTn(const float inductance, const float resistance)
	{
		if(resistance > 1e-9) return (inductance/resistance);
		else return (1e3f);
	}


	/**
	 * run any of the measurement functions with stop handling
	 * @param enable	true if measurement is allowed to be active
	 * @param func		function pointer to one of the run functions of the measurements
	 * @return true if measurement finished regardless if measuement was a success or stop was true
	 */
	bool Run(const bool enable, bool (*func)(bool))
	{
		bool stop = false;

		// handle PWM led to show PWM status
		if(!hardware::pwm::output::Active() || !enable)
		{
			palClearLine(LINE_LED_PWM);

			stop = true;
		}
		else
		{
			palSetLine(LINE_LED_PWM);
		}

		return func(stop);
	}




	/**
	 * @namespace resistance measurement values
	 */
	namespace r
	{
		///< measure all the phases.
		constexpr std::array<std::int32_t, 3> PHI_STEPS = {0, unit::Q31D(120.0f), unit::Q31D(-120.0f)};

		///< currents (x) and voltages (y) at each sample point
		std::array<float, PHI_STEPS.size()> x;
		std::array<float, PHI_STEPS.size()> y;

		///< target current
		float current = 0.0f;

		///< current measurement voltage
		float u = 0.0f;

		///< current phi step
		std::uint8_t phi_step = 0;

		///< current measurement point
		std::uint8_t point = 0;

		///< cycle counter
		std::uint32_t cycle = 0;

		///< statemachine state
		enum sequencer_e
		{
			START,
			MEASURE,
			CALCULATE
		} sequencer = START;

		/**
		 * Measure the stators winding resistance.
		 *
		 * The function will slowly increase current in each of the 3 phases direction
		 * until current reaches 75% of current limit.
		 *
		 * @param stop	if true the measurement stops in a clean manner
		 * @return true if measurement finished regardless if measuement was a success or stop was true
		 */
		bool Run(bool stop)
		{
			bool finished = false;

			using namespace values::motor;
			using namespace management;

			if(stop)
			{
				sequencer = CALCULATE;
			}

			switch(sequencer)
			{
			case START:
				u = 0.0f;
				rotor::u.d = 0.0f;
				rotor::u.q = 0.0f;
				rotor::phi = 0;
				rotor::omega = 0.0f;
				control::current = false;
				observer::hall = false;
				observer::flux = false;
				observer::hfi = false;

				current = settings.motor.limits.i * 0.75;
				sequencer = MEASURE;
				break;
			case MEASURE:
				// reached current steps current target
				if((rotor::i.d > current
						&& cycle > 100)
						||	rotor::u.d > values::battery::u * 0.50
						||	u > values::battery::u * 0.50)
				{
					// sample the point
					x[point] = rotor::i.d;
					y[point] = rotor::u.d;

					point++;
					phi_step++;
					u = 0.0f;

					if(phi_step >= PHI_STEPS.size())
					{
						// finished with all phases
						sequencer = CALCULATE;
					}
					else
					{
						rotor::phi = PHI_STEPS[phi_step];
					}

				}	else if(std::fabs(rotor::i.d) > current
						|| std::fabs(rotor::i.q) > current
						|| !hardware::pwm::output::Active())
				{
					// error target current not reached within voltage limits
					// or the other currents reached target current but not the main current
					// there exists a connection problem
					sequencer = CALCULATE;
				}

				cycle++;
				if((cycle % 25) == 0 && cycle > 100)
				{
					u += 100e-3f; // 100mv increase every 25ms
				}
				rotor::u.d = u;
				break;
			case CALCULATE:
				rotor::u.d = 0.0f;
				rotor::u.q = 0.0f;
				rotor::phi = 0;
				rotor::omega = 0.0f;

				if(phi_step >= PHI_STEPS.size())
				{
					float r = 0.0f;
					for (std::uint8_t i = 0; i < PHI_STEPS.size(); ++i)
					{
						r += y[i] / x[i];
					}
					r /= (float)PHI_STEPS.size();

					settings.motor.rs = r;

					settings.control.current.kp = CalculateKp(settings.motor.l.d, hardware::Tf());
					settings.control.current.tn = CalculateTn(settings.motor.l.q, settings.motor.rs);
				}
				point = 0;
				u = 0.0f;
				cycle = 0;
				phi_step = 0;

				sequencer = START;
				finished = true;
				break;
			}
			return finished;
		}

	}


	/**
	 * @namespace inductance measurement values
	 */
	namespace l
	{
		///< measurement current.
		float CUR = 3.0f;

		///< measurement frequency
		float FREQ = 800.0f;

		///< inductance measurement voltage
		float u = 0.0f;

		///< cycle counter
		std::uint32_t cycle = 0;

		///< old omega limit
		float w_limit = 0.0f;

		///< statemachine state
		enum sequencer_e
		{
			START,
			MEASURE,
			CALCULATE
		} sequencer = START;

		/**
		 * Measure the stators winding inductance with axial components.
		 *
		 * The function will slowly increase current in a fast rotating field
		 * until current reaches the set current limit.
		 *
		 * @param stop	if true the measurement stops in a clean manner
		 * @return true if measurement finished regardless if measuement was a success or stop was true
		 */
		bool Run(bool stop)
		{
			bool finished = false;

			using namespace values::motor;
			using namespace management;

			if(stop)
			{
				sequencer = CALCULATE;
			}

			switch(sequencer)
			{
			case START:
				u = 0.0f;
				rotor::u.d = 0.0f;
				rotor::u.q = 0.0f;
				rotor::phi = 0;
				rotor::omega = math::_2PI * FREQ;
				torque::load = 0.0f;

				w_limit = settings.motor.limits.omega.forwards;
				settings.motor.limits.omega.forwards = rotor::omega;
				control::current = false;
				observer::hall = false;
				observer::flux = false;
				observer::hfi = false;
				sequencer = MEASURE;
				break;
			case MEASURE:
				cycle++;
				if((cycle % 50) == 0)
				{
					if(systems::Length(rotor::i) > CUR
						||	rotor::u.q > values::battery::u * 0.50
						||	u > values::battery::u * 0.50)
					{
						sequencer = CALCULATE;
					}
					else
					{
						u += 100e-3f; // 100mv increase every 25ms
					}
				}
				rotor::u.q = u;
				break;

			case CALCULATE:
				// TODO needs rework: phase Lag of current vetor is motor depandent.
				// calculate the dc levels
				rotor::gid.SetFrequency(0.0f, hardware::Fc());
				rotor::gid.Calculate();
				{
					// only use inductive part of the response
					float i_len = rotor::gid.Magnitude();

					if(i_len > 0.0f)
					{
						// get the Ld - Lq current at twice the injection frequency
						rotor::gid.SetFrequency(2.0f * FREQ, hardware::Fc());
						rotor::gid.Calculate();
						float iac = rotor::gid.Magnitude();

						// assume that Ld is always lower than Lq due to Saturation
						settings.motor.l.d = u/(rotor::omega * (i_len + iac));
						settings.motor.l.q = u/(rotor::omega * (i_len - iac));

						settings.control.current.kp = CalculateKp(settings.motor.l.d, hardware::Tf());
						settings.control.current.tn = CalculateTn(settings.motor.l.q, settings.motor.rs);
					}
				}

				u = 0.0f;
				rotor::u.d = 0.0f;
				rotor::u.q = 0.0f;
				rotor::phi = 0;
				rotor::omega = 0.0f;
				torque::load = 0.0f;
				settings.motor.limits.omega.forwards = w_limit;
				control::current = false;
				observer::hall = false;
				observer::flux = false;
				observer::hfi = false;

				cycle = 0;
				finished = true;
				sequencer = START;
				break;
			}

			return finished;
		}
	}

	/**
	 * @namespace flux measurement values
	 */
	namespace psi
	{
		///< old current controller kp
		float kp = 0.0f;

		///< cycle counter
		std::uint32_t cycle = 0;

		///< statemachine state
		enum sequencer_e
		{
			START,
			UPDATE_KP,
			MEASURE,
			SLOWDOWN,
			CALCULATE,
			STOP,
		} sequencer = START;

		/**
		 * Measure the rotor flux.
		 *
		 * The function will turn on current control and set 50% max current then slowly increases omega
		 * until the voltages needed for the current control exceed 25% battery voltage
		 *
		 * @param stop	if true the measurement stops in a clean manner
		 * @return true if measurement finished regardless if measuement was a success or stop was true
		 */
		bool Run(bool stop)
		{
			bool finished = false;

			using namespace values::motor;
			using namespace management;

			if(stop)
			{
				sequencer = STOP;
			}

			switch(sequencer)
			{
			case START:
				rotor::u.d = 0.0f;
				rotor::u.q = 0.0f;
				rotor::phi = 0.0f;
				rotor::omega = 0.0f;
				torque::load = 0.0f;

				observer::hall = false;
				observer::flux = false;
				observer::hfi = false;
				control::feedforward = false;

				// turn controller off to update KP
				control::current = false;

				kp = settings.control.current.kp;
				settings.control.current.kp = 0.1f;
				sequencer = UPDATE_KP;
				break;
			case UPDATE_KP:
				control::current = true;
				settings.motor.psi = 0.0f;
				rotor::setpoint::i.d = 0.0f;
				rotor::setpoint::i.q = settings.motor.limits.i * 0.5f;
				sequencer = MEASURE;
				break;
			case MEASURE:
				cycle++;
				if((cycle % 100) == 0)
				{
					if(	   std::fabs(rotor::u.d) > values::battery::u * 0.25f
							|| std::fabs(rotor::u.q) > values::battery::u * 0.25f
							|| rotor::omega > 0.5f*settings.motor.limits.omega.forwards)
					{
						sequencer = CALCULATE;
						rotor::setpoint::i.d = 0.0f;
						rotor::setpoint::i.q = 0.0f;
					}
					else
					{
						rotor::omega += 1.0f;
					}
				}
				break;

			case CALCULATE:
			{
				float bemf_d = rotor::u.d - rotor::i.d*settings.motor.rs + rotor::omega*settings.motor.l.q*rotor::i.q;
				float bemf_q = rotor::u.q - rotor::i.q*settings.motor.rs - rotor::omega*settings.motor.l.d*rotor::i.d;
				systems::dq bemf = {bemf_d, bemf_q};
				if(rotor::omega > 1.0f) settings.motor.psi = systems::Length(bemf)/rotor::omega;

				sequencer = SLOWDOWN;
				break;
			}
			case SLOWDOWN:	// let the motor slow down with 0 current to not stop abrubt
				cycle++;
				if((cycle % 10) == 0)
				{
					rotor::omega -= 1.0f;

					if(rotor::omega < 10.0f)
					{
						sequencer = STOP;
					}
				}
				break;
			case STOP:
				observer::hall = false;
				observer::flux = false;
				observer::hfi = false;

				control::feedforward = false;
				control::current = false;

				settings.control.current.kp = kp;

				rotor::u.d = 0.0f;
				rotor::u.q = 0.0f;
				rotor::omega = 0.0f;
				torque::load = 0.0f;

				cycle = 0;
				sequencer = START;
				finished = true;
				break;
			}
			return finished;
		}
	}
} /* namespace measurement */



