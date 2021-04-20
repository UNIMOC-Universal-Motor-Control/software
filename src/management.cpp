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
#include <cstring>
#include <stdint.h>
#include <array>
#include "ch.hpp"
#include "filter.hpp"
#include "hal.h"
#include "hardware_interface.hpp"
#include "management.hpp"
#include "as5048b.hpp"

using namespace chibios_rt;


volatile bool save = false;


/**
 * @namespace controller management classes
 */
namespace management
{
	/**
	 * @namespace observer flags
	 */
	namespace observer
	{
		///< release high frequency injection observer
		bool hfi = false;

		///< release flux observer
		bool flux = false;

		///< release hall sensor based observer
		bool hall = false;
	}

	/**
	 * @namespace controller flags
	 */
	namespace control
	{
		///< release current control
		bool current = false;

		///< feedforward omega
		bool feedforward = false;

		///< release speed control
		bool speed = false;

		///< release position control
		bool position = false;
	}

	/**
	 * @namespace measurement flags
	 */
	namespace measure
	{
		///< measure all parameters
		bool all = false;

		/**
		 * @namespace resistance measurement values
		 */
		namespace r
		{
			///< currents (x) and voltages (y) at each sample point
			std::array<float, PHI_STEPS.size()> x;
			std::array<float, PHI_STEPS.size()> y;

			///< target current
			float current = 0.0f;

			///< enable flag
			bool enable = false;

			///< current measurement voltage
			float u = 0.0f;

			///< current phi step
			std::uint8_t phi_step = 0;

			///< current measurement point
			std::uint8_t point = 0;

			///< cycle counter
			std::uint32_t cycle = 0;
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

			///< enable flag
			bool enable = false;

			///< inductance measurement voltage
			float u = 0.0f;

			///< cycle counter
			std::uint32_t cycle = 0;

			///< old omega limit
			float w_limit = 0.0f;
		}

		/**
		 * @namespace flux measurement values
		 */
		namespace psi
		{
			///< enable flag
			bool enable = false;

			///< old current controller kp
			float kp = 0.0f;

			///< cycle counter
			std::uint32_t cycle = 0;
		}
		
		/**
		 * @namespace hallsensor measurement values
		 */
		namespace hall
		{
			///< enable flag
			bool enable = false;

			///< cycle counter
			std::uint32_t cycle = 0;

			///< actual hall sensor state
			std::uint8_t state = 0;

			///< last hall sensor state
			std::uint8_t old_state = 0;

			///< hall sensor state change angles table
			std::array<float, 8> angles_cw = {0.0f};
			std::array<float, 8> angles_ccw = {0.0f};

			///< direction of turning
			typedef enum direction_e
			{
				CW = 0,
				CCW = 1,
			} direction_te;
			direction_te direction = CW;

			///< measured hall states
			std::uint8_t states_seen = 0;

			///< old current controller kp
			float kp = 0.0f;
		}

	}

	/**
	 * generic constructor
	 */
	thread::thread(): deadline(0), sequencer(STARTUP)
	{};

	/**
	 * @brief Thread main function
	 */
	void thread::main(void)
	{
		setName("Management");


		deadline = chibios_rt::System::getTime();

		/*
		 * Normal main() thread activity
		 */
		while (TRUE)
		{
			using namespace values;
			deadline = chibios_rt::System::getTime();

			converter::temp = hardware::adc::temperature::Bridge();
			motor::temp = hardware::adc::temperature::Motor();

			if(save)
			{
				settings.Save();
				save = false;
			}

			switch(sequencer)
			{
			/* Startup point */
			case STARTUP:
				hardware::pwm::output::Disable();

				settings.Load();

				// clear all leds
				palClearLine(LINE_LED_RUN);
				palClearLine(LINE_LED_MODE);
				palClearLine(LINE_LED_ERROR);
				palClearLine(LINE_LED_PWM);

				sequencer = CURRENT_OFFSETS;
				delay = 100; // wait 100ms before taking current samples

				// software release for PWM
				hardware::pwm::output::Enable();

				break;
			/* measure current offsets */
			case CURRENT_OFFSETS:
				if(delay)
				{
					delay--;
				}
				else
				{
					hardware::adc::current::SetOffset();

					sequencer = RUN;
				}
				break;
			case RUN:
				// software release for PWM
				hardware::pwm::output::Enable();

				// handle PWM led to show PWM status
				if(hardware::pwm::output::Active()) palSetLine(LINE_LED_PWM);
				else palClearLine(LINE_LED_PWM);
//				// set Run Mode LED
//				palSetLine(LINE_LED_RUN);
				palClearLine(LINE_LED_MODE);

				// activate control and observers
				if(hardware::pwm::output::Active())
				{
					if(control::current != settings.control.current.active && control::current)
					{
						motor::rotor::u.d = 0.0f;
						motor::rotor::u.q = 0.0f;
						motor::rotor::setpoint::i.d = 0.0f;
						motor::rotor::setpoint::i.q = 0.0f;
					}
					control::current = settings.control.current.active;
				}
				else
				{
					control::current = false;
				}
				observer::flux = settings.observer.flux.enable;
				// hall with hysteresis
				if(settings.observer.hall.enable)
				{
					 if(observer::hall && (motor::rotor::omega > 1.2f * settings.observer.hall.omega_max))
					 {
						 observer::hall = false;
					 }
					 else if(!observer::hall && (motor::rotor::omega < settings.observer.hall.omega_max))
					 {
						 observer::hall = true;
					 }
				}
				else
				{
					observer::hall = false;
				}

				// hfi with hysteresis
				if(settings.observer.hfi.enable)
				{
					 if(observer::hfi && (motor::rotor::omega > 1.2f * settings.observer.hfi.omega_max))
					 {
						 observer::hfi = false;
					 }
					 else if(!observer::hfi && (motor::rotor::omega < settings.observer.hfi.omega_max))
					 {
						 observer::hfi = true;
					 }
				}
				else
				{
					observer::hfi = false;
				}

				control::feedforward = settings.control.current.feedforward;

				// Handling of measurement flags
				if(measure::all)
				{
					measure::r::enable = true;
					measure::l::enable = true;
					measure::psi::enable = true;
//					measure::hall::enable = true;
				}

				if(measure::r::enable) sequencer = MEASURE_RS;
				else if(measure::l::enable) sequencer = MEASURE_LS;
				else if(measure::psi::enable) sequencer = MEASURE_PSI;
//				else if(measure::hall::enable) sequencer = MEASURE_HALL;

				break;

			case MEASURE_RS:
				// init the measurement
				if(measure::r::cycle == 0 && measure::r::phi_step == 0)
				{
					using namespace values::motor::rotor;
					measure::r::u = 0.0f;
					u.d = 0.0f;
					u.q = 0.0f;
					phi = 0;
					omega = 0.0f;
					control::current = false;
					observer::hall = false;
					observer::flux = false;
					observer::hfi = false;

					measure::r::current = settings.motor.limits.i * 0.75;
				}

				// handle PWM led to show PWM status
				if(hardware::pwm::output::Active() && measure::r::enable) palSetLine(LINE_LED_PWM);
				else
				{
					// wait for PWM release
					sequencer = RUN;

					motor::rotor::u.d = 0.0f;
					motor::rotor::u.q = 0.0f;
					motor::rotor::phi = 0;
					motor::rotor::omega = 0.0f;
					measure::r::point = 0;
					measure::r::u = 0.0f;
					measure::r::enable = false;
					measure::r::cycle = 0;
					measure::r::phi_step = 0;

					palClearLine(LINE_LED_PWM);
				}
//				// set Run Mode LED
//				palClearLine(LINE_LED_RUN);

				// reached current steps current target
				if((motor::rotor::i.d > measure::r::current
					&& measure::r::cycle > 100)
					||	motor::rotor::u.d > battery::u * 0.50
					||	measure::r::u > battery::u * 0.50)
				{
					using namespace values::motor::rotor;
					// sample the point
					measure::r::x[measure::r::point] = i.d;
					measure::r::y[measure::r::point] = u.d;

					// set zero position of the position sensor
					if(measure::r::phi_step == 0)
					{
						settings.mechanics.zero_pos = sense::position;
					}

					measure::r::point++;
					measure::r::phi_step++;
					measure::r::u = 0.0f;

					if(measure::r::phi_step >= measure::r::PHI_STEPS.size())
					{
						// finished with all phases
						sequencer = CALCULATE_RS;
					}
					else
					{
						phi = measure::r::PHI_STEPS[measure::r::phi_step];
					}

				}	else if(std::fabs(motor::rotor::i.d) > measure::r::current
						|| std::fabs(motor::rotor::i.q) > measure::r::current
						|| !hardware::pwm::output::Active())
				{
					// error target current not reached within voltage limits
					// or the other currents reached target current but not the main current
					// there exists a connection problem
					sequencer = CALCULATE_RS;
				}

				measure::r::cycle++;
				if((measure::r::cycle % 25) == 0 && measure::r::cycle > 100)
				{
					measure::r::u += 100e-3f; // 100mv increase every 25ms
				}
				motor::rotor::u.d = measure::r::u;
				break;

			case CALCULATE_RS:
				motor::rotor::u.d = 0.0f;
				motor::rotor::u.q = 0.0f;
				motor::rotor::phi = 0;
				motor::rotor::omega = 0.0f;

				if(	measure::r::phi_step >= measure::r::PHI_STEPS.size())
				{
					float r = 0.0f;
					for (std::uint8_t i = 0; i < measure::r::PHI_STEPS.size(); ++i)
					{
						r += measure::r::y[i] / measure::r::x[i];
					}
					r /= (float)measure::r::PHI_STEPS.size();

					settings.motor.rs = r;

					settings.control.current.kp = CalculateKp(settings.motor.l.d, hardware::Tf());
					settings.control.current.tn = CalculateTn(settings.motor.l.q, settings.motor.rs);
				}
				measure::r::point = 0;
				measure::r::u = 0.0f;
				measure::r::enable = false;
				measure::r::cycle = 0;
				measure::r::phi_step = 0;

				sequencer = RUN;

				break;

			case MEASURE_LS:
				// init the measurement
				if(measure::l::cycle == 0)
				{
					using namespace values::motor::rotor;
					measure::l::u = 0.0f;
					u.d = 0.0f;
					u.q = 0.0f;
					phi = 0;
					omega = math::_2PI * measure::l::FREQ;
					motor::m_l = 0.0f;

					measure::l::w_limit = settings.motor.limits.omega;
					settings.motor.limits.omega = omega;
					control::current = false;
					observer::hall = false;
					observer::flux = false;
					observer::hfi = false;
				}

				// handle PWM led to show PWM status
				if(hardware::pwm::output::Active() && measure::l::enable) palSetLine(LINE_LED_PWM);
				else
				{
					// wait for PWM release
					sequencer = RUN;

					motor::rotor::u.d = 0.0f;
					motor::rotor::u.q = 0.0f;
					motor::rotor::phi = 0;
					motor::rotor::omega = 0.0f;
					motor::m_l = 0.0f;
					settings.motor.limits.omega =  measure::l::w_limit;
					control::current = false;

					observer::hall = false;
					observer::flux = false;

					measure::l::enable = false;
					measure::l::cycle = 0;

					palClearLine(LINE_LED_PWM);
				}
//				// set Run Mode LED
//				palClearLine(LINE_LED_RUN);

				measure::l::cycle++;
				if((measure::l::cycle % 50) == 0)
				{

					if(systems::Length(motor::rotor::i) > measure::l::CUR
						||	motor::rotor::u.q > battery::u * 0.50
						||	measure::l::u > battery::u * 0.50)
					{
						sequencer = CALCULATE_LS;
					}
					else
					{
						measure::l::u += 100e-3f; // 100mv increase every 25ms
					}
				}
				motor::rotor::u.q = measure::l::u;
				break;

			case CALCULATE_LS:
				// TODO needs rework: phase Lag of current vetor is motor depandent.
				// calculate the dc levels
				motor::rotor::gid.SetFrequency(0.0f, hardware::Fc());
				motor::rotor::gid.Calculate();
				{
					using namespace values::motor::rotor;
					// only use inductive part of the response
					float i_len = gid.Magnitude();

					if(i_len > 0.0f)
					{
						// get the Ld - Lq current at twice the injection frequency
						gid.SetFrequency(2.0f * measure::l::FREQ, hardware::Fc());
						gid.Calculate();
						float iac = gid.Magnitude();

						// assume that Ld is always lower than Lq due to Saturation
						settings.motor.l.d = measure::l::u/(omega * (i_len + iac));
						settings.motor.l.q = measure::l::u/(omega * (i_len - iac));

						settings.control.current.kp = CalculateKp(settings.motor.l.d, hardware::Tf());
						settings.control.current.tn = CalculateTn(settings.motor.l.q, settings.motor.rs);
					}
				}

				motor::rotor::u.d = 0.0f;
				motor::rotor::u.q = 0.0f;
				motor::rotor::phi = 0;
				motor::rotor::omega = 0.0f;
				motor::m_l = 0.0f;
				settings.motor.limits.omega =  measure::l::w_limit;
				control::current = false;

				observer::hall = false;
				observer::flux = false;

				measure::l::enable = false;
				measure::l::cycle = 0;

				sequencer = RUN;
				break;

			case MEASURE_PSI:
				// init the measurement
				if(measure::psi::cycle == 0)
				{
					using namespace values::motor::rotor;
					u.d = 0.0f;
					u.q = 0.0f;
					phi = 0.0f;
					omega = 0.0f;
					motor::m_l = 0.0f;

					observer::hall = false;
					observer::flux = false;
					observer::hfi = false;
					control::feedforward = false;

					// turn controller off to update KP
					control::current = false;

					measure::psi::kp = settings.control.current.kp;
					settings.control.current.kp = 0.1f;

					// sleep to get kp updated
					sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
					deadline = chibios_rt::System::getTime();

					control::current = true;

					settings.motor.psi = 0.0f;
					setpoint::i.d = 0.0f;
					setpoint::i.q = settings.motor.limits.i * 0.5f;
				}

				// handle PWM led to show PWM status
				if(hardware::pwm::output::Active() && measure::psi::enable) palSetLine(LINE_LED_PWM);
				else
				{
					using namespace values::motor::rotor;

					// wait for PWM release
					sequencer = RUN;

					observer::hall = false;
					observer::flux = false;
					observer::hfi = false;

					control::feedforward = false;
					control::current = false;

					settings.control.current.kp = measure::psi::kp;
					u.d = 0.0f;
					u.q = 0.0f;
					omega = 0.0f;
					motor::m_l = 0.0f;

					setpoint::i.d = 0.0f;
					setpoint::i.q = 0.0f;


					measure::psi::enable = false;
					measure::psi::cycle = 0;

					palClearLine(LINE_LED_PWM);
				}
//				// set Run Mode LED
//				palClearLine(LINE_LED_RUN);

				measure::psi::cycle++;
				if((measure::psi::cycle % 100) == 0)
				{
					using namespace values::motor::rotor;
					if(	   std::fabs(u.d) > battery::u * 0.25f
							|| std::fabs(u.q) > battery::u * 0.25f
							|| omega > 0.5f*settings.motor.limits.omega)
					{
						sequencer = CALCULATE_PSI;
					}
					else
					{
						omega += 1.0f;
					}
				}

				break;

			case CALCULATE_PSI:
			{
				using namespace values::motor::rotor;

				float bemf_d = u.d - i.d*settings.motor.rs + omega*settings.motor.l.q*i.q;
				float bemf_q = u.q - i.q*settings.motor.rs - omega*settings.motor.l.d*i.d;
				systems::dq bemf = {bemf_d, bemf_q};
				if(omega > 1.0f) settings.motor.psi = systems::Length(bemf)/omega;

				observer::hall = false;
				observer::flux = false;
				observer::hfi = false;

				control::feedforward = false;
				control::current = false;

				settings.control.current.kp = measure::psi::kp;
				u.d = 0.0f;
				u.q = 0.0f;
				omega = 0.0f;
				motor::m_l = 0.0f;

				setpoint::i.d = 0.0f;
				setpoint::i.q = 0.0f;


				measure::psi::enable = false;
				measure::psi::cycle = 0;
				sequencer = RUN;
				break;
			}
//			case MEASURE_HALL:
//				// init the measurement
//				if(measure::hall::cycle == 0)
//				{
//
//					measure::hall::kp = settings.control.current.kp;
//					settings.control.current.kp = CalculateKp(settings.motor.l.d, hardware::Tf()) / 10.0f;
//
//					// sleep to get kp updated
//					sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
//					deadline = chibios_rt::System::getTime();
//
//					observer::hall = false;
//					observer::flux = false;
//					observer::hfi = false;
//					control::feedforward = false;
//					control::current = true;
//
//					values.motor.rotor.u.d = 0.0f;
//					values.motor.rotor.u.q = 0.0f;
//					values.motor.rotor.phi = 0.0f;
//					if(measure::hall::direction == measure::hall::direction_te::CW) values.motor.rotor.omega = 2.0f;
//					if(measure::hall::direction == measure::hall::direction_te::CCW) values.motor.rotor.omega = -2.0f;
//					values.motor.m_l = 0.0f;
//
//					if(measure::hall::direction == measure::hall::direction_te::CW)  std::memset((void*)measure::hall::angles_cw.data(), 0, sizeof(measure::hall::angles_cw.data()));
//					if(measure::hall::direction == measure::hall::direction_te::CCW) std::memset((void*)measure::hall::angles_ccw.data(), 0, sizeof(measure::hall::angles_ccw.data()));
//
//					values.motor.rotor.setpoint.i.d = settings.motor.limits.i*0.5f;
//					values.motor.rotor.setpoint.i.q = 0.0f;
//				}
//
//				// handle PWM led to show PWM status
//				if(hardware::pwm::output::Active() && measure::hall::enable) palSetLine(LINE_LED_PWM);
//				else
//				{
//					// wait for PWM release
//					sequencer = RUN;
//
//					measure::hall::cycle = 0;
//
//					settings.control.current.kp = measure::hall::kp;
//
//					palClearLine(LINE_LED_PWM);
//				}
//				// set Run Mode LED
//				palClearLine(LINE_LED_RUN);
//
//				// get hall state
//				measure::hall::state = values.motor.rotor.hall;
//
//				// set first sector
//				if(!measure::hall::cycle)
//				{
//					measure::hall::old_state = measure::hall::state;
//				}
//
//				// new edge of a Hall sensor
//				if(measure::hall::state != measure::hall::old_state && measure::hall::cycle > 1000)
//				{
//					float* angle = &measure::hall::angles_cw[measure::hall::state];
//					if(measure::hall::direction == measure::hall::direction_te::CW) angle = &measure::hall::angles_cw[measure::hall::state];
//					if(measure::hall::direction == measure::hall::direction_te::CCW) angle = &measure::hall::angles_ccw[measure::hall::state];
//
//					if(*angle == 0.0f)
//					{
//						measure::hall::states_seen++;
//						*angle = values.motor.rotor.phi;
//					}
//				}
//				measure::hall::old_state = measure::hall::state;
//
//				measure::hall::cycle++;
//
//				// only valid hall states count
//				if(measure::hall::state < 1 || measure::hall::state > 6
//						|| (measure::hall::states_seen >= 6 && measure::hall::cycle > 10000)
//						|| measure::hall::cycle > 30000)
//				{
//					values.motor.rotor.u.d = 0.0f;
//					values.motor.rotor.u.q = 0.0f;
//					values.motor.rotor.phi = 0.0f;
//					values.motor.rotor.omega = 0.0f;
//
//					observer::hall = false;
//					observer::flux = false;
//					observer::hfi = false;
//
//					control::feedforward = false;
//					control::current = false;
//
//					values.motor.rotor.setpoint.i.d = 0.0f;
//					values.motor.rotor.setpoint.i.q = 0.0f;
//
//					if(measure::hall::direction == measure::hall::direction_te::CCW)
//					{
//						measure::hall::direction = measure::hall::direction_te::CW;
//						measure::hall::enable = false;
//						settings.control.current.kp = measure::hall::kp;
//						sequencer = RUN;
//					}
//					else
//					{
//						measure::hall::direction = measure::hall::direction_te::CCW;
//						measure::hall::states_seen = 0;
//					}
//					measure::hall::cycle = 0;
//
//					// measurement successful
//					if(measure::hall::states_seen >= 6)
//					{
//						settings.motor.hall_table_cw = measure::hall::angles_cw;
//						settings.motor.hall_table_ccw = measure::hall::angles_ccw;
//					}
//
//
//				}
//				break;
			}

			sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
		}
	}

} /* namespace management */


