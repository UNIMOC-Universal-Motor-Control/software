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

		///< release mechanic observer
		bool mechanic = false;
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
			std::array<float, CUR_STEPS.size() * PHI_STEPS.size()> x;
			std::array<float, CUR_STEPS.size() * PHI_STEPS.size()> y;

			///< enable flag
			bool enable = false;

			///< current measurement voltage
			float u = 0.0f;

			///< current current step
			std::uint8_t cur_step = 0;

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
		///< enable flag
		bool enable = false;

		///< current measurement voltage
		float u = 0.0f;

		///< cycle counter
		std::uint32_t cycle = 0;
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
			deadline = chibios_rt::System::getTime();

			values.converter.temp = hardware::adc::temperature::Bridge();
			values.motor.temp = hardware::adc::temperature::Motor();

			if(save) settings.Save();
			save = false;

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
				// set Run Mode LED
				palSetLine(LINE_LED_RUN);
				palClearLine(LINE_LED_MODE);

				// activate control and observers
				if(hardware::pwm::output::Active())
				{
					if(control::current != settings.control.current.active && control::current)
					{
						values.motor.rotor.u.d = 0.0f;
						values.motor.rotor.u.q = 0.0f;
					}
					control::current = settings.control.current.active;
				}
				else
				{
					control::current = false;
				}
				observer::flux = settings.observer.flux.enable;
				observer::hfi = settings.observer.hfi.enable;

				if(observer::flux || observer::hfi) observer::mechanic = true;
				else	observer::mechanic = false;

				control::feedforward = settings.control.current.feedforward;

				// Handling of measurement flags
				if(measure::all)
				{
					measure::r::enable = true;
					measure::l::enable = true;
//					measure::flux = true;
				}

				if(measure::r::enable) sequencer = MEASURE_RS;
				else if(measure::l::enable) sequencer = MEASURE_LS;
//				else if(measure::flux) sequencer = MEASURE_FLUX;

				break;

			case MEASURE_RS:
				// init the measurement
				if(measure::r::cycle == 0)
				{
					measure::r::cur_step = 0;
					measure::r::u = 0.0f;
					measure::r::enable = false;
					measure::r::cur_step = 0;
					measure::r::phi_step = 0;
					values.motor.rotor.u.d = 0.0f;
					values.motor.rotor.u.q = 0.0f;
					values.motor.rotor.phi = 0.0f;
					values.motor.rotor.omega = 0.0f;
					observer::mechanic = false;
					observer::flux = false;
					observer::hfi = false;
				}

				// handle PWM led to show PWM status
				if(hardware::pwm::output::Active()) palSetLine(LINE_LED_PWM);
				else
				{
					// wait for PWM release
					sequencer = RUN;

					palClearLine(LINE_LED_PWM);
				}
				// set Run Mode LED
				palClearLine(LINE_LED_RUN);

				// reached current steps current target
				if(values.motor.rotor.i.d > measure::r::CUR_STEPS[measure::r::cur_step])
				{
					// sample the point
					measure::r::x[measure::r::point] = values.motor.rotor.i.d;
					measure::r::y[measure::r::point] = measure::r::u;
					measure::r::point++;
					measure::r::cur_step++;
				}

				if(		measure::r::cur_step >= measure::r::CUR_STEPS.size()
					||	values.motor.rotor.u.d > values.battery.u * 0.50) // Voltage limit
				{
					measure::r::cur_step = 0;
					measure::r::phi_step++;
					measure::r::u = 0.0f;
					values.motor.rotor.u.d = 0.0f;
					values.motor.rotor.u.q = 0.0f;

					if(measure::r::phi_step >= measure::r::PHI_STEPS.size())
					{
						// finished with all phases
						sequencer = CALCULATE_RS;
					}
					else
					{
						values.motor.rotor.phi = measure::r::PHI_STEPS[measure::r::phi_step];
					}
				}

				if(	   std::fabs(values.motor.rotor.i.d) > measure::r::CUR_STEPS.back()
					|| std::fabs(values.motor.rotor.i.q) > measure::r::CUR_STEPS.back()
					|| !hardware::pwm::output::Active())
				{
					// error target current not reached within voltage limits
					// or the other currents reached target current but not the main current
					// there exists a connection problem
					sequencer = CALCULATE_RS;
				}

				measure::r::cycle++;
				if((measure::r::cycle % 25) == 0)
				{
					measure::r::u += 100e-3f; // 100mv increase every 25ms
				}
				values.motor.rotor.u.d = measure::r::u;
				break;

			case CALCULATE_RS:
				values.motor.rotor.u.d = 0.0f;
				values.motor.rotor.u.q = 0.0f;
				values.motor.rotor.phi = 0.0f;
				values.motor.rotor.omega = 0.0f;

				// at least 3 measurement points on all phases are there.
				if(			measure::r::cur_step > 2
					||		measure::r::phi_step >= measure::r::PHI_STEPS.size())
				{
					float gain, offset;
					filter::LinearRegression(measure::r::x.begin(), measure::r::y.begin(), measure::r::point, gain, offset);

					settings.motor.rs = gain;

					// calculate effective deadtime equivalent voltage error.
					settings.converter.dt = 2.0f * offset / values.battery.u;
				}
				measure::r::point = 0;
				measure::r::u = 0.0f;
				measure::r::enable = false;
				measure::r::cycle = 0;
				measure::r::cur_step = 0;
				measure::r::phi_step = 0;

				sequencer = RUN;

				break;

			case MEASURE_LS:
				// init the measurement
				if(measure::l::cycle == 0)
				{
					measure::r::cur_step = 0;
					measure::r::u = 0.0f;
					measure::r::enable = false;
					measure::r::cur_step = 0;
					measure::r::phi_step = 0;
					values.motor.rotor.u.d = 0.0f;
					values.motor.rotor.u.q = 0.0f;
					values.motor.rotor.phi = 0.0f;
					values.motor.rotor.omega = measure::l::OMEGA;
					observer::mechanic = false;
					observer::flux = false;
					observer::hfi = false;
				}

				// handle PWM led to show PWM status
				if(hardware::pwm::output::Active()) palSetLine(LINE_LED_PWM);
				else
				{
					// wait for PWM release
					sequencer = RUN;

					palClearLine(LINE_LED_PWM);
				}
				// set Run Mode LED
				palClearLine(LINE_LED_RUN);

				if(	   std::fabs(values.motor.rotor.i.d) > measure::l::CUR
					|| std::fabs(values.motor.rotor.i.q) > measure::l::CUR)
				{
					sequencer = CALCULATE_LS;
				}

				measure::l::cycle++;
				if((measure::l::cycle % 25) == 0)
				{
					measure::l::u += 100e-3f; // 100mv increase every 25ms
				}
				values.motor.rotor.u.d = measure::l::u;
				break;

			case CALCULATE_LS:
				values.motor.rotor.u.d = 0.0f;
				values.motor.rotor.u.q = 0.0f;
				values.motor.rotor.phi = 0.0f;
				values.motor.rotor.omega = 0.0f;

				measure::l::enable = false;
				measure::l::cycle = 0;

				sequencer = CALCULATE_LS;
				break;
			}

			sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
		}
	}

} /* namespace management */


