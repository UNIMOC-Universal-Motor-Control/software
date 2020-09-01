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
			///< currents and voltages at each sample point
			std::array<std::array<point, hardware::PHASES>, 10> table;

			///< enable flag
			bool enable = false;

			///< current measurement voltage
			float u = 0.0f;

			///< current current step
			std::uint8_t cur_step = 0;

			///< current phi step
			std::uint8_t phi_step = 0;

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
				control::feedforward = settings.control.current.feedforward;

				// Handling of measurement flags
				if(measure::all)
				{
					measure::r::enable = true;
//					measure::inductance = true;
//					measure::flux = true;
				}

				if(measure::r::enable) sequencer = MEASURE_RS;
//				else if(measure::inductance) sequencer = MEASURE_LS;
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


				if(values.motor.rotor.u.d > values.battery.u * 0.45
						|| values.motor.rotor.i.d < -measure::r::CUR_STEPS.back()		// inverse current measurement
						|| std::fabs(values.motor.rotor.i.q) > measure::r::CUR_STEPS.back()
						|| !hardware::pwm::output::Active())
				{
					// error target current not reached within voltage limits
					// or the other currents reached target current but not the main current
					// there exists a connection problem
					sequencer = RUN;
					measure::r::u = 0.0f;
					measure::r::enable = false;
					measure::r::cycle = 0;
					measure::r::cur_step = 0;
					measure::r::phi_step = 0;
					values.motor.rotor.u.d = 0.0f;
					values.motor.rotor.u.q = 0.0f;
					values.motor.rotor.phi = 0.0f;
					values.motor.rotor.omega = 0.0f;
				}

				// reached current steps current target
				if(values.motor.rotor.i.d > measure::r::CUR_STEPS[measure::r::cur_step])
				{
					// sample the point
					measure::r::table[measure::r::phi_step][measure::r::cur_step].u = measure::r::u;
					measure::r::table[measure::r::phi_step][measure::r::cur_step].i = values.motor.i;

					measure::r::cur_step++;

					if(measure::r::cur_step >= measure::r::CUR_STEPS.size())
					{
						// reached the last step for this phase
						measure::r::cur_step = 0;
						measure::r::phi_step++;
						measure::r::u = 0.0f;
						values.motor.rotor.u.d = 0.0f;
						values.motor.rotor.u.q = 0.0f;

						if(measure::r::phi_step >= measure::r::PHI_STEPS.size())
						{
							// finished with all phases
							sequencer = RUN;
							measure::r::u = 0.0f;
							measure::r::enable = false;
							measure::r::cycle = 0;
							measure::r::cur_step = 0;
							measure::r::phi_step = 0;
							values.motor.rotor.u.d = 0.0f;
							values.motor.rotor.u.q = 0.0f;
							values.motor.rotor.phi = 0.0f;
							values.motor.rotor.omega = 0.0f;
						}
						else
						{
							values.motor.rotor.phi = measure::r::PHI_STEPS[measure::r::phi_step];
						}
					}
				}

				measure::r::cycle++;
				if((measure::r::cycle % 25) == 0)
				{
					measure::r::u += 100e-3f; // 100mv increase every 25ms
				}
				values.motor.rotor.u.d = measure::r::u;
				break;
			}

			sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
		}
	}

} /* namespace management */


