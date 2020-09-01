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
					measure::resistance = true;
					measure::inductance = true;
					measure::flux = true;
				}

				if(measure::resistance) sequencer = MEASURE_RS_INIT;
				else if(measure::inductance) sequencer = MEASURE_LS;
				else if(measure::flux) sequencer = MEASURE_FLUX;

				break;

			case MEASURE_RS_INIT:
				measure::Rs = 0.0f;
				measure::u = 0.0f;
				values.motor.rotor.u.d = 0.0f;
				values.motor.rotor.u.q = 0.0f;
				values.motor.rotor.phi = 0.0f;
				values.motor.rotor.omega = 0.0f;
				sequencer = MEASURE_RS_A;

				// handle PWM led to show PWM status
				if(hardware::pwm::output::Active()) palSetLine(LINE_LED_PWM);
				else
				{
					// wait for PWM release
					sequencer = MEASURE_RS_INIT;

					palClearLine(LINE_LED_PWM);
				}
				// set Run Mode LED
				palClearLine(LINE_LED_RUN);
				palSetLine(LINE_LED_MODE);
				break;
			case MEASURE_RS_A: // Find the general range of the stator resistance
				static std::uint8_t cycle = 0;

				cycle++;
				if(cycle > 25)
				{
					cycle = 0;
					measure::u += 100e-3f; // 10mv increase leads to 2.5s max time to reach 24V on the output
				}
				values.motor.rotor.u.d = measure::u;

				if(values.motor.rotor.u.d > values.battery.u * 0.45
						|| values.motor.i.a < -TARGET_CURRENT		// inverse current measurement
						|| std::fabs(values.motor.i.b) > TARGET_CURRENT
						|| std::fabs(values.motor.i.c) > TARGET_CURRENT
						|| !hardware::pwm::output::Active())
				{
					// error target current not reached within voltage limits
					// or the other currents reached target current but not the main current
					// there exists a connection problem
					sequencer = RUN;
					measure::Rs = 0.0f;
					measure::u = 0.0f;
					measure::resistance = false;
					values.motor.rotor.u.d = 0.0f;
					values.motor.rotor.u.q = 0.0f;
					values.motor.rotor.phi = 0.0f;
					values.motor.rotor.omega = 0.0f;
				}
				else if(values.motor.i.a > TARGET_CURRENT)
				{
					measure::resistance = false;
					// error target current not reached within voltage limits
					// or the other currents reached target current but not the main current
					// there exists a connection problem
					sequencer = RUN;
					measure::Rs = 0.0f;
					measure::u = 0.0f;
					values.motor.rotor.u.d = 0.0f;
					values.motor.rotor.u.q = 0.0f;
					values.motor.rotor.phi = 0.0f;
					values.motor.rotor.omega = 0.0f;

				}
				break;

			case MEASURE_LS:
				measure::Ls = {0.0f, 0.0f};
				// set Run Mode LED
				palClearLine(LINE_LED_RUN);
				palSetLine(LINE_LED_MODE);
				break;


			case MEASURE_FLUX:
				measure::PsiM = 0.0f;
				// set Run Mode LED
				palClearLine(LINE_LED_RUN);
				palSetLine(LINE_LED_MODE);
				break;

			}

			sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
		}
	}

} /* namespace management */


