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
		///< release admittance observer
		bool admittance = false;

		///< release injection signal for observer
		bool injection = false;

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

		///< release speed control
		bool speed = false;

		///< release position control
		bool position = false;
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
			values.converter.throttle = hardware::adc::Throttle();


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
					systems::abc dummy_gain = {1.0f, 1.0f, 1.0f};
					systems::abc offsets;
					std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> ac_offsets;
					hardware::adc::current::gain::DC(dummy_gain);
					hardware::adc::current::gain::AC(dummy_gain);
					hardware::adc::current::Mean(offsets);
					hardware::adc::current::offset::DC(offsets);
					hardware::adc::current::Injection(ac_offsets);
					offsets = hardware::adc::current::InjectionMean(ac_offsets);
					hardware::adc::current::offset::AC(offsets);
					// set original gains
					hardware::adc::current::gain::DC(settings.converter.dc_gains);
					hardware::adc::current::gain::AC(settings.converter.ac_gains);

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

				// activate control and observers
				control::current = settings.control.current.active;
				observer::injection = settings.motor.u_inj > 0.0f;
				observer::admittance = settings.observer.admittance;
				observer::flux = settings.observer.flux;

				break;
			}

			sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
		}
	}

} /* namespace management */

