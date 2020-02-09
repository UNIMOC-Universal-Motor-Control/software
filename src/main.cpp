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
#include "ch.hpp"
#include "hal.h"
#include "usbcfg.h"
#include "hardware_interface.hpp"
#include "freemaster_wrapper.hpp"
#include "main.hpp"

using namespace chibios_rt;

static modules::freemaster::thread freemaster;
static control::thread controller;

/**
 * Code entry point
 * @return never
 */
int main(void)
{
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	System::init();

	/*
	 * Initializes two serial-over-USB CDC drivers.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg1);
	sduObjectInit(&SDU2);
	sduStart(&SDU2, &serusbcfg2);

	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusbcfg1.usbp);
	usbDisconnectBus(serusbcfg2.usbp);
	chThdSleepMilliseconds(1500);
	usbStart(serusbcfg1.usbp, &usbcfg);
	usbStart(serusbcfg2.usbp, &usbcfg);
	usbConnectBus(serusbcfg1.usbp);
	usbConnectBus(serusbcfg2.usbp);

	/*
	 * start freemaster thread
	 */
	freemaster.start(NORMALPRIO);
	hardware::control_thread = controller.start(HIGHPRIO);

	/*
	 * Set hall inputs to outputs for debugging
	 */
	palSetLineMode(LINE_HALL_A, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_HALL_B, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_HALL_C, PAL_MODE_OUTPUT_PUSHPULL);

	/*
	 * initialize hardware with no control thread
	 */
	hardware::memory::Init();
	hardware::pwm::Init();
	hardware::adc::Init();

	hardware::pwm::EnableOutputs();

	palClearLine(LINE_LED_ERROR);
	palClearLine(LINE_LED_MODE);
	palClearLine(LINE_LED_PWM);

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (true)
	{
		if(hardware::pwm::OutputActive())
		{
			palSetLine(LINE_LED_PWM);
		}
		else
		{
			palClearLine(LINE_LED_PWM);
		}
		BaseThread::sleep(TIME_MS2I(500));
	}
}

namespace control
{

	volatile hardware::adc::current_values_ts i_tmp;

	/**
	 * generic constructor
	 */
	thread::thread():flux(), mech(settings::observer::Q, settings::observer::R),
			foc(settings::converter::ts, settings::motor::Rs, 1.0f)
	{}

	/**
	 * @brief Thread main function
	 */
	void thread::main(void)
	{
		setName("Control");


		/*
		 * Normal main() thread activity
		 */
		while (TRUE)
		{

			/* Checks if an IRQ happened else wait.*/
			chEvtWaitAny((eventmask_t)1);

			palSetLine(LINE_HALL_C);

			hardware::adc::GetCurrents((hardware::adc::current_values_ts*)&i_tmp);

			values::battery::u = hardware::adc::GetDCBusVoltage();

			std::memcpy(i_abc.array, (void*)i_tmp.current, sizeof(float)*3);

			// calculate the sine and cosine of the new angle
			float angle = values::motor::rotor::phi
					+ values::motor::rotor::omega * settings::converter::ts;

			// calculate new
			systems::SinCos(angle, values::motor::rotor::sin_cos);

			// convert 3 phase system to ortogonal
			i_ab = systems::transform::Clark(i_abc);
			// convert current samples from clark to rotor frame;
			values::motor::rotor::i = systems::transform::Park(i_ab, values::motor::rotor::sin_cos);
//
//			// calculate the flux observer
//			float flux_error = flux.Calculate();
//			mech.Update(flux_error, correction);
//
//			// predict motor behavior
//			observer::mechanic::Predict();
//			// correct the prediction
//			observer::mechanic::Correct(correction);
//
//			// calculate the field orientated controllers
//			foc.Calculate();
//
			// transform the voltages to stator frame
			u_ab = systems::transform::InversePark(values::motor::rotor::u, values::motor::rotor::sin_cos);
			// next transform to abc system
			u_abc = systems::transform::InverseClark(u_ab);
			// this also sets the internal scaling for the pwm dutys

			//scale the voltages
			if(std::fabs(values::battery::u)> 10.0f)
			{
				u_abc.a /= values::battery::u;
				u_abc.b /= values::battery::u;
				u_abc.c /= values::battery::u;

				// set the dutys
				hardware::pwm::SetDutys(u_abc.array);
			}
			else
			{
				// faulty dc bus voltage
				u_abc.a = 0.0f;
				u_abc.b = 0.0f;
				u_abc.c = 0.0f;
				hardware::pwm::SetDutys(u_abc.array);
			}
			palClearLine(LINE_HALL_C);
		}

	}
}/* namespace control */
