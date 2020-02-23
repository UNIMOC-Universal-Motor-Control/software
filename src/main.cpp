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

volatile bool save = false;

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

	while(1)
	{
		uint8_t buffer[16];
		for (uint8_t i = 0; i < 16; ++i)
		{
			buffer[i] = 0x50 + i;
		}

		hardware::memory::Write(0, buffer, 16);

		hardware::memory::Read(0, buffer, 16);

		if(buffer[0] != 0x50)
		{
			palToggleLine(LINE_HALL_A);
		}
	}

	hardware::pwm::Init();
	hardware::adc::Init();

	settings.Load();

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

		if(save) settings.Save();
		save = false;
	}
}

namespace control
{
	/**
	 * generic constructor
	 */
	thread::thread():flux(), mech(settings.observer.Q, settings.observer.R),
			foc(settings.converter.ts, settings.motor.Rs, 1.0f)
	{}

	/**
	 * calculate the mean of a hole injection cycle of adc measurements
	 * @param currents referes to the samples of one hole injection cycle
	 * @return the mean per phase of the injection cycle
	 */
	systems::abc thread::InjectionMean(const std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& currents)
	{
		systems::abc abc;

		for (uint8_t k = 0; k < currents[0].array.size(); ++k)
		{
			abc.array[k] = 0.0f;
			for (uint8_t i = 0; i < currents.size(); ++i)
			{
				abc.array[k] += currents[i].array[k];
			}
			abc.array[k] /= currents.size();
		}
		return abc;
	}


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

			hardware::adc::PrepareSamples();

			values.battery.u = hardware::adc::GetDCBusVoltage();

			hardware::adc::GetCurrentsMean(i_dc);

			if(settings.motor.u_inj != 0.0f)
			{
				systems::sin_cos sin_cos;

				// slow control with injection
				settings.converter.ts = hardware::Tc * hardware::pwm::INJECTION_CYCLES;

				i_abc = InjectionMean(i_dc);

				hardware::adc::GetCurrentsInjection(i_ac);

				for (uint8_t i = 0; i < hardware::pwm::INJECTION_CYCLES; ++i)
				{
					i_ab_ac[i] = systems::transform::Clark(i_ac[i]);
				}
				values.motor.y = admittance.GetVector(i_ab_ac);

				// calculate the sine and cosine of the new angle
				float angle = values.motor.rotor.phi
						+ values.motor.rotor.omega * settings.converter.ts;

				// calculate new
				systems::SinCos(angle, sin_cos);

				// convert 3 phase system to ortogonal
				i_ab = systems::transform::Clark(i_abc);
				// convert current samples from clark to rotor frame;
				values.motor.rotor.i = systems::transform::Park(i_ab, sin_cos);

				// transform the admittance vector to rotor frame
				// the admittance vector is rotating with double the rotor frequency
				y_dq = systems::transform::Park(values.motor.y, sin_cos);
				y_ab.alpha = y_dq.d;
				y_ab.beta = y_dq.q;
				values.motor.rotor.y = systems::transform::Park(y_ab, sin_cos);

				if(settings.observer.admittance)
				{
					// calculate the mech observer from admittance vector
					mech.Update(values.motor.rotor.y.q, correction);

					// predict motor behavior
					observer::mechanic::Predict();
					// correct the prediction
					observer::mechanic::Correct(correction);
				}

				if(settings.control.current)
				{
					// calculate the field orientated controllers
					foc.Calculate();
				}

				// transform the voltages to stator frame
				u_ab = systems::transform::InversePark(values.motor.rotor.u, sin_cos);

				// add the injection pattern to the voltage output
				for (uint8_t i = 0; i < hardware::pwm::INJECTION_CYCLES; ++i)
				{
					const std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES> inj_pat =
					{{
							{1.0f, 0.0f},
							{0.0f, 1.0f},
							{-1.0f, 0.0f},
							{0.0f, -1.0f}
					}};

					systems::alpha_beta u_tmp =
					{
						u_ab.alpha + settings.motor.u_inj * inj_pat[i].alpha,
						u_ab.beta + settings.motor.u_inj * inj_pat[i].beta
					};

					// next transform to abc system
					u_abc[i] = systems::transform::InverseClark(u_tmp);

					modules::freemaster::Recorder();
				}
			}
			else
			{
				std::array<systems::sin_cos, hardware::pwm::INJECTION_CYCLES> sin_cos;

				// highspeed control without injection
				settings.converter.ts = hardware::Tc;

				for (uint8_t i = 0; i < hardware::pwm::INJECTION_CYCLES; ++i)
				{
					values.motor.i = i_dc[i];

					// calculate the sine and cosine of the new angle
					float angle = values.motor.rotor.phi
							+ values.motor.rotor.omega * settings.converter.ts;

					// calculate new sine and cosine
					systems::SinCos(angle, sin_cos[i]);

					// convert 3 phase system to ortogonal
					i_ab = systems::transform::Clark(values.motor.i);
					// convert current samples from clark to rotor frame;
					values.motor.rotor.i = systems::transform::Park(i_ab, sin_cos[i]);

					if(settings.observer.flux)
					{
						// calculate the flux observer
						float flux_error = flux.Calculate(sin_cos[i]);
						mech.Update(flux_error, correction);

						// predict motor behavior
						observer::mechanic::Predict();
						// correct the prediction
						observer::mechanic::Correct(correction);
					}
					else
					{
						values.motor.rotor.phi += values.motor.rotor.omega * settings.converter.ts;
					}

					modules::freemaster::Recorder();
				}

				if(settings.control.current)
				{
					// calculate the field orientated controllers
					foc.Calculate();
				}

				for (uint8_t i = 0; i < hardware::pwm::INJECTION_CYCLES; ++i)
				{
					// transform the voltages to stator frame
					u_ab = systems::transform::InversePark(values.motor.rotor.u, sin_cos[i]);
					// next transform to abc system
					u_abc[i] = systems::transform::InverseClark(u_ab);
				}
			}

			//scale the voltages
			if(std::fabs(values.battery.u)> 10.0f)
			{
				for (uint8_t i = 0; i < hardware::pwm::INJECTION_CYCLES; ++i)
				{
					u_abc[i].a /= values.battery.u;
					u_abc[i].b /= values.battery.u;
					u_abc[i].c /= values.battery.u;
				}

			}
			else
			{
				systems::abc tmp = {0};
				// faulty dc bus voltage
				u_abc.fill(tmp);
			}

			hardware::pwm::SetDutys(u_abc);
			palClearLine(LINE_HALL_C);
		}

	}
}/* namespace control */
