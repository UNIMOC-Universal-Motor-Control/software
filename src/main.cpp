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
#include "ch.hpp"
#include "hal.h"
#include "hardware_interface.hpp"
//#include "uavcan.hpp"
#include "main.hpp"

using namespace chibios_rt;

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

	hardware::control_thread = controller.start(HIGHPRIO);

	/*
	 * initialize hardware with no control thread
	 */
	hardware::memory::Init();
	hardware::pwm::Init();
	hardware::adc::Init();
//	uavcan::Init();

//	settings.Load();

	hardware::pwm::EnableOutputs();

	// set node operational
	values.uavcan.mode = uavcan::mode_e::UAVCAN_NODE_MODE_OPERATIONAL;


	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (true)
	{
		values.converter.temp = hardware::adc::GetBridgeTemp();
		values.motor.temp = hardware::adc::GetMotorTemp();


//		if(save) settings.Save();
//				save = false;

		if(hardware::pwm::OutputActive())
		{
			palSetLine(LINE_LED_PWM);
		}
		else
		{
			palClearLine(LINE_LED_PWM);
		}

		uavcan::Run();
	}
}

namespace control
{
	/**
	 * generic constructor
	 */
	thread::thread():flux(), mech(settings.observer.Q, settings.observer.R),
			/*
			 * R1 = 1k7, R2 = 47k, R3 = 4k7, C1 = 10n, C2 = 1n
			 * Q = 0.47619047619048, Fc = 3386.2753849339
			 */
			foc(0.47619047619048f, 3386.2753849339f/hardware::Fc * hardware::pwm::INJECTION_CYCLES, 0.5f, 0.01f)
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

				if(settings.control.current.active)
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
				}

				if(settings.control.current.active)
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
		}

	}
}/* namespace control */

