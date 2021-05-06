/*
    UNIMOC - Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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
#include "filter.hpp"
#include "hal.h"
#include "hardware_interface.hpp"
#include "management.hpp"
#include "measurement.hpp"
#include "control_thread.hpp"

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

		///< measure stator resistance
		bool resitance = false;

		///< measure stator inductance
		bool inductance = false;

		///< measure rotor flux
		bool flux = false;

		///< measure hall sensor state positons
		bool hall = false;
	}
} /* namespace management */

/**
 * derate control input envelope
 * @param limit	value to end derating
 * @param envelope positive value sets the envelope below the limit, negative above the limit
 * @param actual actual value
 * @return 1 when no derating active and 1 to 0 when in envelope and 0 when above limit
 */
float management::thread::Derate(const float limit, const float envelope, const float actual)
{
	const float start = limit - envelope;

	float derating = (actual - start)/envelope;

	if(derating < 0.0f) derating = 0.0f;
	if(derating > 1.0f) derating = 1.0f;

	// cut off the derating value from the maximum
	return (1.0f - derating);
}

/**
 * limit the input value
 * @param[in/out] in input value
 * @param min minimal value
 * @param max maximal value
 * @return true when value is out of limits
 */
bool management::thread::Limit(float& in, const float min, const float max)
{
	bool did_trunc = false;

	if (in > max)
	{
		in = max;
		did_trunc = true;

	}
	else if (in < min)
	{
		in = min;
		did_trunc = true;
	}

	return did_trunc;
}


/**
 * generic constructor
 */
management::thread::thread(): deadline(0), sequencer(STARTUP), uq(1e3f, 1.0f, 10e-3f), ubat(1e3f, 1.0f, 10e-3f), w(1e3f, 1.0f, 10e-3f)
{};

/**
 * Limit the current setpoint according to temp, voltage, and current limits
 * @param setpoint[in/out] current setpoint
 */
void management::thread::LimitCurrentSetpoint(systems::dq& setpoint)
{
	using namespace values::motor::rotor;
	using namespace values;

	float ratio = ubat.Calculate(battery::u) / uq.Calculate(u.q);
	float min = -settings.motor.limits.i;
	float max =  settings.motor.limits.i;

	// deadzone for battery current limiter of 1W
	if(std::fabs(uq.Get() * setpoint::i.q) > 1.0f)
	{
		if(ratio > 1e-3f)
		{
			min = ratio * -settings.battery.limits.i.charge;
			max = ratio * settings.battery.limits.i.drive;
		}
		else if(ratio < -1e-3f)
		{
			min = ratio * settings.battery.limits.i.drive;
			max = ratio * -settings.battery.limits.i.charge;
		}

		if(min < -settings.motor.limits.i) min = -settings.motor.limits.i;
		if(max > settings.motor.limits.i) max =  settings.motor.limits.i;
	}

	// derate by temperature and voltage
	std::array<float, 5> derate;
	derate[0] = Derate(settings.converter.limits.temperature,
			settings.converter.derating.temprature, converter::temp);

	derate[1] = Derate(settings.battery.limits.voltage.low,
			-settings.converter.derating.voltage, ubat.Get());

	derate[2] = Derate(settings.battery.limits.voltage.high,
					settings.converter.derating.voltage, ubat.Get());

	w.Calculate(omega);
	derate[3] = Derate(settings.motor.limits.omega.forwards,
			settings.converter.derating.omega, w.Get());

	derate[4] = Derate(settings.motor.limits.omega.backwards,
			-settings.converter.derating.omega, w.Get());

	// always use the minimal derating possible
	float derating = *std::min_element(derate.begin(), derate.end());

	// derate the limits
	min *= derating;
	max *= derating;

	motor::rotor::setpoint::limit::i::min = min;
	motor::rotor::setpoint::limit::i::max = max;

	Limit(setpoint.q, motor::rotor::setpoint::limit::i::min, motor::rotor::setpoint::limit::i::max);
}

/**
 * calculate analog input signal with deadzones
 * @param input 0-1 input
 * @return	0-1 output with dead zones
 */
float management::thread::UniAnalogThrottleDeadzone(const float input)
{
	float output = (input - settings.throttle.deadzone.low)/(settings.throttle.deadzone.high - settings.throttle.deadzone.low);
	Limit(output, 0.0f, 1.0f);
	return output;
}

/**
 * calculate analog input signal with dead zones in bidirectional manner
 * @param input 0-1 input
 * @return	-1-1 output with dead zones at 0 and +-1
 */
float management::thread::BiAnalogThrottleDeadzone(const float input)
{
	float bi_input = (input *2.0f) - 1.0f;
	// low dead zone bidirectional
	if(std::fabs(bi_input) < settings.throttle.deadzone.low) bi_input = 0.0f;
	else bi_input -= std::copysign(settings.throttle.deadzone.low, bi_input);
	// scaling for high dead zone
	float output = bi_input/(settings.throttle.deadzone.high - settings.throttle.deadzone.low);
	// cut of high dead zone
	Limit(output, -1.0f, 1.0f);

	return output;
}

/**
 * get the mapped throttle input signal
 * @param setpoint
 */
void management::thread::SetThrottleSetpoint(systems::dq& setpoint)
{
	using namespace values::motor::rotor;

	switch(settings.throttle.sel)
	{
	default:
	case settings_ts::throttle_s::NONE:
		break;
	case settings_ts::throttle_s::ANALOG_BIDIRECTIONAL:
	{
		setpoint.d = 0.0f;
		float throttle = BiAnalogThrottleDeadzone(hardware::adc::input());

		if(throttle > 0.0f) setpoint.q = throttle * setpoint::limit::i::max;
		else setpoint.q = throttle * setpoint::limit::i::min;

		break;
	}

	case settings_ts::throttle_s::ANALOG_FORWARDS:
		setpoint.d = 0.0f;
		setpoint.q = UniAnalogThrottleDeadzone(hardware::adc::input()) * setpoint::limit::i::max;
		break;
	case settings_ts::throttle_s::ANALOG_BACKWARDS:
		setpoint.d = 0.0f;
		setpoint.q = UniAnalogThrottleDeadzone(hardware::adc::input()) * setpoint::limit::i::min;
		break;
	case settings_ts::throttle_s::ANALOG_SWITCH:
		break;
	case settings_ts::throttle_s::PAS:
		break;
	}
}


/**
 * @brief Thread main function
 */
void management::thread::main(void)
{
	setName("Management");


	deadline = System::getTime();

	/*
	 * Normal main() thread activity
	 */
	while (TRUE)
	{
		using namespace values;
		deadline = System::getTime();

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

			// slow tasks of the control loop thread
			controller.Manage();

			// get throttle commands from the configured source
			SetThrottleSetpoint(motor::rotor::setpoint::i);

			// calculate the limits of the current setpoint an apply them
			LimitCurrentSetpoint(motor::rotor::setpoint::i);

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
				measure::resitance = true;
				measure::inductance = true;
				measure::flux = true;
			}

			if(measure::resitance) sequencer = MEASURE_RS;
			else if(measure::inductance) sequencer = MEASURE_LS;
			else if(measure::flux) sequencer = MEASURE_PSI;

			break;

		case MEASURE_RS:

			if(measurement::Run(measure::resitance, measurement::r::Run))
			{
				sequencer = RUN;
			}
			break;

		case MEASURE_LS:
			if(measurement::Run(measure::inductance, measurement::l::Run))
			{
				sequencer = RUN;
			}
			break;

		case MEASURE_PSI:
			if(measurement::Run(measure::flux, measurement::psi::Run))
			{
				sequencer = RUN;
			}
			break;
		}

		sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
	}
}




