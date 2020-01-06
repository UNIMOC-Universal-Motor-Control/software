/*
    UNIMOC - Universal Motor Control  2019 Alexander <tecnologic86@gmail.com> Brand

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
#include <cstdint>
#include "hardware_interface.hpp"
#include "hal.h"

///< PWM Timer clock in Hz
constexpr uint32_t PWM_TIMER_CLOCK = STM32_TIMCLK2;


///< deadtime in nano seconds
constexpr uint32_t DEADTIME = 300;

///< pwm frequency in Hz
constexpr uint32_t STOCK_PWM_FREQUENCY = 32000;

///< pwm driver instance
constexpr PWMDriver* PWMP = &PWMD1;

///< pwm frequency in Hz
uint32_t frequency = STOCK_PWM_FREQUENCY;

/**
 * macro to calculate auto reload register value
 * @param freq_hz PWM frequency in Hz
 * @return ARR value in timer clock ticks
 */
constexpr uint32_t ARR(const uint32_t freq_hz)
{
	return (PWM_TIMER_CLOCK/(2*freq_hz) - 1);
}

/**
 * macro to calculate frequency from auto reload register value
 * @param arr auto reload register value
 * @return pwm frequency in Hz
 */
constexpr uint32_t FREQUENCY(const uint32_t arr)
{
	return (PWM_TIMER_CLOCK/((arr + 1)*2));
}

/**
 * macro to calculate DTG value for BDTR
 * @param deadtime in ns
 * @return dtg value
 */
constexpr uint16_t DTG(const uint32_t deadtime)
{
	float fdeadtime = (float)deadtime * 1e-9f;
	float clock = (float)STM32_TIMCLK2;
	osalDbgAssert((fdeadtime * clock) >= 256.0f,
	                "PWM: Dead too long");
	return (uint16_t)((fdeadtime * clock) -1);
}

/**
 * basic PWm configuration
 */
const PWMConfig pwmcfg =
{
		PWM_TIMER_CLOCK,
		ARR(STOCK_PWM_FREQUENCY),
		NULL,
		{ /*  */
				{PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_DISABLED, NULL}
		},
		/*
		 * CR2 Register
		 *
		 */
		0,
		/*
		 * Break and Deadtime Register
		 * Break input enabled with filter of 8 clock cycles
		 */
		STM32_TIM_BDTR_DTG(DTG(DEADTIME)) | STM32_TIM_BDTR_BKE | STM32_TIM_BDTR_BKF(3),
		/*
		 * DIER Register
		 */
		0
};

/**
 * Initialize PWM hardware with outputs disabled!
 */
void unimoc::hardware::pwm::Init(void)
{
	/*
	 * Set Debug register to stop Tim1 in DebugMode
	 */
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
	/* Start the PWM timer */
	pwmStart(PWMP, &pwmcfg);
	PWMP->tim->CR1 &=~ STM32_TIM_CR1_CEN; // timer stop
	PWMP->tim->CR1 |= STM32_TIM_CR1_CMS(2); // center aligned mode
	PWMP->tim->CR1 |= STM32_TIM_CR1_CEN; // timer start again

	/* enable outputs */
	pwmEnableChannel(PWMP, 0, PWMP->period/2);
	pwmEnableChannel(PWMP, 1, PWMP->period/2);
	pwmEnableChannel(PWMP, 2, PWMP->period/2);
}

/**
 * Set PWM Freqency
 * @param freq_hz PWM frequency in Hz
 */
bool unimoc::hardware::pwm::SetFreqency(const uint32_t freq_hz)
{
	bool result = true;
	uint32_t new_counter_max = ARR(freq_hz);
	if(new_counter_max < 1000 || new_counter_max > 0xFFF0)
	{
		result  = false;
	}
	else
	{
		frequency = freq_hz;
		pwmChangePeriod(PWMP, new_counter_max);
		result = true;
	}
	return (result);
}

/**
 * Get PWM Frequency
 * @return PWM frequency in Hz
 */
uint32_t unimoc::hardware::pwm::GetFreqency(void)
{
	return (frequency);
}

/**
 * Enable PWM Outputs.
 * @note Power output is active after this call
 */
void unimoc::hardware::pwm::EnableOutputs(void)
{
	palSetLine(LINE_EN_PWM_OUT);

	// reset the break state
	PWMP->tim->BDTR |= STM32_TIM_BDTR_AOE;
}

/**
 * Disable PWM Outputs.
 */
void unimoc::hardware::pwm::DisableOutputs(void)
{
	palClearLine(LINE_EN_PWM_OUT);
}

/**
 * Get pwm output state
 * @return pwm output state, true = pwm active
 */
bool unimoc::hardware::pwm::OutputActive(void)
{
	return (PWMP->tim->BDTR & STM32_TIM_BDTR_MOE);
}

/**
 * Set the normalized duty cycles for each phase
 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
 */
void unimoc::hardware::pwm::SetDutys(float dutys[PHASES])
{
	int16_t mid = PWMP->period/2;

	for (uint16_t i = 0; i < PHASES; ++i)
	{
		int16_t new_duty = mid + (int16_t)((float)mid * dutys[i]);

		if(new_duty < 0) new_duty = 0;
		if(new_duty > (int16_t)PWMP->period) new_duty = PWMP->period;

		pwmEnableChannel(PWMP, i, (uint16_t)new_duty);
	}
}



