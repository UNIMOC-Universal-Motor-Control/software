/*
    UNIMOC - Universal Motor Control  2019 Alexander <tecnologic86@gmail.com>

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

///< pwm frequency in Hz
uint32_t frequency = STOCK_PWM_FREQUENCY;

/**
 * macro to calculate auto reload register value
 * @param freq_hz PWM frequency in Hz
 * @return ARR value in timer clock ticks
 */
constexpr uint32_t COUNTER_MAX(uint32_t freq_hz)
{
	return (PWM_TIMER_CLOCK/freq_hz - 1);
}

/**
 * basic PWm configuration
 */
const PWMConfig pwmcfg =
{
		PWM_TIMER_CLOCK,
		COUNTER_MAX(STOCK_PWM_FREQUENCY),
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
		 */
		STM32_TIM_BDTR_DTG((uint16_t)(PWM_DEADTIME * (float32_t)STM32_TIMCLK2 -1)),
		/*
		 * DIER Register
		 */
		0
};

/**
 * Initialize PWM hardware with outputs disabled!
 * @param freq_hz PWM frequency in Hz
 */
void unimoc::hardware::pwm::Init(uint32_t freq_hz)
{
	/*
	 * Set Debug register to stop Tim1 in DebugMode
	 */
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
	/* Start the PWM timer */
	pwmStart(pwmp, &pwmcfg);
	pwmp->tim->CR1 &=~ STM32_TIM_CR1_CEN; // timer stop
	pwmp->tim->CR1 |= STM32_TIM_CR1_CMS(2); // center aligned mode
	pwmp->tim->CR1 |= STM32_TIM_CR1_CEN; // timer start again

	/* enable outputs */
	pwmEnableChannel(pwmp, 0, COUNTER_MAX_INT/2);
	pwmEnableChannel(pwmp, 1, COUNTER_MAX_INT/2);
	pwmEnableChannel(pwmp, 2, COUNTER_MAX_INT/2);
}

/**
 * Set PWM Freqency
 * @param freq_hz PWM frequency in Hz
 */
void unimoc::hardware::pwm::SetFreqency(uint32_t freq_hz)
{

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

}

/**
 * Disable PWM Outputs.
 */
void unimoc::hardware::pwm::DisableOutputs(void)
{

}

/**
 * Set the normalized duty cycles for each phase
 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
 */
void unimoc::hardware::pwm::SetDutys(float dutys[PHASES])
{

}



