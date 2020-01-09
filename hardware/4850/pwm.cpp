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
#include "pwm.hpp"
#include "hardware_interface.hpp"

///< PWM driver instance
constexpr PWMDriver* PWMP = &PWMD1;

///< PWM duty counts
uint16_t unimoc::hardware::pwm::duty_counts[PHASES] = {0};

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
 * Callback for timer over/unterflow interrupt
 * @param pwmp PWM driver instance
 */
static void period_callback(PWMDriver *pwmp)
{
	(void)pwmp;
	unimoc::hardware::adc::Start();
}

/**
 * basic PWm configuration
 */
const PWMConfig pwmcfg =
{
		unimoc::hardware::pwm::TIMER_CLOCK,
		unimoc::hardware::pwm::PERIOD,
		period_callback,
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
		STM32_TIM_BDTR_DTG(DTG(unimoc::hardware::pwm::DEADTIME)) | STM32_TIM_BDTR_BKE | STM32_TIM_BDTR_BKF(3),
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

	/* enable pwms */
	pwmEnableChannel(PWMP, 0, PERIOD/2);
	pwmEnableChannel(PWMP, 1, PERIOD/2);
	pwmEnableChannel(PWMP, 2, PERIOD/2);

	/* enable period irq */
	pwmEnablePeriodicNotification(PWMP);
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
	const int16_t mid = PERIOD/2;

	for (uint16_t i = 0; i < PHASES; ++i)
	{
		int16_t new_duty = mid + (int16_t)((float)mid * dutys[i]);

		if(new_duty < 0) new_duty = 0;
		if(new_duty > (int16_t)PERIOD) new_duty = PERIOD;

		duty_counts[i] = (uint16_t)new_duty;

		pwmEnableChannel(PWMP, i, duty_counts[i]);
	}
}



