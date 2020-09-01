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
#include <cstdint>
#include "hardware_interface.hpp"
#include "systems.hpp"

using namespace hardware;
using namespace hardware::pwm;

///< PWM driver instance
PWMDriver* PWMP = &PWMD1;

///< ADC trigger timer driver instance
PWMDriver* ADC_TRIGP = &PWMD4;

///< PWM duty counts
systems::abc duty_counts = {0};


///< PWM Timer clock in Hz
const uint32_t hardware::pwm::TIMER_CLOCK = STM32_TIMCLK2;


///< deadtime in nano seconds
const uint32_t hardware::pwm::DEADTIME = 300;

/**
 * PWM frequency in Hz
 *
 * With center aligned PWM this is the period of PWM half period.
 */
const uint32_t hardware::pwm::PERIOD = 6749;

///< Control cycle time
const float hardware::Tc = ((float)(hardware::pwm::PERIOD + 1) / (float)hardware::pwm::TIMER_CLOCK);

///< Control cycle frequency
const float hardware::Fc = 1.0f/hardware::Tc;

/**
 * macro to calculate DTG value for BDTR
 * @param deadtime in ns
 * @return dtg value
 */
constexpr uint16_t DTG(const uint32_t deadtime)
{
	float fdeadtime = (float)deadtime * 1e-9f;
	float clock = (float)STM32_TIMCLK2;
	osalDbgAssert((fdeadtime * clock) < 256.0f,
	                "PWM: Dead too long");
	return (uint16_t)((fdeadtime * clock) -1);
}


/**
 * basic PWM configuration
 */
const PWMConfig pwmcfg =
{
		TIMER_CLOCK,
		PERIOD,
		nullptr,
		{ /*  */
				{PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, nullptr},
				{PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, nullptr},
				{PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, nullptr},
				{PWM_OUTPUT_DISABLED, nullptr}
		},
		/*
		 * CR2 Register
		 * Master mode update event on TRGO
		 */
		STM32_TIM_CR2_MMS(2),
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
 * Configuration for ADC Trigger Timer
 */
const PWMConfig adctriggercfg =
{
		STM32_TIMCLK1,
		0xFFFE,
		nullptr,
		{ /*  */
				{PWM_OUTPUT_DISABLED, nullptr},
				{PWM_OUTPUT_DISABLED, nullptr},
				{PWM_OUTPUT_DISABLED, nullptr},
				{PWM_OUTPUT_ACTIVE_HIGH, nullptr}
		},
		/*
		 * CR2 Register
		 * CH4 on TRGO
		 */
		STM32_TIM_CR2_MMS(7),
		/*
		 * Break and Deadtime Register
		 */
		0,
		/*
		 * DIER Register
		 */
		0
};


/**
 * Initialize PWM hardware with outputs disabled!
 */
void hardware::pwm::Init(void)
{
	/*
	 * Set Debug register to stop TIM1 and TIM5 in DebugMode
	 */
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP;
	/* Start the PWM timer */
	pwmStart(PWMP, &pwmcfg);
	PWMP->tim->CR1 &=~ STM32_TIM_CR1_CEN; // timer stop
	PWMP->tim->CR1 |= STM32_TIM_CR1_CMS(2); // center aligned mode
	PWMP->tim->SMCR |= STM32_TIM_SMCR_MSM;
	PWMP->tim->CR1 &= ~STM32_TIM_CR1_URS; // every thing is an update event

	/* start the ADC trigger timer */
	pwmStart(ADC_TRIGP, &adctriggercfg);
	ADC_TRIGP->tim->CR1 &=~ STM32_TIM_CR1_CEN; // timer stop
	ADC_TRIGP->tim->SMCR |= STM32_TIM_SMCR_SMS(4) | STM32_TIM_SMCR_MSM;
	ADC_TRIGP->tim->CR1 |= STM32_TIM_CR1_CEN; // adc trigger timer start again

	PWMP->tim->CR1 |= STM32_TIM_CR1_CEN; // pwm timer start again

	/* set adc trigger offset = minimal */
	pwmEnableChannel(ADC_TRIGP, 3, 1);

	/* enable pwms */
	pwmEnableChannel(PWMP, 0, PERIOD/2);
	pwmEnableChannel(PWMP, 1, PERIOD/2);
	pwmEnableChannel(PWMP, 2, PERIOD/2);
}

/**
 * Enable PWM Outputs.
 * @note Power output is active after this call
 */
void hardware::pwm::output::Enable(void)
{
	palSetLine(LINE_EN_PWM_OUT);

	// reset the break state
	PWMP->tim->BDTR |= STM32_TIM_BDTR_AOE;
}

/**
 * Disable PWM Outputs.
 */
void hardware::pwm::output::Disable(void)
{
	palClearLine(LINE_EN_PWM_OUT);
}

/**
 * Get pwm output state
 * @return pwm output state, true = pwm active
 */
bool hardware::pwm::output::Active(void)
{
	return (PWMP->tim->BDTR & STM32_TIM_BDTR_MOE);
}

/**
 * Set the normalized duty cycles for each phase
 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
 */
void hardware::pwm::Duty(const systems::abc& dutys)
{
	constexpr int16_t mid = PERIOD/2;

	for (uint16_t i = 0; i < PHASES; ++i)
	{
		int16_t new_duty = mid + (int16_t)((float)mid * dutys.array[i]);

		if(new_duty < 0) new_duty = 0;
		if(new_duty > (int16_t)PERIOD) new_duty = PERIOD;

		pwmEnableChannel(PWMP, i, new_duty);
	}
}



