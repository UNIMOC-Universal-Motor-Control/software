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
#include "pwm.hpp"
#include "adc.hpp"
#include "hardware_interface.hpp"

///< PWM driver instance
PWMDriver* hardware::pwm::PWMP = &PWMD1;

///< ADC trigger timer driver instance
PWMDriver* ADC_TRIGP = &PWMD4;

///< PWM duty counts
uint16_t hardware::pwm::duty_counts[PHASES] = {0};

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
		hardware::pwm::TIMER_CLOCK,
		hardware::pwm::PERIOD,
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
		STM32_TIM_BDTR_DTG(DTG(hardware::pwm::DEADTIME)) | STM32_TIM_BDTR_BKE | STM32_TIM_BDTR_BKF(3),
		/*
		 * DIER Register
		 */
		0
};

/**
 * callback triggers the control task
 *
 * @note callback also invalidates cache for samples section recently sampled
 *
 * @param pwmp
 */
static inline void control_trigger(PWMDriver *pwmp)
{
	(void)pwmp;
	palToggleLine(LINE_HALL_B);

	/* DMA buffer invalidation because data cache, only invalidating the
     * buffer just filled.
     */
	for (uint8_t i = 0; i < hardware::PHASES; ++i)
	{
		cacheBufferInvalidate(&hardware::adc::samples[i][hardware::adc::samples_index][0],
				sizeof(adcsample_t)*hardware::adc::LENGTH_ADC_SEQ);
	}


	if(hardware::adc::samples_index < (hardware::adc::ADC_SEQ_BUFFERED - 1)) hardware::adc::samples_index++;
	else hardware::adc::samples_index = 0;


	if(!hardware::control_thread.isNull())
	{
		/* Wakes up the thread.*/
		osalSysLockFromISR();
		chEvtSignalI(hardware::control_thread.getInner(), (eventmask_t)1);
		osalSysUnlockFromISR();
	}
}

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
				{PWM_OUTPUT_ACTIVE_HIGH, control_trigger}
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

	/* set adc trigger offset */
	pwmEnableChannel(ADC_TRIGP, 3, PERIOD/4 - 50);
	pwmEnableChannelNotification(ADC_TRIGP, 3);

	/* enable pwms */
	pwmEnableChannel(PWMP, 0, PERIOD/2);
	pwmEnableChannel(PWMP, 1, PERIOD/2);
	pwmEnableChannel(PWMP, 2, PERIOD/2);
	duty_counts[0] = PERIOD/2;
	duty_counts[1] = PERIOD/2;
	duty_counts[2] = PERIOD/2;

}

/**
 * Enable PWM Outputs.
 * @note Power output is active after this call
 */
void hardware::pwm::EnableOutputs(void)
{
	palSetLine(LINE_EN_PWM_OUT);

	// reset the break state
	PWMP->tim->BDTR |= STM32_TIM_BDTR_AOE;
}

/**
 * Disable PWM Outputs.
 */
void hardware::pwm::DisableOutputs(void)
{
	palClearLine(LINE_EN_PWM_OUT);
}

/**
 * Get pwm output state
 * @return pwm output state, true = pwm active
 */
bool hardware::pwm::OutputActive(void)
{
	return (PWMP->tim->BDTR & STM32_TIM_BDTR_MOE);
}

/**
 * Set the normalized duty cycles for each phase
 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
 */
void hardware::pwm::SetDutys(float dutys[PHASES])
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



