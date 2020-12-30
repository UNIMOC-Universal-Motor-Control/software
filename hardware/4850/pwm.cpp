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

///< default PWM period in counts
const std::uint32_t DEF_PERIOD = 6750;

///< default deadtime in nanoseconds
const std::uint32_t DEF_DEADTIME = 800;

///< deadtime in nano seconds
uint32_t deadtime = DEF_DEADTIME;

/**
 * PWM counts
 *
 * With center aligned PWM this is the period of PWM half period.
 */
uint32_t period = DEF_PERIOD;

/**
 * PWM frequency in Hz
 *
 * With center aligned PWM this is the period of PWM half period.
 */
uint32_t frequency = hardware::pwm::TIMER_CLOCK/(2*DEF_PERIOD);

///< control frequency in Hz
float fc = hardware::pwm::TIMER_CLOCK/(DEF_PERIOD);


///< control period in s
float tc = (float)DEF_PERIOD/(float)hardware::pwm::TIMER_CLOCK;

///< Filter group delay
/// RC Filter with 11k and 1n plus 5us for the INA240
///  Software needs 1 control cycle for calculations
const float TF_HW = (11e3f * 1e-9f) + 5e-6f;
float tf = tc + TF_HW;

/**
 * basic PWM configuration
 */
PWMConfig pwmcfg =
{
		TIMER_CLOCK,
		6750,
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
		STM32_TIM_BDTR_DTG(250) | STM32_TIM_BDTR_BKE | STM32_TIM_BDTR_BKF(3),
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
		 */
		0,
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
	PWMP->tim->CR1 |= STM32_TIM_CR1_CMS(2) | STM32_TIM_CR1_CKD(1); // center aligned mode and deadtime generation with clock/2
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
	pwmEnableChannel(PWMP, 0, period/2);
	pwmEnableChannel(PWMP, 1, period/2);
	pwmEnableChannel(PWMP, 2, period/2);
}


/**
 * Get Deadtime of PWM
 * @return deadtime in nano seconds
 */
std::uint32_t hardware::pwm::Deadtime(void)
{
	return deadtime;
}

/**
 * Set Deadtime of PWM
 *
 * @param ns set deadtime in nano seconds to this value
 * @return new deadtime in nano seconds
 */
std::uint32_t hardware::pwm::Deadtime(const std::uint32_t ns)
{
	// Deadtime generator counts is half the timer clock due to CR1 CKD = 1
	std::uint16_t dtg = ((float)ns * (float)TIMER_CLOCK * 5e-8f);

	if(dtg > 255)
	{
		dtg = 255;
	}

	deadtime = (dtg * 5e8f) / TIMER_CLOCK;

	PWMP->tim->BDTR &= ~STM32_TIM_BDTR_DTG_MASK;
	PWMP->tim->BDTR |= STM32_TIM_BDTR_DTG(dtg);

	return deadtime;
}

/**
 * Get Period of PWM
 *
 * @note With center aligned PWM this is the period of PWM half period.
 * @return Period of PWM in timer counts
 */
std::uint32_t hardware::pwm::Period(void)
{
	return period;
}

/**
 * Get Frequency of PWM
 *
 * @return PWM frequency in Hz
 */
std::uint32_t hardware::pwm::Frequency(void)
{
	return frequency;
}

/**
 * Set Frequency of PWM
 *
 * @param freq 	new PWM frequency
 * @return new PWM frequency in Hz
 */
std::uint32_t hardware::pwm::Frequency(const std::uint32_t freq)
{
	period = (std::uint32_t)((float)TIMER_CLOCK / (2.0f*(float)freq));

	if(period > 0xffff) period = 0xffff;
	if(period < 1000) period = 1000;

	pwmChangePeriod(PWMP, period);

	fc = (float)TIMER_CLOCK / (float)(period);
	tc = (float)(period) / (float)TIMER_CLOCK;

	frequency = (std::uint32_t)(fc * 0.5f);

	return frequency;
}


/**
 * get control cycle time
 * @return control cycle frequency in s
 */
float hardware::Tc(void)
{
	return tc;
}

/**
 * get control cycle frequency
 * @return control cycle frequency in Hz
 */
float hardware::Fc(void)
{
	return fc;
}

/**
 * Get Filters Group delay (Hardware and Software)
 * @return Group delay of the hole signal chain in s
 */
float hardware::Tf(void)
{
	tf = tc + TF_HW;
	return tf;
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
 * @param dutys 0 = LOW, 1=HIGH
 */
void hardware::pwm::Duty(const systems::abc& dutys)
{

	for (uint16_t i = 0; i < PHASES; ++i)
	{
		int16_t new_duty = (int16_t)((float)period * dutys.array[i]);

		if(new_duty < 0) new_duty = 0;
		if(new_duty > (int16_t)period) new_duty = period;

		pwmEnableChannel(PWMP, i, new_duty);
	}
}



