/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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

///< Pointer to the dma stream for duty update
const stm32_dma_stream_t* dmastp;

///< PWM Timer clock in Hz
const std::uint32_t hardware::pwm::TIMER_CLOCK = STM32_TIMCLK2;

///< default PWM period in counts
const std::uint32_t DEF_PERIOD = 6750;

///< default deadtime in nanoseconds
const std::uint32_t DEF_DEADTIME = 1000;

///< deadtime in nano seconds
std::uint32_t deadtime = DEF_DEADTIME;

///< adc sampling offset
const std::uint32_t ADC_SAMPLING_OFFSET = 3.5e-6f * (float)hardware::pwm::TIMER_CLOCK;

/**
 * PWM counts
 *
 * With center aligned PWM this is the period of PWM half period.
 */
std::uint32_t period = DEF_PERIOD;

/**
 * PWM frequency in Hz
 *
 * With center aligned PWM this is the period of PWM half period.
 */
std::uint32_t frequency = hardware::pwm::TIMER_CLOCK/(2*DEF_PERIOD);

///< control frequency in Hz
float fc = hardware::pwm::TIMER_CLOCK/(DEF_PERIOD);


///< control period in s
float tc = (float)(DEF_PERIOD)/(float)hardware::pwm::TIMER_CLOCK;

///< Filter group delay
/// RC Filter with 11k and 1n plus 5us for the INA240
const float TF_HW_CUR = (510.0f * 1e-9f);
/// RC Filter with soem Resistior and Cap. TODO
const float TF_HW_VOLT = (510.0f * 1e-9f);

///  Software needs 1 control cycle for calculations
float ctf = tc + TF_HW_CUR;
float vtf = tc + TF_HW_VOLT;

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
void hardware_pwm_Init(void)
{
	/*
	 * Set Debug register to stop TIM1 and TIM4 in DebugMode
	 */
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP;

	/* Start the PWM timer */
	pwmStart(PWMP, &pwmcfg);
	PWMP->tim->CR1 &=~ STM32_TIM_CR1_CEN; // timer stop
	PWMP->tim->CR1 |= STM32_TIM_CR1_CMS(2) | STM32_TIM_CR1_CKD(1); // center aligned mode and deadtime generation with clock/2
	PWMP->tim->SMCR |= STM32_TIM_SMCR_MSM;
	PWMP->tim->CR1 &= ~STM32_TIM_CR1_URS; // every thing is an update event
	PWMP->tim->CR1 |= STM32_TIM_CR1_CEN; // timer start again

	/* start the ADC trigger timer */
	pwmStart(ADC_TRIGP, &adctriggercfg);
	ADC_TRIGP->tim->CR1 &=~ STM32_TIM_CR1_CEN; // timer stop
	ADC_TRIGP->tim->SMCR |= STM32_TIM_SMCR_SMS(4) | STM32_TIM_SMCR_MSM;
	ADC_TRIGP->tim->CR1 |= STM32_TIM_CR1_CEN; // adc trigger timer start again

	/* set adc trigger offset = minimal */
	pwmEnableChannel(ADC_TRIGP, 3, ADC_SAMPLING_OFFSET);

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
	std::uint16_t dtg = ((float)ns * (float)TIMER_CLOCK * 5e-10f);

	if(dtg > 255)
	{
		dtg = 255;
	}

	deadtime = (dtg * 2e9f) / TIMER_CLOCK;

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
	ctf = tc + TF_HW_CUR;
	vtf = tc + TF_HW_VOLT;

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
float hardware::analog::current::Tf(void)
{
	return ctf;
}

/**
 * Get Filters Group delay (Hardware and Software)
 * @return Group delay of the hole signal chain in s
 */
float hardware::analog::voltage::Tf(void)
{
	return vtf;
}

/**
 * Enable PWM Outputs.
 * @note Power output is active after this call
 */
void hardware::pwm::output::Enable(void)
{
	// reset the break state
	PWMP->tim->BDTR |= STM32_TIM_BDTR_AOE;
}

/**
 * Disable PWM Outputs.
 */
void hardware::pwm::output::Disable(void)
{
	PWMP->tim->BDTR &= ~STM32_TIM_BDTR_MOE;
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
	for (uint16_t p = 0; p < PHASES; ++p)
	{
		int16_t new_duty = (int16_t)((float)period * dutys.array[p]);

		if(new_duty < 0) new_duty = 0;
		if(new_duty > (int16_t)period) new_duty = period;

		pwmEnableChannel(PWMP, p, new_duty);
	}
}



