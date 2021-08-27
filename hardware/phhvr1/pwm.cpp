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

static void pwm_serve_dma_interrupt(PWMDriver *pwmp, uint32_t flags);

///< PWM driver instance
PWMDriver* PWMP = &PWMD1;

///< ADC trigger timer driver instance
PWMDriver* ADC_TRIGP = &PWMD4;

///< Pointer to the dma stream for duty update
const stm32_dma_stream_t* dmastp;

///< PWM duty counts
std::array<std::uint32_t, PHASES * INJECTION_CYCLES * 2> duty_counts = {0};

///< buffer index for the duty buffer
std::uint_fast8_t duty_index = 0;

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
float fc = hardware::pwm::TIMER_CLOCK/(DEF_PERIOD * INJECTION_CYCLES);


///< control period in s
float tc = (float)(DEF_PERIOD * INJECTION_CYCLES)/(float)hardware::pwm::TIMER_CLOCK;

///< Filter group delay
/// RC Filter with 11k and 1n plus 5us for the INA240
///  Software needs 1 control cycle for calculations
const float TF_HW = (510.0f * 1e-9f);
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
		STM32_TIM_DIER_UDE
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
 * @brief   PWM DMA ISR service routine.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void pwm_serve_dma_interrupt(PWMDriver *pwmp, uint32_t flags)
{
	(void)pwmp;
	/* DMA errors handling.*/
	if(flags & STM32_DMA_ISR_TCIF)		// Transfer complete
	{
//		palClearLine(LINE_LED_ERROR);
	}
	if(flags & STM32_DMA_ISR_HTIF)		// Half Transfer
	{
//		palSetLine(LINE_LED_ERROR);
	}
	if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
		/* DMA, this could help only if the DMA tries to access an unmapped
          address space or violates alignment rules.*/
		osalSysHalt("PWM DMA Error");
	}
}


/**
 * Initialize PWM hardware with outputs disabled!
 */
void hardware::pwm::Init(void)
{
	/*
	 * Set Debug register to stop TIM1 and TIM5 in DebugMode
	 */
	DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;
	DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM4_STOP;


	/* Start the PWM timer */
	pwmStart(PWMP, &pwmcfg);
	PWMP->tim->CR1 &=~ STM32_TIM_CR1_CEN; // timer stop
	PWMP->tim->CR1 |= STM32_TIM_CR1_CMS(2) | STM32_TIM_CR1_CKD(1); // center aligned mode and deadtime generation with clock/2
	PWMP->tim->SMCR |= STM32_TIM_SMCR_MSM;
//	PWMP->tim->CR1 &= ~STM32_TIM_CR1_URS; // every thing is an update event

	/* start the ADC trigger timer */
	pwmStart(ADC_TRIGP, &adctriggercfg);
	ADC_TRIGP->tim->CR1 &=~ STM32_TIM_CR1_CEN; // timer stop
	ADC_TRIGP->tim->SMCR |= STM32_TIM_SMCR_SMS(4) | STM32_TIM_SMCR_MSM;
	ADC_TRIGP->tim->CR1 |= STM32_TIM_CR1_CEN; // adc trigger timer start again

	/* DMA setup.*/
	dmastp = dmaStreamAlloc(STM32_DMA_STREAM_ID(2, 5), 3,
							(stm32_dmaisr_t)pwm_serve_dma_interrupt, (void*)PWMP);
	osalDbgAssert(dmastp != nullptr, "unable to allocate stream");

	PWMP->tim->DCR = STM32_TIM_DCR_DBL(2) | STM32_TIM_DCR_DBA(0xD); //
	PWMP->tim->CR1 |= STM32_TIM_CR1_CEN; // pwm timer start again

	dmaStreamSetPeripheral(dmastp, &PWMP->tim->DMAR);
	dmaStreamSetMemory0(dmastp, duty_counts.data());

	dmaStreamSetTransactionSize(dmastp, duty_counts.size());

	std::uint32_t dmamode = STM32_DMA_CR_CHSEL(6)	 |
							STM32_DMA_CR_MSIZE_WORD  | STM32_DMA_CR_PSIZE_WORD  |
	                  	  	STM32_DMA_CR_PL(3)       | STM32_DMA_CR_MINC        |
							STM32_DMA_CR_DIR_M2P     | STM32_DMA_CR_CIRC        |
							STM32_DMA_CR_HTIE        | STM32_DMA_CR_TCIE		|
							STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE;

	dmaStreamSetMode(dmastp, dmamode);

	dmaStreamEnable(dmastp);

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

	fc = (float)TIMER_CLOCK / (float)(period * INJECTION_CYCLES);
	tc = (float)(period * INJECTION_CYCLES) / (float)TIMER_CLOCK;
	tf = tc + TF_HW;

	frequency = (std::uint32_t)(fc * 2.0f);

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
	return tf;
}

/**
 * Enable PWM Outputs.
 * @note Power output is active after this call
 */
void hardware::pwm::output::Enable(void)
{
	palSetLine(LINE_GATE_EN);

	// reset the break state
	PWMP->tim->BDTR |= STM32_TIM_BDTR_AOE;
}

/**
 * Disable PWM Outputs.
 */
void hardware::pwm::output::Disable(void)
{
	palSetLine(LINE_GATE_EN);
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
void hardware::pwm::Duty(const std::array<systems::abc, INJECTION_CYCLES>& dutys)
{
	duty_index++;
	if(duty_index > 1) duty_index = 0;

	for (std::uint_fast8_t i = 0; i < INJECTION_CYCLES; ++i)
	{
		for (uint16_t p = 0; p < PHASES; ++p)
		{
			int16_t new_duty = (int16_t)((float)period * dutys[i].array[p]);

			if(new_duty < 0) new_duty = 0;
			if(new_duty > (int16_t)period) new_duty = period;

			duty_counts[p + 3*(i + (INJECTION_CYCLES * duty_index))] = new_duty;
		}
	}

	cacheBufferFlush(duty_counts.data(), sizeof(duty_counts));
}



