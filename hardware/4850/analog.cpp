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
#include <cstring>
#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "hardware_interface.hpp"
#include "hal.h"

using namespace hardware::analog;

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err);
static void adccallback(ADCDriver *adcp);
static float adc2ntc_temperature(const uint16_t adc_value);


/**
 * Structure from ADC Channel mapping
 *
 * Signal			Pin		ADC1	ADC2	ADC3
 * -----------------------------------------------
 * AIN_TORQUE		PC1		CH11	CH11	CH11
 * AIN_CUR_A_DC		PC2		CH12	CH12	CH12
 * AIN_CUR_A_AC		PC3		CH13	CH13	CH13
 * AIN_CUR_B_DC 	PA0		CH0		CH0		CH0
 * AIN_CUR_B_AC 	PA1		CH1		CH1		CH1
 * AIN_CUR_C_DC 	PA2		CH2		CH2		CH2
 * AIN_CUR_C_AC 	PA3		CH3		CH3		CH3
 * AIN_VDC			PA4		CH4		CH4		---
 * AIN_BRDG_TEMP	PA5		CH5		CH5		---
 * AIN_MOT_TEMP		PA6		CH6		CH6		---
 */
#define ADC_CH_TORQUE			ADC_CHANNEL_IN11
#define ADC_CH_CUR_A			ADC_CHANNEL_IN12
#define ADC_CH_CUR_A_AC			ADC_CHANNEL_IN13
#define ADC_CH_CUR_B			ADC_CHANNEL_IN0
#define ADC_CH_CUR_B_AC			ADC_CHANNEL_IN1
#define ADC_CH_CUR_C			ADC_CHANNEL_IN2
#define ADC_CH_CUR_C_AC			ADC_CHANNEL_IN3
#define ADC_CH_VDC				ADC_CHANNEL_IN4
#define ADC_CH_BRDG_TEMP		ADC_CHANNEL_IN5
#define ADC_CH_MOT_TEMP			ADC_CHANNEL_IN6


/**
 * ADC sampling strategy.
 *
 * ADC has 16 samples in a conversion sequence available
 * We have to sample the ac and dc component of the current
 * at the as much as we can, but we need to ignore the
 * disturbance introduced thoug the pwm edge.
 *
 * Thats why we fill the 16 samples conversion sequence with all
 * current samples except for the samples which will be disturbed
 * by the edge there we add the 2 non current samples per adc.
 *
 * For VRef we except that the sample time will violate the minimum
 * sample time of 10us required in the datasheet. Lets see how useful
 * the value still is with that.
 */

/**
 * @note C++ Black Magic :D
 * 		 compile time calculation of lookup table
 *
 * @ref https://stackoverflow.com/questions/19016099/lookup-table-with-constexpr
 */
typedef struct NTC_LUT_S
{
	///< nominal value of the NTC
	static constexpr float R0 = 10e3f;

	///< Pullup resistor
	static constexpr float Rp = 10e3f;

	///< Nominal Temperature in K
	static constexpr float T0 = 25.0f + 273.15f;

	///< NTCs beta coefficient
	static constexpr float B = 3950.0f;

	///< Resistor Value for infinite temperature
	static constexpr float R8 = R0 * std::exp(-B/T0);

	///< Number of NTC resistor LUT values
	static constexpr uint16_t TABLE_LEN = 128;

	///< NTC adc value to temperature in 0.01°C LUT
	std::array<int16_t, TABLE_LEN> table;

    constexpr NTC_LUT_S() : table()
    {
        for (uint32_t i = 0; i < TABLE_LEN; ++i)
        {
        	float r = Rp/((float)TABLE_LEN/(float)(i+1) - 1.0f);
        	float t = B/std::log(r/R8) - T0 + 25.0f;
        	table[i] = (int16_t)(t*100.0f);
        }
    }
}NTC_LUT_ST;

const NTC_LUT_ST NTC_TABLE;

///< Samples in ADC sequence.
/// Caution: samples are 16bit but the hole sequence must be 32 bit aligned!
///          so even length of sequence is best choice.
constexpr uint32_t LENGTH_ADC_SEQ = 16;

///< ADC sequences in buffer.
/// Caution: samples are 16bit but the hole sequence must be 32 bit aligned!
///          so even length of sequence is best choice.
/// @note: must be 2 to have the controller running twice per PWM period
constexpr uint32_t ADC_SEQ_BUFFERED = hardware::pwm::INJECTION_CYCLES * 2;

///< # of ADCs
constexpr uint32_t NUM_OF_ADC = 3;

///< absolute maximum current
constexpr float hardware::analog::current::MAX = 1.65f/(20.0f*(0.002f/3.0f));

///< Three 2mR shunts in parallel with a 20V/V gain map to 2048 per 1.65V
constexpr float ADC2CURRENT = hardware::analog::current::MAX /(2048.0f);

///< Currents are amplified by high pass
constexpr float ADC2DERIVATIVE = ADC2CURRENT * 44.0f;

///< Voltage divider
constexpr float ADC2VDC = (24e3f+1.2e3f)/1.2e3f * 3.3f/(4096.0f);


/* Note, the buffer is aligned to a 32 bytes boundary because limitations
   imposed by the data cache. Note, this is GNU specific, it must be
   handled differently for other compilers.
   Only required if the ADC buffer is placed in a cache-able area.*/
///< dma accessed buffer for the adcs, probably cached
__attribute__((aligned (32)))
std::array<std::array<std::array<adcsample_t, LENGTH_ADC_SEQ>, ADC_SEQ_BUFFERED>, NUM_OF_ADC> samples;

///< reference to thread to be woken up in the hardware control cycle.
chibios_rt::ThreadReference hardware::control_thread = nullptr;

///< current offsets
std::int32_t current_offset[hardware::PHASES];

///< current derivative offsets
std::int32_t derivative_offset[hardware::PHASES];


/**
 * ADC conversion group.
 * Mode:        Continuous, 3 samples of 2 channels
 * Channels:    AIN_CUR_A, AIN_VDC, AIN_REF.
 */
static ADCConversionGroup adcgrpcfg1 = {
		true,
		LENGTH_ADC_SEQ,
		adccallback,
		adcerrorcallback,
		0,                                                    /* CR1   */
		ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2,/* CR2   */
		ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3),                    /* SMPR1 */
		ADC_SMPR2_SMP_AN4(ADC_SAMPLE_3),                     /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |
		ADC_SQR1_SQ16_N(ADC_CH_CUR_A_AC) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_A_AC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_A_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_A_AC),                  	  /* SQR1  */
		ADC_SQR2_SQ12_N(ADC_CH_CUR_A_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_A_AC) |
		ADC_SQR2_SQ10_N(ADC_CH_VDC) |
		ADC_SQR2_SQ9_N(ADC_CH_VDC) |
		ADC_SQR2_SQ8_N(ADC_CH_VDC) |
		ADC_SQR2_SQ7_N(ADC_CH_VDC),                 	     /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_A) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_A) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_A) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_A) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_A) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_A)                            /* SQR3  */
};

/**
 * ADC conversion group.
 * Mode:        Continuous, 3 samples of 3 channels
 * Channels:    AIN_CUR_B, AIN_BRDG_TEMP, AIN_MOT_TEMP.
 */
static ADCConversionGroup adcgrpcfg2 = {
		true,
		LENGTH_ADC_SEQ,
		nullptr,
		adcerrorcallback,
		0,                                                    /* CR1   */
		ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2,/* CR2   */
		0,                                                    /* SMPR1 */
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3) |
		ADC_SMPR2_SMP_AN5(ADC_SAMPLE_3) |
		ADC_SMPR2_SMP_AN6(ADC_SAMPLE_3),                     /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |
		ADC_SQR1_SQ16_N(ADC_CH_CUR_B_AC) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_B_AC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_B_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_B_AC),                      /* SQR1  */
		ADC_SQR2_SQ12_N(ADC_CH_CUR_B_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_B_AC) |
		ADC_SQR2_SQ10_N(ADC_CH_BRDG_TEMP) |
		ADC_SQR2_SQ9_N(ADC_CH_BRDG_TEMP) |
		ADC_SQR2_SQ8_N(ADC_CH_BRDG_TEMP) |
		ADC_SQR2_SQ7_N(ADC_CH_BRDG_TEMP),                      /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_B) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_B) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_B) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_B) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_B) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_B)                   /* SQR3  */
};

/**
 * ADC conversion group.
 * Mode:        Continuous, 3 samples of 2 channels
 * Channels:    AIN_CUR_C, AIN_TORQUE.
 */
static ADCConversionGroup adcgrpcfg3 = {
		true,
		LENGTH_ADC_SEQ,
		nullptr,
		adcerrorcallback,
		0,                                                    /* CR1   */
		ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2,/* CR2   */
		ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),                     /* SMPR1 */
		ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3),                      /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |
		ADC_SQR1_SQ16_N(ADC_CH_CUR_C_AC) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_C_AC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_C_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_C_AC),                      /* SQR1  */
		ADC_SQR2_SQ12_N(ADC_CH_CUR_C_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_C_AC) |
		ADC_SQR2_SQ10_N(ADC_CH_TORQUE) |
		ADC_SQR2_SQ9_N(ADC_CH_TORQUE) |
		ADC_SQR2_SQ8_N(ADC_CH_TORQUE) |
		ADC_SQR2_SQ7_N(ADC_CH_TORQUE),                      /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_C) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_C) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_C) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_C) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_C) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_C)                       /* SQR3  */
};


/**
 * Initialize ADC hardware
 */
void hardware::analog::Init(void)
{
	/*
	 * Fixed an errata on the STM32F7xx, the DAC clock is required for ADC
	 * triggering.
	 */
	rccEnableDAC1(false);

	/*
	 * Activates the ADC drivers and the VREF input.
	 */
	adcStart(&ADCD1, nullptr);
	adcStart(&ADCD2, nullptr);
	adcStart(&ADCD3, nullptr);

	/*
	 * Starts an ADC continuous conversion
	 */
	adcStartConversion(&ADCD1, &adcgrpcfg1, &samples[0][0][0], ADC_SEQ_BUFFERED);
	adcStartConversion(&ADCD2, &adcgrpcfg2, &samples[1][0][0], ADC_SEQ_BUFFERED);
	adcStartConversion(&ADCD3, &adcgrpcfg3, &samples[2][0][0], ADC_SEQ_BUFFERED);

//	/* Enabling events on both edges of the button line.*/
//	palEnableLineEvent(LINE_CADENCE, PAL_EVENT_MODE_BOTH_EDGES);
//	palSetLineCallback(LINE_CADENCE, palcallback, NULL);
}

/**
 * Get the current values in the last control cycle
 * @param currents references to the current samples
 */
void hardware::analog::current::Value(std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& currents)
{
	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		for (std::uint_fast8_t s = 0; s < hardware::pwm::INJECTION_CYCLES; s++)
		{
			std::int_fast32_t sum = 0;
			std::int_fast32_t cnt = 0;

			for (std::uint_fast8_t a = 0; a < 6; a++)
			{
				sum += samples[i][s][a] + samples[i][s + hardware::pwm::INJECTION_CYCLES][a];
				cnt += 2;
			}
			currents[s].array[i] = ADC2CURRENT * (float)(current_offset[i] - sum) / (float)cnt;
		}
	}
}

/**
 * Get current derivatives in the last control
 * cycles
 * @param derivatives
 */
void hardware::analog::current::Derivative(std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& derivatives)
{
	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		for (std::uint_fast8_t s = 0; s < hardware::pwm::INJECTION_CYCLES; s++)
		{
			std::int_fast32_t sum = 0;
			std::int_fast32_t cnt = 0;

			for (std::uint_fast8_t a = 10; a < 16; a++)
			{
				sum += samples[i][s][a] + samples[i][s + hardware::pwm::INJECTION_CYCLES][a];
				cnt += 2;
			}
			derivatives[s].array[i] = ADC2DERIVATIVE * (float)(derivative_offset[i] - sum) / (float)cnt;
		}
	}
}

/**
 * set dc current offsets
 */
void hardware::analog::current::SetOffset(void)
{
	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		std::int_fast32_t sum = 0;

		for (std::uint_fast8_t s = 0; s < ADC_SEQ_BUFFERED; s++)
		{
			for (std::uint_fast8_t a = 0; a < 6; a++)
			{
				sum += samples[i][s][a];
			}
		}
		current_offset[i] = sum / hardware::pwm::INJECTION_CYCLES;
	}

	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		std::int_fast32_t sum = 0;

		for (std::uint_fast8_t s = 0; s < ADC_SEQ_BUFFERED; s++)
		{
			for (std::uint_fast8_t a = 10; a < 16; a++)
			{
				sum += samples[i][s][a];
			}
		}
		derivative_offset[i] = sum / hardware::pwm::INJECTION_CYCLES;
	}
}

/**
 * Read the DC Bus voltage
 * @return DC Bus voltage in Volts
 */
float hardware::analog::voltage::DCBus(void)
{
	std::int_fast32_t sum = 0;
	std::int_fast32_t cnt = 0;
	float vdc;

	for(std::uint_fast32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		for (std::uint_fast8_t a = 6; a < 10; a++)
		{
			sum += samples[0][i][a];
			cnt ++;
		}
	}

	// Filter inverts the values
	vdc = (float)sum * ADC2VDC / (float)cnt;

	return vdc;
}

/**
 * Get the temperature of the power electronics
 * @return Temperature of the power electronics in °C
 */
float hardware::analog::temperature::Bridge(void)
{
	std::int_fast32_t sum = 0;
	std::int_fast32_t cnt = 0;

	for(std::uint_fast32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		for (std::uint_fast8_t a = 6; a < 10; a++)
		{
			sum += samples[1][i][a];
			cnt ++;
		}
	}

	// Filter inverts the values
	return adc2ntc_temperature(sum/cnt);
}

/**
 * Get the temperature of the motor
 * @return Temperature of the Motor in °C
 */
float hardware::analog::temperature::Motor(void)
{
//	std::int_fast32_t sum = 0;
//	std::int_fast32_t cnt = 0;
//
//	for(std::uint_fast32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
//	{
//		for (std::uint_fast8_t a = 6; a < 10; a++)
//		{
//			sum += samples[0][i][a];
//			cnt ++;
//		}
//	}

	// Filter inverts the values
	return 0.0f;
}


/**
 * get analog input
 * @return analog in put 0 - 1
 */
float hardware::analog::input(void)
{
	std::int_fast32_t sum = 0;
	std::int_fast32_t cnt = 0;

	for(std::uint_fast32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		for (std::uint_fast8_t a = 6; a < 10; a++)
		{
			sum += samples[2][i][a];
			cnt ++;
		}
	}

	return (float)sum / (float)(cnt * 4096);
}

/**
 * \brief    interpolate temperature via a LUT in the range of -10°C to 150°C with an error of 0.393°C
 *
 * \param    adc_value  adc counts
 * \return              temperature in °C
 *
 */
static float adc2ntc_temperature(const uint16_t adc_value)
{
	constexpr float onebylen = 1.0f/NTC_TABLE.TABLE_LEN;
	constexpr uint16_t conv_faktor = 4096/NTC_TABLE.TABLE_LEN;
	int16_t p1,p2;
	/* get the points from table left an right of the actual adc value */
	p1 = NTC_TABLE.table[adc_value/conv_faktor    ];
	p2 = NTC_TABLE.table[adc_value/conv_faktor + 1];

	/* linear interpolation between both points */
	return ((float)p1 - ( (float)(p1-p2) * (float)(adc_value & (NTC_TABLE.TABLE_LEN - 1)) ) * onebylen)*0.01f;
};


/**
 *  ADC errors callback, should never happen.
 * @param adcp
 * @param err
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err)
{

	(void)adcp;
	(void)err;

	osalSysHalt("ADC Error");
}

/**
 * ADC sampling is finished, branch to the control thread
 *
 * @note on a cache MCU this invalidates cache for samples
 */
static void adccallback(ADCDriver *adcp)
{
	(void)adcp;

	/* DMA buffer invalidation because data cache, only invalidating the
     * buffer just filled.
     */
	cacheBufferInvalidate(&samples[0][0][0], sizeof(adcsample_t)*LENGTH_ADC_SEQ*ADC_SEQ_BUFFERED*NUM_OF_ADC);

	// only every second cycle because of current zero delay sample estimation
	if(!hardware::control_thread.isNull())
	{
		/* Wakes up the thread.*/
		osalSysLockFromISR();
		chEvtSignalI(hardware::control_thread.getInner(), (eventmask_t)1);
		osalSysUnlockFromISR();
	}
}

