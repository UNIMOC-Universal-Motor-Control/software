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
#include <cstring>
#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "hardware_interface.hpp"
#include "hal.h"

using namespace hardware::adc;

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err);
static float adc2ntc_temperature(const uint16_t adc_value);


/**
 * Structure from ADC Channel mapping
 *
 * Signal			Pin		ADC1	ADC2	ADC3
 * -----------------------------------------------
 * AIN_ACC 			PC0		CH10	CH10	CH10
 * AIN_DCC  		PC1		CH11	CH11	CH11
 * AIN_CUR_A_DC		PC2		CH12	CH12	CH12
 * AIN_CUR_A_AC		PC3		CH13	CH13	CH13
 * AIN_CUR_B_DC 	PA0		CH0		CH0		CH0
 * AIN_CUR_B_AC 	PA1		CH1		CH1		CH1
 * AIN_CUR_C_DC 	PA2		CH2		CH2		CH2
 * AIN_CUR_C_AC 	PA3		CH3		CH3		CH3
 * AIN_VDC			PA4		CH4		CH4		---
 * AIN_BRDG_TEMP	PA5		CH5		CH5		---
 * AIN_MOT_TEMP		PA6		CH6		CH6		---
 * VREF				---		CH17	---		---
 */
#define ADC_CH_ACC				ADC_CHANNEL_IN10
#define ADC_CH_DCC				ADC_CHANNEL_IN11
#define ADC_CH_CUR_A_DC			ADC_CHANNEL_IN12
#define ADC_CH_CUR_A_AC			ADC_CHANNEL_IN13
#define ADC_CH_CUR_B_DC			ADC_CHANNEL_IN0
#define ADC_CH_CUR_B_AC			ADC_CHANNEL_IN1
#define ADC_CH_CUR_C_DC			ADC_CHANNEL_IN2
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
constexpr uint32_t ADC_SEQ_BUFFERED = 16;

///< # of ADCs
constexpr uint32_t NUM_OF_ADC = 3;


///< absolute maximum current
constexpr float hardware::adc::current::MAX = 1.65f/(20.0f*(0.002f/3.0f));

///< number of current samples per adc sampling cycle
constexpr std::uint_fast8_t CURRENT_SAMPLES = LENGTH_ADC_SEQ - 2;
/*
 * Three 2mR shunts in parallel with a 20V/V gain map to 2048 per 1.65V
 */
constexpr float ADC2CURRENT = hardware::adc::current::MAX /(2048.0f * CURRENT_SAMPLES * ADC_SEQ_BUFFERED);


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
std::uint32_t current_offset[hardware::PHASES];



/**
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 42 channels
 * Channels:    AIN_CUR_A_DC, AIN_VDC.
 */
static ADCConversionGroup adcgrpcfg1 = {
		true,
		LENGTH_ADC_SEQ,
		nullptr,
		adcerrorcallback,
		0,                                                    /* CR1   */
		ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2,/* CR2   */
		ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3),                     /* SMPR1 */
		ADC_SMPR2_SMP_AN4(ADC_SAMPLE_3),                      /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |
		ADC_SQR1_SQ16_N(ADC_CH_VDC) |
		ADC_SQR1_SQ15_N(ADC_CH_VDC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_A_DC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_A_DC),                      /* SQR1  */
		ADC_SQR2_SQ12_N(ADC_CH_CUR_A_DC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_A_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_A_DC) |
		ADC_SQR2_SQ9_N(ADC_CH_CUR_A_DC) |
		ADC_SQR2_SQ8_N(ADC_CH_CUR_A_DC) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_A_DC),                      /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_A_DC)                       /* SQR3  */
};

/**
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 3 channels
 * Channels:    AIN_CUR_B_DC, AIN_BRDG_TEMP, AIN_MOT_TEMP.
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
		ADC_SMPR2_SMP_AN6(ADC_SAMPLE_3),                      /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |
		ADC_SQR1_SQ16_N(ADC_CH_MOT_TEMP) |
		ADC_SQR1_SQ15_N(ADC_CH_BRDG_TEMP) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_B_DC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_B_DC),                      /* SQR1  */
		ADC_SQR2_SQ12_N(ADC_CH_CUR_B_DC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_B_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_B_DC) |
		ADC_SQR2_SQ9_N(ADC_CH_CUR_B_DC) |
		ADC_SQR2_SQ8_N(ADC_CH_CUR_B_DC) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_B_DC),                      /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_B_DC)                       /* SQR3  */
};

/**
 * ADC conversion group.
 * Mode:        Continuous, 4 samples of 3 channels
 * Channels:    AIN_CUR_C_DC, AIN_ACC, AIN_DCC.
 */
static ADCConversionGroup adcgrpcfg3 = {
		true,
		LENGTH_ADC_SEQ,
		nullptr,
		adcerrorcallback,
		0,                                                    /* CR1   */
		ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2,/* CR2   */
		ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3) |
		ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),                     /* SMPR1 */
		ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3),                      /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |
		ADC_SQR1_SQ16_N(ADC_CH_DCC) |
		ADC_SQR1_SQ15_N(ADC_CH_ACC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_C_DC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_C_DC),                      /* SQR1  */
		ADC_SQR2_SQ12_N(ADC_CH_CUR_C_DC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_C_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_C_DC) |
		ADC_SQR2_SQ9_N(ADC_CH_CUR_C_DC) |
		ADC_SQR2_SQ8_N(ADC_CH_CUR_C_DC) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_C_DC),                      /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_C_DC)                       /* SQR3  */
};


/**
 * Initialize ADC hardware
 */
void hardware::adc::Init(void)
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
}

/**
 * prepare samples for evaluation
 *
 * @note on a cache MCU this invalidates cache for samples
 */
void hardware::adc::PrepareSamples(void)
{
	/* DMA buffer invalidation because data cache, only invalidating the
     * buffer just filled.
     */
	cacheBufferInvalidate(&samples,
		sizeof(adcsample_t)*NUM_OF_ADC*LENGTH_ADC_SEQ*ADC_SEQ_BUFFERED);
}

/**
 * Get current means of the current in the last control
 * cycles
 * @param currents references to the current mean samples
 */
void hardware::adc::current::Value(systems::abc& currents)
{
	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		std::uint32_t tmp = 0;
		for (std::uint_fast8_t s = 0; s < ADC_SEQ_BUFFERED; s++)
		{
			for (std::uint_fast8_t j = 0; j < CURRENT_SAMPLES; ++j)
			{
				tmp += samples[i][s][j];
			}
		}
		currents.array[i] = ADC2CURRENT * (float)(tmp - current_offset[i]);
	}
}


/**
 * set dc current offsets
 */
void hardware::adc::current::SetOffset(void)
{
	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		std::uint32_t tmp = 0;
		for (std::uint_fast8_t s = 0; s < ADC_SEQ_BUFFERED; s++)
		{
			for (std::uint_fast8_t j = 0; j < CURRENT_SAMPLES; ++j)
			{
				tmp += samples[i][s][j];
			}
		}
		current_offset[i] = tmp;
	}
}
/**
 * Read the DC Bus voltage
 * @return DC Bus voltage in Volts
 */
float hardware::adc::voltage::DCBus(void)
{
	constexpr float divisor = 4096.0f * 2.0f * ADC_SEQ_BUFFERED;
	constexpr float ADC_2_VDC = (24e3f+1.2e3f)/1.2e3f * 3.3f/divisor;
	uint32_t sum = 0;
	float vdc;


	for(uint32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * VDC is sampled by ADC1 2 times as a non current sample
		 */
		sum += samples[0][i][LENGTH_ADC_SEQ - 1];
		sum += samples[0][i][LENGTH_ADC_SEQ - 2];
	}

	// Filter inverts the values
	vdc = (float)sum * ADC_2_VDC;

	return vdc;
}

/**
 * Get the temperature of the power electronics
 * @return Temperature of the power electronics in °C
 */
float hardware::adc::temperature::Bridge(void)
{
	uint32_t sum = 0;

	for(uint32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * Bridge temperature is sampled by ADC2 first non current sample
		 */
		sum += samples[1][i][LENGTH_ADC_SEQ - 2];
	}

	// Filter inverts the values
	return adc2ntc_temperature(sum/ADC_SEQ_BUFFERED);
}

/**
 * Get the temperature of the motor
 * @return Temperature of the Motor in °C
 */
float hardware::adc::temperature::Motor(void)
{
	uint32_t sum = 0;

	for(uint32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * Motor temperature is sampled by ADC2 second non current sample
		 */
		sum += samples[1][i][LENGTH_ADC_SEQ - 1];
	}

	// Filter inverts the values
	return adc2ntc_temperature(sum/ADC_SEQ_BUFFERED);
}

/**
 * Get the external throttle command value
 * @return Throttle in a range of -1 to 1
 */
float hardware::adc::Throttle(void)
{
	/// FIXME because of 10k input resistor pot only reach 800
	constexpr float ADC_2_THROTTLE = 1.0f/(790.0f * ADC_SEQ_BUFFERED);
	int32_t sum = 0;
	float throttle;

	for(uint32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * ACC is sampled by ADC3 DCC is the first and ACC the second sample
		 */
		sum -= samples[2][i][LENGTH_ADC_SEQ - 1];
		sum += samples[2][i][LENGTH_ADC_SEQ - 2];
	}

	throttle = (float)sum *ADC_2_THROTTLE;

	return throttle;
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
