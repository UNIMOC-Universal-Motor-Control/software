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
static void adccallback(ADCDriver *adcp);
static void palcallback(void *arg);
static float adc2ntc_temperature(const uint16_t adc_value);


/**
 * Structure from ADC Channel mapping
 *
 * Signal			Pin		ADC1	ADC2	ADC3
 * -----------------------------------------------
 * AIN_TORQUE		PC1		CH11	CH11	CH11
 * AIN_CUR_A		PC2		CH12	CH12	CH12
 * AIN_CUR_B	 	PA0		CH0		CH0		CH0
 * AIN_CUR_C	 	PA2		CH2		CH2		CH2
 * AIN_VDC			PA4		CH4		CH4		---
 * AIN_BRDG_TEMP	PA5		CH5		CH5		---
 * AIN_MOT_TEMP		PA6		CH6		CH6		---
 * VREF				---		CH17	---		---
 */
#define ADC_CH_TORQUE			ADC_CHANNEL_IN11
#define ADC_CH_CUR_A			ADC_CHANNEL_IN12
#define ADC_CH_CUR_B			ADC_CHANNEL_IN0
#define ADC_CH_CUR_C			ADC_CHANNEL_IN2
#define ADC_CH_VDC				ADC_CHANNEL_IN4
#define ADC_CH_BRDG_TEMP		ADC_CHANNEL_IN5
#define ADC_CH_MOT_TEMP			ADC_CHANNEL_IN6
#define ADC_CH_REF				ADC_CHANNEL_IN17


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
constexpr uint32_t LENGTH_ADC_SEQ = 3;

///< ADC sequences in buffer.
/// Caution: samples are 16bit but the hole sequence must be 32 bit aligned!
///          so even length of sequence is best choice.
constexpr uint32_t ADC_SEQ_BUFFERED = 32;

///< # of ADCs
constexpr uint32_t NUM_OF_ADC = 3;

///< absolute maximum current
constexpr float hardware::adc::current::MAX = 1.65f/(20.0f*(0.002f/3.0f));

///< Three 2mR shunts in parallel with a 20V/V gain map to 2048 per 1.65V
constexpr float ADC2CURRENT = hardware::adc::current::MAX /(2048.0f);

///< Voltage divider
constexpr float ADC2VDC = (24e3f+1.2e3f)/1.2e3f * 3.3f/(4096.0f * 2.0f);


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

///< samples index in the adc buffer.
std::uint32_t sample_index = ADC_SEQ_BUFFERED - 1;
std::uint32_t prev_index = ADC_SEQ_BUFFERED - 2;

///< cadence signal counter
std::uint32_t cadence_counter = 0;

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
		ADC_SMPR1_SMP_AN12(ADC_SAMPLE_15),                    /* SMPR1 */
		ADC_SMPR2_SMP_AN4(ADC_SAMPLE_15),                     /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ),                      /* SQR1  */
		0,                                                    /* SQR2  */
		ADC_SQR3_SQ3_N(ADC_CH_VDC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_A) |
		ADC_SQR3_SQ1_N(ADC_CH_VDC)                            /* SQR3  */
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
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_15) |
		ADC_SMPR2_SMP_AN5(ADC_SAMPLE_15) |
		ADC_SMPR2_SMP_AN6(ADC_SAMPLE_15),                     /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ),                      /* SQR1  */
		0,                                                    /* SQR2  */
		ADC_SQR3_SQ3_N(ADC_CH_BRDG_TEMP) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_B) |
		ADC_SQR3_SQ1_N(ADC_CH_MOT_TEMP)                       /* SQR3  */
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
		ADC_SMPR1_SMP_AN11(ADC_SAMPLE_15),                     /* SMPR1 */
		ADC_SMPR2_SMP_AN2(ADC_SAMPLE_15),                      /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ),                      /* SQR1  */
		0,                                                    /* SQR2  */
		ADC_SQR3_SQ3_N(ADC_CH_TORQUE) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_C) |
		ADC_SQR3_SQ1_N(ADC_CH_TORQUE)                       /* SQR3  */
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

	/* Enabling events on both edges of the button line.*/
	palEnableLineEvent(LINE_CADENCE, PAL_EVENT_MODE_BOTH_EDGES);
	palSetLineCallback(LINE_CADENCE, palcallback, NULL);
}

/**
 * Get the current values in the last control cycle
 * @param currents references to the current samples
 */
void hardware::adc::current::Value(systems::abc& currents)
{
	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		std::int32_t tmp = 2.0f * samples[i][sample_index][1] - samples[i][prev_index][1];

		currents.array[i] = ADC2CURRENT * (float)(current_offset[i] - tmp);
	}
}

/**
 * set dc current offsets
 */
void hardware::adc::current::SetOffset(void)
{
	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		std::int32_t sum = 0;
		for (std::uint_fast32_t s = 0; s < ADC_SEQ_BUFFERED; s++)
		{
			sum += samples[i][s][1];
		}
		sum = (float)sum / (float)ADC_SEQ_BUFFERED;
		current_offset[i] = sum;
	}
}

/**
 * Read the DC Bus voltage
 * @return DC Bus voltage in Volts
 */
float hardware::adc::voltage::DCBus(void)
{
	uint32_t sum = 0;
	float vdc;

	/*
	 * VDC is sampled by ADC1 2 times as a non current sample
	 */
	sum += samples[0][sample_index][0];
	sum += samples[0][sample_index][2];
	sum *= 2.0f;
	sum -= samples[0][prev_index][0];
	sum -= samples[0][prev_index][2];

	// Filter inverts the values
	vdc = (float)sum * ADC2VDC;

	return vdc;
}

/**
 * Get the temperature of the power electronics
 * @return Temperature of the power electronics in °C
 */
float hardware::adc::temperature::Bridge(void)
{
	uint32_t sum = 0;

	for(std::uint_fast32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * Bridge temperature is sampled by ADC2 first non current sample
		 */
		sum += samples[1][i][2];
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

	for(std::uint_fast32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * Motor temperature is sampled by ADC2 second non current sample
		 */
		sum += samples[1][i][0];
	}

	// Filter inverts the values
	return adc2ntc_temperature(sum/ADC_SEQ_BUFFERED);
}

/**
 * Torque on the crank arm
 *
 * @param offset in Volts
 * @param gain in Nm/V
 * @return Torque in Nm
 */
float hardware::crank::Torque(const float offset, const float gain)
{
	constexpr float ADC2VOLTAGE = 3.3f/(4096.0f * 2.0f * ADC_SEQ_BUFFERED);
	uint32_t sum = 0;
	float torque;

	for(std::uint_fast32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		sum += samples[2][i][0];
		sum += samples[2][i][2];
	}
	torque = (((float)sum * ADC2VOLTAGE) - offset) * gain;

	return torque;
}

/**
 * Angle of the crank arm
 *
 * @param edge_max Number of edges per revolution
 * @return Angle in rads, range 0 - 2*PI
 */
float hardware::crank::Angle(uint32_t edge_max)
{
	static float angle = 0.0f;

	if(cadence_counter)
	{
		angle += (float)cadence_counter/(float)edge_max * math::_2PI;
		cadence_counter = 0;
	}

	if(angle > math::_2PI) angle -= math::_2PI;

	return angle;
}

/**
 * get the angle which is represented by the hall sensors
 * @param[out] sincos angle of the halls represented as sin/cos values
 * @return true on hall signal error
 */
bool hardware::adc::hall::Angle(systems::sin_cos& sincos)
{
	const float half_sqrt3 = 0.5f * std::sqrt(3.0f);
	uint32_t halls = palReadGroup(GPIOC, 0x0007, 13);
	bool err = false;

	switch(halls)
	{
	case 5:
		// 30°
		sincos.cos = 0.5f;
		sincos.sin = half_sqrt3;
		break;
	case 1:
		// 90°
		sincos.cos = 1.0f;
		sincos.sin = 0.0f;
		break;
	case 3:
		// 150°
		sincos.cos = 0.5f;
		sincos.sin = -half_sqrt3;
		break;
	case 2:
		// 210°
		sincos.cos = -0.5f;
		sincos.sin = -half_sqrt3;
		break;
	case 6:
		// 270°
		sincos.cos = -1.0f;
		sincos.sin = 0.0f;
		break;
	case 4:
		// 330°
		sincos.cos = -0.5f;
		sincos.sin = half_sqrt3;
		break;
	default:
		// some thing is wrong
		err = true;
		break;

	}

	return err;
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
 * callback from pal driver on each edge of the PAS Cadence signal
 * @param arg
 */
static void palcallback(void *arg)
{
	(void)arg;
	cadence_counter++;
}

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

	sample_index++;
	prev_index++;
	if(sample_index >= ADC_SEQ_BUFFERED) sample_index = 0;
	if(prev_index >= ADC_SEQ_BUFFERED) prev_index = 0;

	/* DMA buffer invalidation because data cache, only invalidating the
     * buffer just filled.
     */
	for(std::uint_fast8_t i = 0; i < hardware::PHASES; i++)
	{
		cacheBufferInvalidate(&samples[i][prev_index][0], sizeof(adcsample_t)*LENGTH_ADC_SEQ);
		cacheBufferInvalidate(&samples[i][sample_index][0], sizeof(adcsample_t)*LENGTH_ADC_SEQ);
	}

	// only every second cycle because of current zero delay sample estimation
	if(!hardware::control_thread.isNull())
	{
		/* Wakes up the thread.*/
		osalSysLockFromISR();
		chEvtSignalI(hardware::control_thread.getInner(), (eventmask_t)1);
		osalSysUnlockFromISR();
	}
}

