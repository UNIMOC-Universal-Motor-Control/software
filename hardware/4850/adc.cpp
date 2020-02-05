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
#include <algorithm>
#include "pwm.hpp"
#include "hardware_interface.hpp"
#include "hal.h"


///< intermediate values for linear regression of current samples
/// Sxy = Σxiyi – (Σxi)(Σyi)/n and Sxx = Σxi2 – (Σxi)2/n
typedef struct current_regression_s
{
	uint32_t xy_sum;			/// x*y sum
	uint32_t x_sum;				/// x sum
	uint32_t y_sum;				/// y sum
	uint32_t x2_sum;			/// x^2 sum
	uint32_t n;					/// number of samples
} current_regression_ts;

static inline void adccallback(ADCDriver *adcp);
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err);
static float adc2ntc_temperature(const uint16_t adc_value);
static void regression_addsample(const int16_t xi, const uint16_t yi, current_regression_ts* const reg);
static void regression_calculate(float* mean, float* accent, current_regression_ts* const reg);


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
#define ADC_CH_VREF				ADC_CHANNEL_VREFINT
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

///< Number of NTC resistor LUT values
static constexpr uint16_t NTC_TABLE_LEN = 128;
///< NTC adc value to temperature in 0.01°C LUT
static const int16_t ntc_table[NTC_TABLE_LEN] =
{
	28399, 23653, 18907, 16501, 14928, 13775,
	12871, 12131, 11507, 10968, 10495, 10073,
	9693, 9347, 9030, 8737, 8465, 8211, 7972,
	7748, 7535, 7334, 7142, 6959, 6784, 6616,
	6455, 6299, 6150, 6005, 5865, 5729, 5598,
	5470, 5346, 5224, 5106, 4991, 4878, 4768,
	4660, 4554, 4450, 4349, 4249, 4150, 4054,
	3958, 3865, 3772, 3681, 3591, 3502, 3414,
	3327, 3241, 3156, 3072, 2988, 2905, 2823,
	2742, 2661, 2580, 2500, 2420, 2341, 2262,
	2184, 2105, 2027, 1949, 1872, 1794, 1716,
	1639, 1561, 1484, 1406, 1328, 1250, 1172,
	1093, 1014, 935, 855, 775, 695, 613, 532,
	449, 366, 282, 197, 111, 23, -65, -154, -245,
	-338, -432, -528, -626, -725, -828, -932,
	-1040, -1150, -1264, -1381, -1503, -1629,
	-1760, -1897, -2041, -2192, -2352, -2522,
	-2704, -2901, -3115, -3351, -3616, -3920,
	-4278, -4720, -5311, -6245
};


/* Note, the buffer is aligned to a 32 bytes boundary because limitations
   imposed by the data cache. Note, this is GNU specific, it must be
   handled differently for other compilers.
   Only required if the ADC buffer is placed in a cache-able area.*/
///< dma accessed buffer for the adcs, probably cached
__attribute__((aligned (32))) static adcsample_t samples[NUM_OF_ADC][ADC_SEQ_BUFFERED][LENGTH_ADC_SEQ];
///< cycle index for cached working buffer
static uint32_t samples_index = 0;

///< index of the non current measurements in adc samples
static uint8_t non_cur_index[2] = {0, LENGTH_ADC_SEQ - 1};

///< reference to thread to be woken up in the hardware control cycle.
chibios_rt::ThreadReference hardware::control_thread = nullptr;

///< current samples gains
static float current_gain[hardware::PHASES];

///< current samples offsets
static int16_t current_offset[hardware::PHASES];


/**
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 4 channels
 * Channels:    AIN_CUR_A_DC, AIN_CUR_A_AC, AIN_VDC, VREF.
 */
static ADCConversionGroup adcgrpcfg1 = {
		true,
		LENGTH_ADC_SEQ,
		adccallback,
		adcerrorcallback,
		0,                                                    /* CR1   */
		ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2,/* CR2   */
		ADC_SMPR1_SMP_AN12(ADC_SAMPLE_15) |
		ADC_SMPR1_SMP_AN13(ADC_SAMPLE_15) |
		ADC_SMPR1_SMP_VREF(ADC_SAMPLE_15),                    /* SMPR1 */
		ADC_SMPR2_SMP_AN4(ADC_SAMPLE_15),                     /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |                     /* SQR1  */
		ADC_SQR1_SQ16_N(ADC_CH_VDC) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_A_DC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_A_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_A_DC),
		ADC_SQR2_SQ12_N(ADC_CH_CUR_A_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_A_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_A_AC) |
		ADC_SQR2_SQ9_N(ADC_CH_CUR_A_DC) |
		ADC_SQR2_SQ8_N(ADC_CH_CUR_A_AC) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_A_DC) ,                     /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_A_AC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_A_AC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_A_AC) |
		ADC_SQR3_SQ1_N(ADC_CH_VDC)       	                /* SQR3  */
};

/**
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 4 channels
 * Channels:    AIN_CUR_B_DC, AIN_CUR_B_AC, AIN_BRDG_TEMP, AIN_MOT_TEMP.
 */
static ADCConversionGroup adcgrpcfg2 = {
		true,
		LENGTH_ADC_SEQ,
		NULL,
		adcerrorcallback,
		0,                                                    /* CR1   */
		ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2,/* CR2   */
		0,                                                    /* SMPR1 */
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_15) |
		ADC_SMPR2_SMP_AN1(ADC_SAMPLE_15) |
		ADC_SMPR2_SMP_AN5(ADC_SAMPLE_15) |
		ADC_SMPR2_SMP_AN6(ADC_SAMPLE_15),                     /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |                     /* SQR1  */
		ADC_SQR1_SQ16_N(ADC_CH_MOT_TEMP) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_B_DC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_B_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_B_DC),
		ADC_SQR2_SQ12_N(ADC_CH_CUR_B_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_B_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_B_AC) |
		ADC_SQR2_SQ9_N(ADC_CH_CUR_B_DC) |
		ADC_SQR2_SQ8_N(ADC_CH_CUR_B_AC) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_B_DC) ,                     /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_B_AC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_B_AC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_B_AC) |
		ADC_SQR3_SQ1_N(ADC_CH_BRDG_TEMP)                       /* SQR3  */
};

/**
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 4 channels
 * Channels:    AIN_CUR_C_DC, AIN_CUR_C_AC, AIN_ACC, AIN_DCC.
 */
static ADCConversionGroup adcgrpcfg3 = {
		true,
		LENGTH_ADC_SEQ,
		nullptr,
		adcerrorcallback,
		0,                                                    /* CR1   */
		ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2,/* CR2   */
		ADC_SMPR1_SMP_AN10(ADC_SAMPLE_15) |
		ADC_SMPR1_SMP_AN11(ADC_SAMPLE_15),                    /* SMPR1 */
		ADC_SMPR2_SMP_AN2(ADC_SAMPLE_15) |
		ADC_SMPR2_SMP_AN3(ADC_SAMPLE_15),                     /* SMPR2 */
		0,                                                    /* HTR */
		0,                                                    /* LTR */
		ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) |                     /* SQR1  */
		ADC_SQR1_SQ16_N(ADC_CH_ACC) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_C_DC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_C_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_C_DC),
		ADC_SQR2_SQ12_N(ADC_CH_CUR_C_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_C_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_C_AC) |
		ADC_SQR2_SQ9_N(ADC_CH_CUR_C_DC) |
		ADC_SQR2_SQ8_N(ADC_CH_CUR_C_AC) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_C_DC) ,                     /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_C_AC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_C_AC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_C_AC) |
		ADC_SQR3_SQ1_N(ADC_CH_DCC)                       /* SQR3  */
};


/**
 * Initialize ADC hardware
 */
void hardware::adc::Init(void)
{
	for(uint8_t i = 0; i < PHASES; i++)
	{
		current_gain[i] = 1.0f;
		current_offset[i] = 2048;
	}

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
 * Get current means, acents and decents of the current in the last control
 * cycles
 * @param currents pointer to a currents structure, will be written
 */
void hardware::adc::GetCurrents(current_values_ts* const currents)
{
	/*
	 * Three 2mR shunts in parallel with a 20V/V gain map to 2048 per 1.65V
	 */
	const float ADC2CURRENT = 1.65f/(20.0f*(0.002f/3.0f))/2048.0f;
	current_regression_ts regres_dc;
	current_regression_ts regres_ac;
	uint32_t duty_min = *std::min_element(&pwm::duty_counts[0], &pwm::duty_counts[2]);
	uint32_t duty_max = *std::max_element(&pwm::duty_counts[0], &pwm::duty_counts[2]);
	/*
	 * When 16 samples are evenly distributed over the hole period the min and max duty
	 * gives the times of the edges in between which the active state is located
	 * Because the ADC trigger is at 50% the samples are located from 50% to 50%
	 * of the next cycle. So start and end samples need to be shifted.
	 */
	int32_t end_sample = LENGTH_ADC_SEQ - (LENGTH_ADC_SEQ/2 - ((duty_min * LENGTH_ADC_SEQ)) / pwm::PERIOD) - 1;
	int32_t start_sample = ((duty_max *LENGTH_ADC_SEQ) / pwm::PERIOD) - LENGTH_ADC_SEQ/2 + 1;

	memset(&regres_dc, 0, sizeof(current_regression_ts));
	memset(&regres_ac, 0, sizeof(current_regression_ts));

	for(uint8_t i = 0; i < PHASES; i++)
	{
		uint8_t s1 = samples_index;

		// start with the rest of the last full decent
		// @note we evaluate the last FULL decent so current sample
		// is one cycle delayed
		for(uint8_t k = start_sample; k < end_sample; k++)
		{
			if(k%2)	// odd samples are ac
			{
				regression_addsample(k, samples[s1][i][k], &regres_ac);
			}
			else // even samples are dc
			{
				regression_addsample(k, samples[s1][i][k], &regres_dc);
			}
		}


		float mean, accent;
		regression_calculate(&mean, &accent, &regres_dc);
		currents->current[i] = (mean - current_offset[i]) * ADC2CURRENT * current_gain[i];
		currents->current_decent[i] = accent*ADC2CURRENT*2e-6f; // per 2 us

		regression_calculate(&mean, &accent, &regres_ac);
		currents->current_acent[i] = accent*ADC2CURRENT*2e-7f; // per 2 us
	}
}

/**
 * Read the DC Bus voltage
 * @return DC Bus voltage in Volts
 */
float hardware::adc::GetDCBusVoltage(void)
{
	constexpr float ADC_2_VDC = 560.0f/(10e3f+560.0f) * 3.3f/(4096.0f * 2.0f * ADC_SEQ_BUFFERED);
	uint32_t sum = 0;
	float vdc;


	for(uint32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * VDC is sampled by ADC1 2 times as a non current sample
		 */
		sum += samples[i][0][non_cur_index[0]];
		sum += samples[i][0][non_cur_index[1]];
	}

	vdc = (float)sum * ADC_2_VDC;

	return vdc;
}

/**
 * Get the temperature of the power electronics
 * @return Temperature of the power electronics in °C
 */
float hardware::adc::GetBridgeTemp(void)
{
	uint16_t sum = 0;

	for(uint32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * Bridge temperature is sampled by ADC2 first non current sample
		 */
		sum += samples[i][0][non_cur_index[0]];
	}

	return adc2ntc_temperature(sum);
}

/**
 * Get the temperature of the motor
 * @return Temperature of the Motor in °C
 */
float hardware::adc::GetMotorTemp(void)
{
	uint16_t sum = 0;

	for(uint32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * Motor temperature is sampled by ADC2 second non current sample
		 */
		sum += samples[i][0][non_cur_index[1]];
	}

	return adc2ntc_temperature(sum);
}

/**
 * Get the external throttle command value
 * @return Throttle in a range of -1 to 1
 */
float hardware::adc::GetThrottle(void)
{
	constexpr float ADC_2_THROTTLE = 1.0f/(4096.0f * ADC_SEQ_BUFFERED);
	int32_t sum = 0;
	float throttle;

	for(uint32_t i = 0; i < ADC_SEQ_BUFFERED; i++)
	{
		/*
		 * ACC is sampled by ADC3 DCC is the first and ACC the second sample
		 */
		sum -= samples[i][0][non_cur_index[0]];
		sum += samples[i][0][non_cur_index[1]];
	}

	throttle = (float)sum *ADC_2_THROTTLE;

	return throttle;
}


/**
 * Calibrate the current measurements for offset an gain.
 *
 * @note A motor must be connected and pwm must be active.
 * @note The gain will only be equalized between the phases.
 * @note The function blocks the thread for about 2secounds.
 *
 * @return false if pwm not active
 */
extern bool hardware::adc::Calibrate(void)
{
	bool result = false;
	const float DUTY_STEP = 0.01f;
	const uint16_t ADC_TARGET = 1000;
	int32_t offset_sum[PHASES];
	float gain[PHASES][PHASES];
	int32_t gain_sum[PHASES][PHASES];
	float test_duty;
	float dutys[PHASES] = {0.0f, 0.0f, 0.0f};

	pwm::DisableOutputs();

	// let the current amps calm down
	osalThreadSleepMilliseconds(100);

	// Calculate Current Offsets
	for(uint8_t i = 0; i < PHASES; i++)
	{
		offset_sum[i] = 0;
	}

	for(uint8_t p = 0; p < PHASES; p++)
	{
		for(uint8_t i = 0; i < ADC_SEQ_BUFFERED; i++)
		{
			for(uint8_t k = 0; k < LENGTH_ADC_SEQ; k+=2)
			{
				// skip non current samples
				if(k == non_cur_index[0] || k == non_cur_index[1]) continue;

				offset_sum[p] += samples[p][i][k];
			}
		}
	}

	for(uint8_t i = 0; i < PHASES; i++)
	{
		current_offset[i] = offset_sum[i] / (ADC_SEQ_BUFFERED * (LENGTH_ADC_SEQ / 2 - 1));
	}

	// caution PWMs are active beyond this line
	pwm::SetDutys(dutys);
	pwm::EnableOutputs();

	osalThreadSleepMilliseconds(1);

	// test what duty gives a current of 50% Fullscale
	while(pwm::OutputActive() && std::abs(samples[0][samples_index][0] - current_offset[0]) < ADC_TARGET)
	{
		dutys[0] += DUTY_STEP;
		dutys[1] -= 0.5f * DUTY_STEP;
		dutys[2] -= 0.5f * DUTY_STEP;
		pwm::SetDutys(dutys);

		osalThreadSleepMilliseconds(1);
	}
	test_duty = dutys[0];

	uint8_t i;
	for(i = 0; i < PHASES && pwm::OutputActive(); i++)
	{
		dutys[i] = test_duty;
		dutys[(i+1)%PHASES] = -0.5f * test_duty;
		dutys[(i+2)%PHASES] = -0.5f * test_duty;
		pwm::SetDutys(dutys);
		osalThreadSleepMilliseconds(10);

		for(uint8_t s = 0; s < ADC_SEQ_BUFFERED; s++)
		{
			for (uint8_t p = 0; p < PHASES; ++p)
			{
				// take only the mid samples in to account
				// The edges are supposed to be below half duty
				for (uint8_t k = 5; k < 11; ++k)
				{
					gain_sum[i][p] += samples[p][s][k];
				}
			}
		}
	}

	pwm::DisableOutputs();
	dutys[0] = dutys[1] = dutys[2]= 0.0f;
	pwm::SetDutys(dutys);

	if(i >= PHASES)
	{
		for(uint8_t i = 0; i < PHASES; i++)
		{
			for(uint8_t k = 0; k < PHASES; k++)
			{
				// transform the other currents back
				if(i == k) gain[i][k] = gain_sum[i][k] / (float)(ADC_SEQ_BUFFERED * 6) - (float)current_offset[k];
				// normally half the current on the back flowing phases
				else gain[i][k] = -((float)gain_sum[i][k] / (float)(ADC_SEQ_BUFFERED * 3)) + (float)current_offset[k];
			}
		}

		float mean_gain = 0.0f;
		for(uint8_t i = 0; i < PHASES; i++)
		{
			for(uint8_t k = 0; k < PHASES; k++)
			{
				mean_gain += gain[i][k];
			}
		}
		mean_gain /= PHASES * PHASES;


		for(uint8_t i = 0; i < PHASES; i++)
		{
			float phase_gain = 0.0f;
			for(uint8_t k = 0; k < PHASES; k++)
			{
				phase_gain += gain[i][k];
			}
			phase_gain /= PHASES;

			current_gain[i] = mean_gain / phase_gain;
		}

		result = true;
	}

	return result;
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
	const float oneby256 = 1.0f/256.0f;
	int16_t p1,p2;
	/* get the points from table left an right of the actual adc value */
	p1 = ntc_table[ (adc_value >> 8)  ];
	p2 = ntc_table[ (adc_value >> 8)+1];

	/* linear interpolation between both points */
	return ((float)p1 - ( (float)(p1-p2) * (float)(adc_value & 0x00FF) ) * oneby256)*0.01f;
};


/**
 * adc processing callback
 *
 * this call back will trigger the control task
 * @param adcp pointer to the adc driver instance
 */
static inline void adccallback(ADCDriver *adcp)
{
	(void)adcp;
	palSetLine(LINE_HALL_B);

	/* DMA buffer invalidation because data cache, only invalidating the
     * buffer just filled.
     */
	for (uint8_t i = 0; i < hardware::PHASES; ++i)
	{
		cacheBufferInvalidate(&samples[i][samples_index][0],
				sizeof(adcsample_t)*LENGTH_ADC_SEQ);
	}


	if(samples_index < ADC_SEQ_BUFFERED) samples_index++;
	else samples_index = 0;


	if(!hardware::control_thread.isNull())
	{
		/* Wakes up the thread.*/
		osalSysLockFromISR();
		chEvtSignalI(hardware::control_thread.getInner(), (eventmask_t)1);
		osalSysUnlockFromISR();
	}

	palClearLine(LINE_HALL_B);
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
 * add sample to the regression window
 * @param xi	x in us
 * @param yi	y in adc counts
 * @param reg	pointer to regression structure
 */
static void regression_addsample(const int16_t xi, const uint16_t yi, current_regression_ts* const reg)
{
	reg->x_sum += xi;
	reg->x2_sum += xi*xi;
	reg->xy_sum += xi*yi;
	reg->y_sum += yi;
	reg->n++;
}

/**
 * calculate the linear regression for the samples added to the window
 * @param mean		pointer to the mean value output
 * @param accent	pointer to the accent value output
 * @param reg		pointer to the regression structure
 */
static void regression_calculate(float* mean, float* accent, current_regression_ts* const reg)
{
	*mean = reg->y_sum / reg->n;
	float sxy = reg->xy_sum - ((reg->x_sum * reg->y_sum)/reg->n);
	float sxx = reg->x2_sum - ((reg->x_sum*reg->x_sum)/reg->n);
	*accent = sxy / sxx;
}
