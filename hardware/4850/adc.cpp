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
#include <cstring>
#include <cstdint>
#include <cmath>
#include "pwm.hpp"
#include "hardware_interface.hpp"
#include "hal.h"

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


/* Note, the buffer is aligned to a 32 bytes boundary because limitations
   imposed by the data cache. Note, this is GNU specific, it must be
   handled differently for other compilers.
   Only required if the ADC buffer is placed in a cache-able area.*/
///< dma accessed buffer for the adcs, probably cached
__attribute__((aligned (32))) static int16_t dma_samples[NUM_OF_ADC][LENGTH_ADC_SEQ];
///< cached working buffer for calculations
__attribute__((aligned (32))) static int16_t samples[ADC_SEQ_BUFFERED][NUM_OF_ADC][LENGTH_ADC_SEQ];
///< cycle index for cached working buffer
static uint32_t samples_index = 0;

///< index of the non current measurements in adc samples
static uint8_t non_cur_index[2] = {7, 8};

///< reference to thread to be woken up in the hardware control cycle.
thread_reference_t* unimoc::hardware::control_thread = nullptr;

///< current samples gains
static float current_gain[PHASES];

///< current samples offsets
static int16_t current_offset[PHASES];


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
     half buffer just filled.
     Only required if the ADC buffer is placed in a cache-able area.*/
	cacheBufferInvalidate(dma_samples,
			sizeof(dma_samples));

	samples_index++;
	if(samples_index > ADC_SEQ_BUFFERED) samples_index = 0;

	std::memcpy(&samples[samples_index][0][0], &dma_samples[0][0], sizeof(dma_samples));


	if(unimoc::hardware::control_thread != nullptr)
	{
		/* Wakes up the thread.*/
		chSysLockFromISR();
		chThdResumeI(unimoc::hardware::control_thread, (msg_t)samples_index);  /* Resuming the thread with message.*/
		chSysUnlockFromISR();
	}

	palClearLine(LINE_HALL_B);
}

/**
 *  ADC errors callback, should never happen.
 * @param adcp
 * @param err
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

	(void)adcp;
	(void)err;

	osalSysHalt("ADC Error");
}


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
		ADC_SQR1_SQ16_N(ADC_CH_CUR_A_AC) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_A_DC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_A_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_A_DC),
		ADC_SQR2_SQ12_N(ADC_CH_CUR_A_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_A_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_A_AC) |
		ADC_SQR2_SQ9_N(ADC_CH_VDC) |
		ADC_SQR2_SQ8_N(ADC_CH_VDC) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_A_DC) ,                     /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_A_AC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_A_AC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_A_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_A_AC) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_A_DC)                       /* SQR3  */
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
		ADC_SQR1_SQ16_N(ADC_CH_CUR_B_AC) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_B_DC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_B_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_B_DC),
		ADC_SQR2_SQ12_N(ADC_CH_CUR_B_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_B_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_B_AC) |
		ADC_SQR2_SQ9_N(ADC_CH_MOT_TEMP) |
		ADC_SQR2_SQ8_N(ADC_CH_BRDG_TEMP) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_B_DC) ,                     /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_B_AC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_B_AC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_B_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_B_AC) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_B_DC)                       /* SQR3  */
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
		ADC_SQR1_SQ16_N(ADC_CH_CUR_C_AC) |
		ADC_SQR1_SQ15_N(ADC_CH_CUR_C_DC) |
		ADC_SQR1_SQ14_N(ADC_CH_CUR_C_AC) |
		ADC_SQR1_SQ13_N(ADC_CH_CUR_C_DC),
		ADC_SQR2_SQ12_N(ADC_CH_CUR_C_AC) |
		ADC_SQR2_SQ11_N(ADC_CH_CUR_C_DC) |
		ADC_SQR2_SQ10_N(ADC_CH_CUR_C_AC) |
		ADC_SQR2_SQ9_N(ADC_CH_ACC) |
		ADC_SQR2_SQ8_N(ADC_CH_DCC) |
		ADC_SQR2_SQ7_N(ADC_CH_CUR_C_DC) ,                     /* SQR2  */
		ADC_SQR3_SQ6_N(ADC_CH_CUR_C_AC) |
		ADC_SQR3_SQ5_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ4_N(ADC_CH_CUR_C_AC) |
		ADC_SQR3_SQ3_N(ADC_CH_CUR_C_DC) |
		ADC_SQR3_SQ2_N(ADC_CH_CUR_C_AC) |
		ADC_SQR3_SQ1_N(ADC_CH_CUR_C_DC)                       /* SQR3  */
};


/**
 * Initialize ADC hardware
 */
void unimoc::hardware::adc::Init(void)
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
	adcSTM32EnableTSVREFE();

	/*
	 * Starts an ADC continuous conversion
	 */
	adcStartConversion(&ADCD1, &adcgrpcfg1, &dma_samples[0][0], 1);
	adcStartConversion(&ADCD2, &adcgrpcfg2, &dma_samples[1][0], 1);
	adcStartConversion(&ADCD3, &adcgrpcfg3, &dma_samples[2][0], 1);
}


/**
 * Get current means, acents and decents of the current in the last control
 * cycles
 * @param currents pointer to a currents structure, will be written
 */
void unimoc::hardware::adc::GetCurrents(current_values_ts* const currents)
{
	(void)currents;
}

/**
 * Read the DC Bus voltage
 * @return DC Bus voltage in Volts
 */
float unimoc::hardware::adc::GetDCBusVoltage(void)
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



/**
 * \brief    Konvertiert das ADC Ergebnis in einen Temperaturwert.
 *
 *           Mit p1 und p2 wird der Stützpunkt direkt vor und nach dem
 *           ADC Wert ermittelt. Zwischen beiden Stützpunkten wird linear
 *           interpoliert. Der Code ist sehr klein und schnell.
 *           Es wird lediglich eine Ganzzahl-Multiplikation verwendet.
 *           Die Division kann vom Compiler durch eine Schiebeoperation.
 *           ersetzt werden.
 *
 *           Im Temperaturbereich von -10°C bis 150°C beträgt der Fehler
 *           durch die Verwendung einer Tabelle 0.393°C
 *
 * \param    adc_value  Das gewandelte ADC Ergebnis
 * \return              Die Temperatur in 0.01 °C
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
 * Get the temperature of the power electronics
 * @return Temperature of the power electronics in °C
 */
float unimoc::hardware::adc::GetBridgeTemp(void)
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
float unimoc::hardware::adc::GetMotorTemp(void)
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
float unimoc::hardware::adc::GetThrottle(void)
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
extern bool unimoc::hardware::adc::Calibrate(void)
{
	bool result = false;
	const float DUTY_STEP = 0.01f;
	const uint16_t ADC_TARGET = 1000;
	uint8_t phase = 0;
	uint32_t offset_sum[PHASES];
	float gain[PHASES][PHASES];
	uint32_t gain_sum[PHASES][PHASES];
	float test_duty;
	float dutys[unimoc::hardware::PHASES] = {0.0f, 0.0f, 0.0f};

	unimoc::hardware::pwm::DisableOutputs();

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

				offset_sum[p] += samples[i][p][k];
			}
		}
	}

	for(uint8_t i = 0; i < PHASES; i++)
	{
		current_offset[i] = offset_sum[i] / (ADC_SEQ_BUFFERED * (LENGTH_ADC_SEQ / 2 - 1));
	}

	// caution PWMs are active beyond this line
	unimoc::hardware::pwm::SetDutys(dutys);
	unimoc::hardware::pwm::EnableOutputs();

	osalThreadSleepMilliseconds(1);

	// test what duty gives a current of 50% Fullscale
	while(unimoc::hardware::pwm::OutputActive() && std::abs(samples[0][0][0] - current_offset[0]) < ADC_TARGET)
	{
		dutys[0] += DUTY_STEP;
		dutys[1] -= 0.5f * DUTY_STEP;
		dutys[2] -= 0.5f * DUTY_STEP;
		unimoc::hardware::pwm::SetDutys(dutys);

		osalThreadSleepMilliseconds(1);
	}
	test_duty = dutys[0];

	for(uint8_t i = 0; i < PHASES && unimoc::hardware::pwm::OutputActive(); i++)
	{
		dutys[i] = test_duty;
		dutys[(i+1)%PHASES] = -0.5f * test_duty;
		dutys[(i+2)%PHASES] = -0.5f * test_duty;
		unimoc::hardware::pwm::SetDutys(dutys);
		osalThreadSleepMilliseconds(10);

		for(uint8_t s = 0; s < ADC_SEQ_BUFFERED; s++)
		{
			// take only the mid samples in to account
			gain_sum[i] += samples[s][i][0] + samples[s][i][LENGTH_ADC_SEQ - 2];
		}
	}

	unimoc::hardware::pwm::DisableOutputs();
	dutys[0] = dutys[1] = dutys[2]= 0.0f;
	unimoc::hardware::pwm::SetDutys(dutys);

	if(i >= PHASES)
	{
		for(uint8_t i = 0; i < PHASES; i++)
		{
			gain[i] = (float)gain_sum[i] / (float)(ADC_SEQ_BUFFERED * 2) - (float)current_offset[i];
		}

		float mean_gain = (gain[0] + gain[1] + gain[2])/3;

		for(uint8_t i = 0; i < PHASES; i++)
		{
			current_gain[i] = gain[i] / mean_gain;
		}

		result = true;
	}

	return result;
}
