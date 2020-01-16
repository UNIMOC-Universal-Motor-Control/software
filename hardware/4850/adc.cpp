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
#include <cstdint>
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
constexpr uint32_t ADC_SEQ_BUFFERED = 8;

///< # of ADCs
constexpr uint32_t NUM_OF_ADC = 3;


/* Note, the buffer is aligned to a 32 bytes boundary because limitations
   imposed by the data cache. Note, this is GNU specific, it must be
   handled differently for other compilers.
   Only required if the ADC buffer is placed in a cache-able area.*/
__attribute__((aligned (32))) static adcsample_t samples[NUM_OF_ADC][LENGTH_ADC_SEQ * ADC_SEQ_BUFFERED];

static void setup_sequence(ADCDriver *adcp, uint8_t edge_index, const uint8_t cur_dc, const uint8_t cur_ac, const uint8_t aux1, const uint8_t aux2);


volatile uint32_t adc_clk = 	STM32_ADCCLK;
/*
 * ADC streaming callback.
 */
static void adccallback(ADCDriver *adcp)
{
	(void)adcp;
	palSetLine(LINE_HALL_B);

	/* DMA buffer invalidation because data cache, only invalidating the
     half buffer just filled.
     Only required if the ADC buffer is placed in a cache-able area.*/
	cacheBufferInvalidate(samples,
			sizeof(samples));

	/* Updating counters.*/
	if (adcIsBufferComplete(adcp))
	{
		uint8_t edge_index[unimoc::hardware::PHASES];

		for(uint8_t i = 0; i < unimoc::hardware::PHASES; i++)
		{
			/*
			 * Calculate the starting index of the non current channels in the sequence.
			 * Trys to center the 2 non current samples around the edge of the pwm
			 */
			edge_index[i] = (uint8_t)(((uint32_t)unimoc::hardware::pwm::duty_counts[i]*(LENGTH_ADC_SEQ)
					+ (uint32_t)unimoc::hardware::pwm::duty_counts[i]/2)/unimoc::hardware::pwm::PERIOD);
		}

		// Setup the sequences for all the channels
		setup_sequence(&ADCD1, edge_index[0], ADC_CH_CUR_A_DC, ADC_CH_CUR_A_AC, ADC_CH_VDC, ADC_CH_VREF);
		setup_sequence(&ADCD2, edge_index[1], ADC_CH_CUR_B_DC, ADC_CH_CUR_B_AC, ADC_CH_BRDG_TEMP, ADC_CH_MOT_TEMP);
		setup_sequence(&ADCD3, edge_index[2], ADC_CH_CUR_C_DC, ADC_CH_CUR_C_AC, ADC_CH_ACC, ADC_CH_DCC);
	}
	palClearLine(LINE_HALL_B);
}

/*
 * ADC errors callback, should never happen.
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

	(void)adcp;
	(void)err;
}

/**
 * setup new ADC sequence according to the edge index of the PWM
 * @param adcp			Pointer to ADC driver instance
 * @param edge_index	Index of ADC sequence with PWM edge
 * @param cur_dc		DC current channel
 * @param cur_ac		AC current channel
 * @param aux1			First non current ADC channel
 * @param aux2			Secound non current ADC channel
 */
static void setup_sequence(ADCDriver *adcp, uint8_t edge_index, const uint8_t cur_dc, const uint8_t cur_ac, const uint8_t aux1, const uint8_t aux2)
{
	if(unimoc::hardware::pwm::PWMP->tim->CR1 & STM32_TIM_CR1_DIR)
	{
		// PWM is down counting
		edge_index = LENGTH_ADC_SEQ - edge_index - 1;
	}

	switch(edge_index)
	{
	case 0:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(aux2) |
							ADC_SQR3_SQ1_N(aux1);           /* SQR3  */
		break;
	case 1:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(aux2) |
							ADC_SQR3_SQ1_N(aux1);           /* SQR3  */
		break;
	case 2:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(aux2) |
							ADC_SQR3_SQ2_N(aux1) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 3:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(aux2) |
							ADC_SQR3_SQ3_N(aux1) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 4:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(aux2) |
							ADC_SQR3_SQ4_N(aux1) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 5:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(aux2) |
							ADC_SQR3_SQ5_N(aux1) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 6:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(aux2);             /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(aux1) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 7:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(aux2) |
							ADC_SQR2_SQ7_N(aux1);             /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 8:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(aux2) |
							ADC_SQR2_SQ8_N(aux1) |
							ADC_SQR2_SQ7_N(cur_dc);             /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 9:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(aux2) |
							ADC_SQR2_SQ9_N(aux1) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);             /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 10:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(aux2) |
							ADC_SQR2_SQ10_N(aux1) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);             /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 11:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(aux2) |
							ADC_SQR2_SQ11_N(aux1) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 12:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(aux2);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(aux1) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 13:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(cur_dc) |
							ADC_SQR1_SQ14_N(aux2) |
							ADC_SQR1_SQ13_N(aux1);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 14:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(cur_ac) |
							ADC_SQR1_SQ15_N(aux2) |
							ADC_SQR1_SQ14_N(aux1) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	case 15:
		adcp->adc->SQR1 = 	ADC_SQR1_NUM_CH(LENGTH_ADC_SEQ) | /* SQR1  */
							ADC_SQR1_SQ16_N(aux2) |
							ADC_SQR1_SQ15_N(aux1) |
							ADC_SQR1_SQ14_N(cur_ac) |
							ADC_SQR1_SQ13_N(cur_dc);
		adcp->adc->SQR2 = 	ADC_SQR2_SQ12_N(cur_ac) |
							ADC_SQR2_SQ11_N(cur_dc) |
							ADC_SQR2_SQ10_N(cur_ac) |
							ADC_SQR2_SQ9_N(cur_dc) |
							ADC_SQR2_SQ8_N(cur_ac) |
							ADC_SQR2_SQ7_N(cur_dc);          /* SQR2  */
		adcp->adc->SQR3 =	ADC_SQR3_SQ6_N(cur_ac) |
							ADC_SQR3_SQ5_N(cur_dc) |
							ADC_SQR3_SQ4_N(cur_ac) |
							ADC_SQR3_SQ3_N(cur_dc) |
							ADC_SQR3_SQ2_N(cur_ac) |
							ADC_SQR3_SQ1_N(cur_dc);          /* SQR3  */
		break;
	}
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
		ADC_SQR2_SQ9_N(ADC_CH_VREF) |
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
		NULL,
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
	adcStart(&ADCD1, NULL);
	adcStart(&ADCD2, NULL);
	adcStart(&ADCD3, NULL);
	adcSTM32EnableTSVREFE();

	/*
	 * Starts an ADC continuous conversion
	 */
//	palSetLine(LINE_HALL_B);
	adcStartConversion(&ADCD1, &adcgrpcfg1, &samples[0][0], 1);
//	adcStartConversion(&ADCD2, &adcgrpcfg2, &samples[1][0], 1);
//	adcStartConversion(&ADCD3, &adcgrpcfg3, &samples[2][0], 1);
}

void unimoc::hardware::adc::Start(void)
{
//
//	adcStartConversion(&ADCD1, &adcgrpcfg1, &samples[0][0], 1);
//	palSetLine(LINE_HALL_B);
}

