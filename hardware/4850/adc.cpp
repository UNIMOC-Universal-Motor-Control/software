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
#include "hardware_interface.hpp"
#include "hal.h"

///< Each adc handles this number of channels
constexpr uint32_t CHANNELS_PER_ADC = 4;

///< External trigger delay in adc clock cycles
constexpr uint32_t EXT_TRIG_DELAY = 2;

///< Pure Conversion time in ADC Clockcycles
constexpr uint32_t CONV_TIME = 12;


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
typedef struct
{
	struct
	{
		adcsample_t cur_a_dc;
		adcsample_t cur_a_ac;
		adcsample_t vdc;
		adcsample_t brdg_temp;
	}adc1;
	struct
	{
		adcsample_t cur_a_dc;
		adcsample_t cur_a_ac;
		adcsample_t vdc;
		adcsample_t brdg_temp;
	}adc2;

};

/* Note, the buffer is aligned to a 32 bytes boundary because limitations
   imposed by the data cache. Note, this is GNU specific, it must be
   handled differently for other compilers.
   Only required if the ADC buffer is placed in a cache-able area.*/
__attribute__((aligned (32))) static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0;
static void adccallback(ADCDriver *adcp) {

#if !DMA_BUFFERS_COHERENCE
  /* DMA buffer invalidation because data cache, only invalidating the
     half buffer just filled.
     Only required if the ADC buffer is placed in a cache-able area.*/
  dmaBufferInvalidate(buffer,
                      n * adcp->grpp->num_channels * sizeof (adcsample_t));
#endif

  /* Updating counters.*/
  if (adcIsBufferComplete(adcp)) {
    nx += 1;
  }
  else {
    ny += 1;
  }
}

/*
 * ADC errors callback, should never happen.
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 2 channels, HS triggered by
 *              GPT4-TRGO.
 * Channels:    Sensor, VRef.
 */
static const ADCConversionGroup adcgrpcfg1 = {
  true,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0,                                                    /* CR1   */
  ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(12),        /* CR2   */
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_144) |
  ADC_SMPR1_SMP_VREF(ADC_SAMPLE_144),                   /* SMPR1 */
  0,                                                    /* SMPR2 */
  0,                                                    /* HTR */
  0,                                                    /* LTR */
  0,                                                    /* SQR1  */
  0,                                                    /* SQR2  */
  ADC_SQR3_SQ2_N(ADC_CHANNEL_SENSOR) |
  ADC_SQR3_SQ1_N(ADC_CHANNEL_VREFINT)                   /* SQR3  */
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
	 * Activates the ADC1 driver and the temperature sensor.
	 */
	adcStart(&ADCD1, NULL);
	adcSTM32EnableTSVREFE();

	/*
	 * Starts an ADC continuous conversion triggered with a period of
	 * 1/10000 second.
	 */
	adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
}


