/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2022 Alexander <tecnologic86@gmail.com> Evers

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

///< absolute maximum current FIXME Current resolution not fix.
constexpr float hardware::analog::current::MAX = 100.0f;

///< scaling current to adc range
constexpr float ADC2CURRENT = hardware::analog::current::MAX /(2048.0f);

///< absolute maximum voltage FIXME Current resolution not fix.
constexpr float hardware::analog::voltage::MAX = 1000.0f;

///< scaling voltage to adc range
constexpr float ADC2VOLTAGE = hardware::analog::voltage::MAX /(2048.0f);

///< Samples in ADC sequence.
/// Caution: samples are 16bit but the hole sequence must be 32 bit aligned!
///          so even length of sequence is best choice.
constexpr uint32_t LENGTH_ADC_SEQ = 16;

///< ADC sequences in buffer.
/// Caution: samples are 16bit but the hole sequence must be 32 bit aligned!
///          so even length of sequence is best choice.
/// @note: must be 2 to have the controller running twice per PWM period
constexpr uint32_t ADC_SEQ_BUFFERED = 4;

///< # of ADCs
constexpr uint32_t NUM_OF_ADC = 3;

/* Note, the buffer is aligned to a 32 bytes boundary because limitations
   imposed by the data cache. Note, this is GNU specific, it must be
   handled differently for other compilers.
   Only required if the ADC buffer is placed in a cache-able area.*/
///< dma accessed buffer for the adcs, probably cached
/* Node: Dual mode for all ADCs present. */
__attribute__((aligned (32)))
std::array<std::array<std::array<adcsample_t, LENGTH_ADC_SEQ * 2>, ADC_SEQ_BUFFERED>, NUM_OF_ADC> samples;

///< reference to thread to be woken up in the hardware control cycle.
chibios_rt::ThreadReference hardware::control_thread = nullptr;

///< current offsets
std::int32_t current_offset[hardware::PHASES];

///< phase voltage offsets
std::int32_t voltage_offset[hardware::PHASES];

/**
 * ADC Channel Mapping
 *
 * Pin		Channel			Signal
 *----------------------------------------------
 * PA0		ADC1_IN1		AIN_IA
 * PC1		ADC1_IN7		AIN_ENC_COS
 * PC2		ADC1_IN8		AIN_RES0
 * PA2		ADC1_IN3		AIN_TR0
 *----------------------------------------------
 * PA1		ADC2_IN2		AIN_IB
 * PC0		ADC2_IN6		AIN_ENC_SIN
 * PC3		ADC2_IN9		AIN_RES1
 *----------------------------------------------
 * PB1		ADC3_IN1		AIN_IC
 * PD12		ADC3_IN9		AIN_T0
 * PB0		ADC3_IN12		AIN_TM
 *----------------------------------------------
 * PD10		ADC4_IN7		AIN_IDC
 * PD13		ADC4_IN10		AIN_T1
 * PD9		ADC4_IN13		AIN_T4
 *----------------------------------------------
 * PD11		ADC5_IN8		AIN_VDC
 * PA8		ADC5_IN1		AIN_TC
 * PD14		ADC5_IN11		AIN_T2
 * PD8		ADC5_IN12		AIN_TR1
 */

/**
 * ADC conversion group.
 * Mode:        Continuous, Dual Mode
 * ADC1 Channels:    AIN_IA, AIN_ENC_COS, AIN_RES0, AIN_TR0. */
constexpr std::uint32_t AIN_IA = ADC_CHANNEL_IN1;
constexpr std::uint32_t AIN_ENC_COS = ADC_CHANNEL_IN7;
constexpr std::uint32_t AIN_TR0 = ADC_CHANNEL_IN3;
constexpr std::uint32_t AIN_RES0 = ADC_CHANNEL_IN8;
/**ADC2 Channels:    AIN_IA, AIN_ENC_COS, AIN_RES0. */
constexpr std::uint32_t AIN_IB = ADC_CHANNEL_IN2;
constexpr std::uint32_t AIN_ENC_SIN = ADC_CHANNEL_IN6;
constexpr std::uint32_t AIN_RES1 = ADC_CHANNEL_IN9;
/*************************************************************/
static ADCConversionGroup adcgrpcfg12 = {
	true,
	LENGTH_ADC_SEQ,
	adccallback,
	adcerrorcallback,
	/* ADC CFGR register initialization data.
	   NOTE: The bits DMAEN and DMACFG are enforced internally
			 to the driver, keep them to zero.
	   NOTE: The bits @p ADC_CFGR_CONT or @p ADC_CFGR_DISCEN must be
			 specified in continuous mode or if the buffer depth is
			 greater than one.*/
	ADC_CFGR_EXTEN_1 | ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_2,
	/* ADC CFGR2 register initialization data.*/
	0,
	/* ADC TR1 register initialization data.*/
	0,
	/* ADC TR2 register initialization data.*/
	0,
	/* ADC TR3 register initialization data.*/
	0,
	/* ADC AWD2CR register initialization data.*/
	0,
	/* ADC AWD3CR register initialization data.*/
	0,
	/* ADC CCR register initialization data.
	   NOTE: Put this field to zero if not using oversampling.*/
	ADC_CCR_DUAL_FIELD(1),
	/* ADC SMPRx registers initialization data.*/
	ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5) |
	ADC_SMPR1_SMP_AN3(ADC_SMPR_SMP_2P5) |
	ADC_SMPR1_SMP_AN7(ADC_SMPR_SMP_2P5) |
	ADC_SMPR1_SMP_AN8(ADC_SMPR_SMP_2P5),                  /* SMPR1 */
	0,                                                    /* SMPR2 */
	/* ADC SQRx register initialization data.*/
	ADC_SQR1_SQ1_N(AIN_IA) |
	ADC_SQR1_SQ2_N(AIN_IA) |
	ADC_SQR1_SQ3_N(AIN_IA) |
	ADC_SQR1_SQ4_N(AIN_IA),                               /* SQR1  */
	ADC_SQR2_SQ5_N(AIN_IA) |
	ADC_SQR2_SQ6_N(AIN_IA) |
	ADC_SQR2_SQ7_N(AIN_ENC_COS) |
	ADC_SQR2_SQ8_N(AIN_ENC_COS) |
	ADC_SQR2_SQ9_N(AIN_ENC_COS),                          /* SQR2  */
	ADC_SQR3_SQ10_N(AIN_ENC_COS) |
	ADC_SQR3_SQ11_N(AIN_ENC_COS) |
	ADC_SQR3_SQ12_N(AIN_ENC_COS) |
	ADC_SQR3_SQ13_N(AIN_TR0) |
	ADC_SQR3_SQ14_N(AIN_TR0),                             /* SQR3  */
	ADC_SQR4_SQ15_N(AIN_RES0) |
	ADC_SQR4_SQ16_N(AIN_RES0),                             /* SQR4  */
	/* Slave ADC SMPRx registers initialization data.
	   NOTE: This field is only present in dual mode.*/
	ADC_SMPR1_SMP_AN2(ADC_SMPR_SMP_2P5) |
	ADC_SMPR1_SMP_AN6(ADC_SMPR_SMP_2P5) |
	ADC_SMPR1_SMP_AN9(ADC_SMPR_SMP_2P5),                  /* SSMPR1 */
	0,                                                    /* SSMPR2 */
	/* Slave ADC SQRx register initialization data.
	   NOTE: This field is only present in dual mode.*/
	ADC_SQR1_SQ1_N(AIN_IB) |
	ADC_SQR1_SQ2_N(AIN_IB) |
	ADC_SQR1_SQ3_N(AIN_IB) |
	ADC_SQR1_SQ4_N(AIN_IB),                               /* SSQR1  */
	ADC_SQR2_SQ5_N(AIN_IB) |
	ADC_SQR2_SQ6_N(AIN_IB) |
	ADC_SQR2_SQ7_N(AIN_ENC_SIN) |
	ADC_SQR2_SQ8_N(AIN_ENC_SIN) |
	ADC_SQR2_SQ9_N(AIN_ENC_SIN),                          /* SSQR2  */
	ADC_SQR3_SQ10_N(AIN_ENC_SIN) |
	ADC_SQR3_SQ11_N(AIN_ENC_SIN) |
	ADC_SQR3_SQ12_N(AIN_ENC_SIN) |
	ADC_SQR3_SQ13_N(AIN_RES1) |
	ADC_SQR3_SQ14_N(AIN_RES1),                            /* SSQR3  */
	ADC_SQR4_SQ15_N(AIN_RES1) |
	ADC_SQR4_SQ16_N(AIN_RES1)                             /* SSQR4  */
};

/**
 * ADC conversion group.
 * Mode:        Continuous, Dual Mode
 * ADC3 Channels:    AIN_IC, AIN_T0, AIN_TM */
constexpr std::uint32_t AIN_IC = ADC_CHANNEL_IN1;
constexpr std::uint32_t AIN_T0 = ADC_CHANNEL_IN9;
constexpr std::uint32_t AIN_TM = ADC_CHANNEL_IN12;
/**ADC4 Channels:    AIN_IDC, AIN_T1, AIN_T4 */
constexpr std::uint32_t AIN_IDC = ADC_CHANNEL_IN7;
constexpr std::uint32_t AIN_T1 = ADC_CHANNEL_IN10;
constexpr std::uint32_t AIN_T4 = ADC_CHANNEL_IN13;
/*************************************************************/
static ADCConversionGroup adcgrpcfg34 = {
	true,
	LENGTH_ADC_SEQ,
	nullptr,
	adcerrorcallback,
	/* ADC CFGR register initialization data.
	   NOTE: The bits DMAEN and DMACFG are enforced internally
			 to the driver, keep them to zero.
	   NOTE: The bits @p ADC_CFGR_CONT or @p ADC_CFGR_DISCEN must be
			 specified in continuous mode or if the buffer depth is
			 greater than one.*/
	ADC_CFGR_EXTEN_1 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_2,
	/* ADC CFGR2 register initialization data.*/
	0,
	/* ADC TR1 register initialization data.*/
	0,
	/* ADC TR2 register initialization data.*/
	0,
	/* ADC TR3 register initialization data.*/
	0,
	/* ADC AWD2CR register initialization data.*/
	0,
	/* ADC AWD3CR register initialization data.*/
	0,
	/* ADC CCR register initialization data.
	   NOTE: Put this field to zero if not using oversampling.*/
	ADC_CCR_DUAL_FIELD(1),
	/* ADC SMPRx registers initialization data.*/
	ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5) |
	ADC_SMPR1_SMP_AN9(ADC_SMPR_SMP_2P5),                  /* SMPR1 */
	ADC_SMPR2_SMP_AN12(ADC_SMPR_SMP_2P5),                 /* SMPR2 */
	/* ADC SQRx register initialization data.*/
	ADC_SQR1_SQ1_N(AIN_IC) |
	ADC_SQR1_SQ2_N(AIN_IC) |
	ADC_SQR1_SQ3_N(AIN_IC) |
	ADC_SQR1_SQ4_N(AIN_IC),                               /* SQR1  */
	ADC_SQR2_SQ5_N(AIN_IC) |
	ADC_SQR2_SQ6_N(AIN_IC) |
	ADC_SQR2_SQ7_N(AIN_T0) |
	ADC_SQR2_SQ8_N(AIN_T0) |
	ADC_SQR2_SQ9_N(AIN_T0),                               /* SQR2  */
	ADC_SQR3_SQ10_N(AIN_T0) |
	ADC_SQR3_SQ11_N(AIN_T0) |
	ADC_SQR3_SQ12_N(AIN_T0) |
	ADC_SQR3_SQ13_N(AIN_TM) |
	ADC_SQR3_SQ14_N(AIN_TM),                              /* SQR3  */
	ADC_SQR4_SQ15_N(AIN_TM) |
	ADC_SQR4_SQ16_N(AIN_TM),                              /* SQR4  */
	/* Slave ADC SMPRx registers initialization data.
	   NOTE: This field is only present in dual mode.*/
	ADC_SMPR1_SMP_AN7(ADC_SMPR_SMP_2P5),                  /* SSMPR1 */
	ADC_SMPR2_SMP_AN10(ADC_SMPR_SMP_2P5) |
	ADC_SMPR2_SMP_AN13(ADC_SMPR_SMP_2P5),                 /* SSMPR2 */
	/* Slave ADC SQRx register initialization data.
	   NOTE: This field is only present in dual mode.*/
	ADC_SQR1_SQ1_N(AIN_IDC) |
	ADC_SQR1_SQ2_N(AIN_IDC) |
	ADC_SQR1_SQ3_N(AIN_IDC) |
	ADC_SQR1_SQ4_N(AIN_IDC),                              /* SSQR1  */
	ADC_SQR2_SQ5_N(AIN_IDC) |
	ADC_SQR2_SQ6_N(AIN_IDC) |
	ADC_SQR2_SQ7_N(AIN_T1) |
	ADC_SQR2_SQ8_N(AIN_T1) |
	ADC_SQR2_SQ9_N(AIN_T1),                               /* SSQR2  */
	ADC_SQR3_SQ10_N(AIN_T1) |
	ADC_SQR3_SQ11_N(AIN_T1) |
	ADC_SQR3_SQ12_N(AIN_T1) |
	ADC_SQR3_SQ13_N(AIN_T4) |
	ADC_SQR3_SQ14_N(AIN_T4),                              /* SSQR3  */
	ADC_SQR4_SQ15_N(AIN_T4) |
	ADC_SQR4_SQ16_N(AIN_T4)                               /* SSQR4  */
};

/**
 * ADC conversion group.
 * Mode:        Continuous, Dual Mode
 * ADC5 Channels:    AIN_VDC, AIN_T0, AIN_TM, AIN_TR1 */
constexpr std::uint32_t AIN_VDC = ADC_CHANNEL_IN8;
constexpr std::uint32_t AIN_TC = ADC_CHANNEL_IN1;
constexpr std::uint32_t AIN_T2 = ADC_CHANNEL_IN11;
constexpr std::uint32_t AIN_TR1 = ADC_CHANNEL_IN12;
/*************************************************************/
static ADCConversionGroup adcgrpcfg5 = {
	true,
	LENGTH_ADC_SEQ,
	nullptr,
	adcerrorcallback,
	/* ADC CFGR register initialization data.
	   NOTE: The bits DMAEN and DMACFG are enforced internally
			 to the driver, keep them to zero.
	   NOTE: The bits @p ADC_CFGR_CONT or @p ADC_CFGR_DISCEN must be
			 specified in continuous mode or if the buffer depth is
			 greater than one.*/
	ADC_CFGR_EXTEN_1 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_2,
	/* ADC CFGR2 register initialization data.*/
	0,
	/* ADC TR1 register initialization data.*/
	0,
	/* ADC TR2 register initialization data.*/
	0,
	/* ADC TR3 register initialization data.*/
	0,
	/* ADC AWD2CR register initialization data.*/
	0,
	/* ADC AWD3CR register initialization data.*/
	0,
	/* ADC CCR register initialization data.
	   NOTE: Put this field to zero if not using oversampling.*/
	0,
	/* ADC SMPRx registers initialization data.*/
	ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5) |
	ADC_SMPR1_SMP_AN8(ADC_SMPR_SMP_2P5),                  /* SMPR1 */
	ADC_SMPR2_SMP_AN11(ADC_SMPR_SMP_2P5) |
	ADC_SMPR2_SMP_AN12(ADC_SMPR_SMP_2P5),                 /* SMPR2 */
	/* ADC SQRx register initialization data.*/
	ADC_SQR1_SQ1_N(AIN_VDC) |
	ADC_SQR1_SQ2_N(AIN_VDC) |
	ADC_SQR1_SQ3_N(AIN_VDC) |
	ADC_SQR1_SQ4_N(AIN_VDC),                               /* SQR1  */
	ADC_SQR2_SQ5_N(AIN_VDC) |
	ADC_SQR2_SQ6_N(AIN_VDC) |
	ADC_SQR2_SQ7_N(AIN_TC) |
	ADC_SQR2_SQ8_N(AIN_TC) |
	ADC_SQR2_SQ9_N(AIN_TC),                               /* SQR2  */
	ADC_SQR3_SQ10_N(AIN_TC) |
	ADC_SQR3_SQ11_N(AIN_T2) |
	ADC_SQR3_SQ12_N(AIN_T2) |
	ADC_SQR3_SQ13_N(AIN_T2) |
	ADC_SQR3_SQ14_N(AIN_T2),                              /* SQR3  */
	ADC_SQR4_SQ15_N(AIN_TR1) |
	ADC_SQR4_SQ16_N(AIN_TR1),                             /* SQR4  */
	/* Slave ADC SMPRx registers initialization data.
	   NOTE: This field is only present in dual mode.*/
	0,                                                    /* SSMPR1 */
	0,                                                    /* SSMPR2 */
	/* Slave ADC SQRx register initialization data.
	   NOTE: This field is only present in dual mode.*/
	0,                                                    /* SSQR1  */
	0,                                                    /* SSQR2  */
	0,                                                    /* SSQR3  */
	0,                                                    /* SSQR4  */
};


/**
 * Initialize ADC hardware
 */
void hardware_analog_Init(void)
{
	/*
	 * Activates the ADC drivers and the VREF input.
	 */
	adcStart(&ADCD1, nullptr);
	adcStart(&ADCD3, nullptr);
	adcStart(&ADCD5, nullptr);

	/*
	 * Starts an ADC continuous conversion
	 */
	adcStartConversion(&ADCD1, &adcgrpcfg12, &samples[0][0][0], ADC_SEQ_BUFFERED);
	adcStartConversion(&ADCD3, &adcgrpcfg34, &samples[1][0][0], ADC_SEQ_BUFFERED);
	adcStartConversion(&ADCD5, &adcgrpcfg5,  &samples[2][0][0], ADC_SEQ_BUFFERED);

}

/**
 * Get the phase current values in the last control cycle
 * @param currents references to the current samples
 */
void hardware::analog::current::Phase(systems::abc& currents)
{
	for(std::uint_fast8_t i = 0; i < PHASES; i++)
	{
		std::int_fast32_t sum = 0;
		std::int_fast32_t cnt = 0;

		for (std::uint_fast8_t s = 0; s < ADC_SEQ_BUFFERED; s++)
		{
			for (std::uint_fast8_t a = 0; a < 6; a++)
			{
				sum += samples[i][s][a];
				cnt += 2;
			}
			currents.array[i] = ADC2CURRENT * (float)(current_offset[i] - sum) / (float)cnt;
		}
	}
}

/**
 * Get the phase voltage values in the last control cycle
 * @param voltages references to the current samples
 */
void hardware::analog::voltage::Phase(systems::abc& voltages)
{
	(void)voltages; /// TODO need SPI for Voltage ADCs
}

/**
 * set phase current offsets
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
		current_offset[i] = sum;
	}
}

/**
 * set phase voltage offsets
 */
void hardware::analog::voltage::SetOffset(void)
{
	(void)voltage_offset; /// TODO need SPI for Voltage ADCs
}

/**
 * Read the DC Bus voltage
 * @return DC Bus voltage in Volts
 */
float hardware::analog::voltage::DCBus(void)
{
	float result = 0.0f;

	std::int_fast32_t sum = 0;
	std::int_fast32_t cnt = 0;

	for (std::uint_fast8_t s = 0; s < ADC_SEQ_BUFFERED; s++)
	{
		for (std::uint_fast8_t a = 0; a < 6; a++)
		{
			sum += samples[4][s][a];
			cnt += 2;
		}
	}
	result = ADC2VOLTAGE * (float)sum / (float)cnt;
	return (result);
}


/**
 * Get the temperature of the power electronics
 * @return Temperature of the power electronics in °C
 */
float hardware::analog::temperature::Bridge(void)
{
	return 0.0f;
}

/**
 * Get the temperature of the motor
 * @return Temperature of the Motor in °C
 */
float hardware::analog::temperature::Motor(void)
{
	return 0.0f;
}


/**
 * get analog input
 * @return analog in put 0 - 1
 */
float hardware::analog::input(void)
{
	return 0.0f;
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


	if(!hardware::control_thread.isNull())
	{
		/* Wakes up the thread.*/
		osalSysLockFromISR();
		chEvtSignalI(hardware::control_thread.getInner(), (eventmask_t)1);
		osalSysUnlockFromISR();
	}
}

