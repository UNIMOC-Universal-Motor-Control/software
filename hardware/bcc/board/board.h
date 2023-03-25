/*
    ChibiOS - Copyright (C) 2006..2020 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for PH High Voltage Rev1 board.
 */

/*
 * Board identifier.
 */
#define BOARD_UNIMOC_PHHVR1
#define BOARD_NAME                  "UNIMOC_BCC"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32G473xx

/*
 * IO pins assignments.
 */
#define GPIOA_AIN_IA                0U
#define GPIOA_AIN_IB                1U
#define GPIOA_AIN_VA                2U
#define GPIOA_LED_RUN               3U
#define GPIOA_LED_ERR               4U
#define GPIOA_PULSE_CRK             5U
#define GPIOA_AIN_VB                6U
#define GPIOA_PWM_AL                7U
#define GPIOA_AIN_TMOT              8U
#define GPIOA_AIN_TBRDG             9U
#define GPIOA_PWM_BRK               10U
#define GPIOA_USB_DP                11U
#define GPIOA_USB_DM                12U
#define GPIOA_SWD_IO                13U
#define GPIOA_SWD_CLK               14U
#define GPIOA_LED_PWM               15U

#define GPIOB_PWM_BL                0U
#define GPIOB_AIN_IC                1U
#define GPIOB_PULSE_MOT             2U
#define GPIOB_CAN_RX                3U
#define GPIOB_CAN_TX                4U
#define GPIOB_PWM_CL                5U
#define GPIOB_PWM_AH                6U
#define GPIOB_CAN_SLP               7U
#define GPIOB_PWM_BH                8U
#define GPIOB_PWM_CH                9U
#define GPIOB_DISP_TX               10U
#define GPIOB_DISP_RX               11U
#define GPIOB_AIN_VDC               12U
#define GPIOB_AIN_VC                13U
#define GPIOB_AIN_CRK_TRQ           14U
#define GPIOB_PWM_BRK               15U

#define GPIOC_PIN0                  0U
#define GPIOC_PIN1                  1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_PIN4                  4U
#define GPIOC_PIN5                  5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_PIN8                  8U
#define GPIOC_PIN9                  9U
#define GPIOC_PIN10                 10U
#define GPIOC_PIN11                 11U
#define GPIOC_PIN12                 12U
#define GPIOC_HALL_A                13U
#define GPIOC_HALL_B                14U
#define GPIOC_HALL_C                15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_PIN2                  2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_PIN5                  5U
#define GPIOD_PIN6                  6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PIN12                 12U
#define GPIOD_PIN13                 13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_PIN6                  6U
#define GPIOE_PIN7                  7U
#define GPIOE_PIN8                  8U
#define GPIOE_PIN9                  9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PIN14                 14U
#define GPIOE_PIN15                 15U

#define GPIOF_OSC_IN                0U
#define GPIOF_OSC_OUT               1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_PIN2                  2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_PIN9                  9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_PIN13                 13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_LED_RUN                PAL_LINE(GPIOA, GPIOA_LED_RUN)
#define LINE_LED_ERR                PAL_LINE(GPIOA, GPIOA_LED_ERR)
#define LINE_LED_PWM                PAL_LINE(GPIOA, GPIOA_LED_PWM)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))
#define PIN_LOCKR_DISABLED(n)       (0U << (n))
#define PIN_LOCKR_ENABLED(n)        (1U << (n))

/*
 * GPIOA setup:
 *
 * PA0  - AIN_IA                    (analog).
 * PA1  - AIN_IB                    (analog).
 * PA2  - AIN_VA	                (analog).
 * PA3  - LED_RUN	                (output pushpull minimum).
 * PA4  - LED_ERR	                (output pushpull minimum)
 * PA5  - PULSE_CRK                 (input pulldown).
 * PA6  - AIN_VB	                (analog).
 * PA7  - PWM_AL	                (alternate 4).
 * PA8  - AIN_TMOT	                (analog).
 * PA9  - AIN_TBRDG                 (analog).
 * PA10 - PWM_BRK                   (alternate 11).
 * PA11 - USB_DP                    (analog).
 * PA12 - USB_DM                    (analog).
 * PA13 - SWD_IO                    (alternate 0).
 * PA14 - SWD_CLK 	                (alternate 0).
 * PA15 - LED_PWM	                (output pushpull minimum).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG	(GPIOA_AIN_IA     ) |     \
                                     PIN_MODE_ANALOG	(GPIOA_AIN_IB     ) |     \
                                     PIN_MODE_ANALOG	(GPIOA_AIN_VA     ) |     \
                                     PIN_MODE_OUTPUT	(GPIOA_LED_RUN    ) |     \
                                     PIN_MODE_OUTPUT	(GPIOA_LED_ERR    ) |     \
                                     PIN_MODE_INPUT	    (GPIOA_PULSE_CRK  ) |     \
                                     PIN_MODE_ANALOG	(GPIOA_AIN_VB     ) |     \
                                     PIN_MODE_ALTERNATE	(GPIOA_PWM_AL     ) |     \
                                     PIN_MODE_ANALOG	(GPIOA_AIN_TMOT   ) |     \
                                     PIN_MODE_ANALOG	(GPIOA_AIN_TBRDG  ) |     \
                                     PIN_MODE_ANALOG	(GPIOA_PWM_BRK    ) |     \
                                     PIN_MODE_ANALOG	(GPIOA_USB_DP     ) |     \
                                     PIN_MODE_ANALOG	(GPIOA_USB_DM     ) |     \
                                     PIN_MODE_ALTERNATE	(GPIOA_SWD_IO     ) |     \
                                     PIN_MODE_ALTERNATE	(GPIOA_SWD_CLK    ) |     \
                                     PIN_MODE_OUTPUT	(GPIOA_LED_PWM    ))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_AIN_IA     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AIN_IB     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AIN_VA     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_RUN    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_ERR    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PULSE_CRK  ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AIN_VB     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM_AL     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AIN_TMOT   ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AIN_TBRDG  ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM_BRK    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWD_IO     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWD_CLK    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_PWM    ))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_AIN_IA     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_AIN_IB     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_AIN_VA     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_RUN    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_ERR    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_PULSE_CRK  ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_AIN_VB     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_PWM_AL     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_AIN_TMOT   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_AIN_TBRDG  ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_PWM_BRK    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_USB_DP     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_USB_DM     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_SWD_IO     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_SWD_CLK    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_PWM    ))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_AIN_IA     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_AIN_IB     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_AIN_VA     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_RUN    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_ERR    ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PULSE_CRK  ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_AIN_VB     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PWM_AL     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_AIN_TMOT   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_AIN_TBRDG  ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PWM_BRK    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DP     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DM     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWD_IO     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWD_CLK    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_PWM    ))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW (GPIOA_AIN_IA     ) |         \
                                     PIN_ODR_LOW (GPIOA_AIN_IB     ) |         \
                                     PIN_ODR_LOW (GPIOA_AIN_VA     ) |         \
                                     PIN_ODR_HIGH(GPIOA_LED_RUN    ) |         \
                                     PIN_ODR_HIGH(GPIOA_LED_ERR    ) |         \
                                     PIN_ODR_LOW (GPIOA_PULSE_CRK  ) |         \
                                     PIN_ODR_LOW (GPIOA_AIN_VB     ) |         \
                                     PIN_ODR_LOW (GPIOA_PWM_AL     ) |         \
                                     PIN_ODR_LOW (GPIOA_AIN_TMOT   ) |         \
                                     PIN_ODR_LOW (GPIOA_AIN_TBRDG  ) |         \
                                     PIN_ODR_LOW (GPIOA_PWM_BRK    ) |         \
                                     PIN_ODR_LOW (GPIOA_USB_DP     ) |         \
                                     PIN_ODR_LOW (GPIOA_USB_DM     ) |         \
                                     PIN_ODR_LOW (GPIOA_SWD_IO     ) |         \
                                     PIN_ODR_LOW (GPIOA_SWD_CLK    ) |         \
                                     PIN_ODR_HIGH(GPIOA_LED_PWM    ))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_AIN_IA     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_AIN_IB     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_AIN_VA     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_LED_RUN    , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_LED_ERR    , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_PULSE_CRK  , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_AIN_VB     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_PWM_AL     , 4U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_AIN_TMOT   , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_AIN_TBRDG  , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_PWM_BRK    ,11U) |     \
                                     PIN_AFIO_AF(GPIOA_USB_DP     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_USB_DM     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_SWD_IO     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_SWD_CLK    , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_LED_PWM    , 0U))

/*
 * GPIOB setup:
 *
 * PB0  - PWM_BL                    (alternate 4).
 * PB1  - AIN_IC                    (analog).
 * PB2  - PULSE_MOT                 (input pulldown).
 * PB3  - CAN_RX                    (alternate 11).
 * PB4  - CAN_TX                    (alternate 11).
 * PB5  - PWM_CL                    (alternate 3).
 * PB6  - PWM_AH                    (alternate 5).
 * PB7  - CAN_SLP                   (output pushpull minimum).
 * PB8  - PWM_BH                    (alternate 10).
 * PB9  - PWM_CH                    (alternate 10).
 * PB10 - DISP_TX                   (alternate 7).
 * PB11 - DISP_RX                   (alternate 7).
 * PB12 - AIN_VDC                   (analog).
 * PB13 - AIN_VC                    (analog).
 * PB14 - AIN_CRK_TRQ               (analog).
 * PB15 - PWM_BRK                   (output opendrain minimum).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE  (GPIOB_PWM_BL      ) |  \
                                     PIN_MODE_ANALOG     (GPIOB_AIN_IC      ) |  \
                                     PIN_MODE_INPUT      (GPIOB_PULSE_MOT   ) |  \
                                     PIN_MODE_ALTERNATE  (GPIOB_CAN_RX      ) |  \
                                     PIN_MODE_ALTERNATE  (GPIOB_CAN_TX      ) |  \
                                     PIN_MODE_ALTERNATE  (GPIOB_PWM_CL      ) |  \
                                     PIN_MODE_ALTERNATE  (GPIOB_PWM_AH      ) |  \
                                     PIN_MODE_OUTPUT     (GPIOB_CAN_SLP     ) |  \
                                     PIN_MODE_ALTERNATE  (GPIOB_PWM_BH      ) |  \
                                     PIN_MODE_ALTERNATE  (GPIOB_PWM_CH      ) |  \
                                     PIN_MODE_ALTERNATE  (GPIOB_DISP_TX     )|  \
                                     PIN_MODE_ALTERNATE  (GPIOB_DISP_RX     ) |  \
                                     PIN_MODE_ANALOG     (GPIOB_AIN_VDC     ) |  \
                                     PIN_MODE_ANALOG     (GPIOB_AIN_VC      ) |  \
                                     PIN_MODE_ANALOG     (GPIOB_AIN_CRK_TRQ ) |  \
                                     PIN_MODE_ANALOG     (GPIOB_PWM_BRK     ))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL (GPIOB_PWM_BL      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_AIN_IC      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_PULSE_MOT   ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_CAN_RX      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_CAN_TX      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_PWM_CL      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_PWM_AH      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_CAN_SLP     ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_PWM_BH      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_PWM_CH      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_DISP_TX     ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_DISP_RX     ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_AIN_VDC     ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_AIN_VC      ) |  \
                                     PIN_OTYPE_PUSHPULL (GPIOB_AIN_CRK_TRQ ) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PWM_BRK     ))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOB_PWM_BL      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_AIN_IC      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PULSE_MOT   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_CAN_RX      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_CAN_TX      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PWM_CL      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PWM_AH      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_CAN_SLP     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PWM_BH      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PWM_CH      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_DISP_TX     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_DISP_RX     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_AIN_VDC     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_AIN_VC      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_AIN_CRK_TRQ ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PWM_BRK     ))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PWM_BL      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_AIN_IC      ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PULSE_MOT   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_RX      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_TX      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PWM_CL      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PWM_AH      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_SLP     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PWM_BH      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PWM_CH      ) |  \
                                     PIN_PUPDR_PULLUP  (GPIOB_DISP_TX     ) |  \
                                     PIN_PUPDR_PULLUP  (GPIOB_DISP_RX     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_AIN_VDC     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_AIN_VC      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_AIN_CRK_TRQ ) |  \
                                     PIN_PUPDR_PULLUP  (GPIOB_PWM_BRK     ))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW (GPIOB_PWM_BL      ) |  \
                                     PIN_ODR_LOW (GPIOB_AIN_IC      ) |  \
                                     PIN_ODR_LOW (GPIOB_PULSE_MOT   ) |  \
                                     PIN_ODR_LOW (GPIOB_CAN_RX      ) |  \
                                     PIN_ODR_LOW (GPIOB_CAN_TX      ) |  \
                                     PIN_ODR_LOW (GPIOB_PWM_CL      ) |  \
                                     PIN_ODR_LOW (GPIOB_PWM_AH      ) |  \
                                     PIN_ODR_LOW (GPIOB_CAN_SLP     ) |  \
                                     PIN_ODR_LOW (GPIOB_PWM_BH      ) |  \
                                     PIN_ODR_LOW (GPIOB_PWM_CH      ) |  \
                                     PIN_ODR_LOW (GPIOB_DISP_TX     ) |  \
                                     PIN_ODR_LOW (GPIOB_DISP_RX     ) |  \
                                     PIN_ODR_LOW (GPIOB_AIN_VDC     ) |  \
                                     PIN_ODR_LOW (GPIOB_AIN_VC      ) |  \
                                     PIN_ODR_LOW (GPIOB_AIN_CRK_TRQ ) |  \
                                     PIN_ODR_LOW (GPIOB_PWM_BRK     ))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PWM_BL       , 4U) |          \
                                     PIN_AFIO_AF(GPIOB_AIN_IC       , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PULSE_MOT    , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_CAN_RX       ,11U) |          \
                                     PIN_AFIO_AF(GPIOB_CAN_TX       ,11U) |          \
                                     PIN_AFIO_AF(GPIOB_PWM_CL       , 3U) |          \
                                     PIN_AFIO_AF(GPIOB_PWM_AH       , 5U) |          \
                                     PIN_AFIO_AF(GPIOB_CAN_SLP      , 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PWM_BH       ,10U) |          \
                                     PIN_AFIO_AF(GPIOB_PWM_CH       ,10U) |          \
                                     PIN_AFIO_AF(GPIOB_DISP_TX      , 7U)|          \
                                     PIN_AFIO_AF(GPIOB_DISP_RX      , 7U) |          \
                                     PIN_AFIO_AF(GPIOB_AIN_VDC      , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_AIN_VC       , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_AIN_CRK_TRQ  , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PWM_BRK      , 0U))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (analog).
 * PC1  - PIN1                      (analog).
 * PC2  - PIN2                      (analog).
 * PC3  - PIN3                      (analog).
 * PC4  - PIN4                      (analog).
 * PC5  - PIN5                      (analog).
 * PC6  - PIN6                      (analog).
 * PC7  - PIN7                      (analog).
 * PC8  - PIN8                      (analog).
 * PC9  - PIN9                      (analog).
 * PC10 - PIN10                     (analog).
 * PC11 - PIN11                     (analog).
 * PC12 - PIN12                     (analog).
 * PC13 - HALL_A                    (input).
 * PC14 - HALL_B                    (input).
 * PC15 - HALL_C                    (input).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG   (GPIOC_PIN0       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN1       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN2       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN3       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN4       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN5       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN6       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN7       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN8       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN9       ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN10      ) |     \
                                     PIN_MODE_ANALOG   (GPIOC_PIN11      ) |    \
                                     PIN_MODE_ANALOG   (GPIOC_PIN12      ) |    \
                                     PIN_MODE_INPUT    (GPIOC_HALL_A     ) |     \
                                     PIN_MODE_INPUT    (GPIOC_HALL_B     ) |     \
                                     PIN_MODE_INPUT    (GPIOC_HALL_C     ))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_HALL_A     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_HALL_B     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_HALL_C     ))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_PIN0       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN1       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN2       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN3       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN4       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN5       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN6       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN7       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN8       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN9       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN10      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN11      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN12      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_HALL_A     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_HALL_B     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_HALL_C     ))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN1       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN4       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN5       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN6       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN7       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN8       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN9       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN10      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN11      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN12      ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOC_HALL_A     ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOC_HALL_B     ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOC_HALL_C     ))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW (GPIOC_PIN0        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN1        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN2        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN3        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN4        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN5        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN6        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN7        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN8        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN9        ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN10       ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN11       ) |         \
                                     PIN_ODR_LOW (GPIOC_PIN12       ) |         \
                                     PIN_ODR_LOW (GPIOC_HALL_A      ) |         \
                                     PIN_ODR_LOW (GPIOC_HALL_B      ) |         \
                                     PIN_ODR_LOW (GPIOC_HALL_C      ))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN1        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN2        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN3        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN4        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN5        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN6        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN7        , 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN9        , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN10       , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN11       , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN12       , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_HALL_A      , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_HALL_B      , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_HALL_C      , 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (analog).
 * PD1  - PIN1                      (analog).
 * PD2  - PIN2                      (analog).
 * PD3  - PIN3                      (analog).
 * PD4  - PIN4                      (analog).
 * PD5  - PIN5                      (analog).
 * PD6  - PIN6                      (analog).
 * PD7  - PIN7                      (analog).
 * PD8  - PIN8                      (analog).
 * PD9  - PIN9                      (analog).
 * PD10 - PIN10                     (analog).
 * PD11 - PIN11                     (analog).
 * PD12 - PIN12                     (analog).
 * PD13 - PIN13                     (analog).
 * PD14 - PIN14                     (analog).
 * PD15 - PIN15                     (analog).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ANALOG   (GPIOD_PIN0      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN1      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN2      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN3      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN4      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN5      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN6      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN7      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN8      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN9      ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN10     ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN11     ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN12     ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN13     ) |       \
                                     PIN_MODE_ANALOG   (GPIOD_PIN14     ) |       \
                                     PIN_MODE_OUTPUT   (GPIOD_PIN15     ))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10     ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11     ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12     ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13     ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14     ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15     ))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOD_PIN0      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN1      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN2      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN3      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN4      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN5      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN6      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN7      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN8      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN9      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN10     ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN11     ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN12     ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN13     ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN14     ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN15     ))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_PIN0      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN1      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN2      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10     ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11     ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN12     ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN13     ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14     ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15     ))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_PIN0      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN1      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN2      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN3      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN4      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN5      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN6      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN7      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN8      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN9      ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN10     ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN11     ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN12     ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN13     ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN14     ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN15     ))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN1      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN2      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN3      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN4      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN5      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN6      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN7      , 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN9      , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN10     , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN11     , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN12     , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN13     , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN14     , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN15     , 0U))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (analog).
 * PE1  - PIN1                      (analog).
 * PE2  - PIN2                      (analog).
 * PE3  - PIN3                      (analog).
 * PE4  - PIN4                      (analog).
 * PE5  - PIN5                      (analog).
 * PE6  - PIN6                      (analog).
 * PE7  - PIN7                      (analog).
 * PE8  - PIN8                      (analog).
 * PE9  - PIN9                      (analog).
 * PE10 - PIN10                     (analog).
 * PE11 - PIN11                     (analog).
 * PE12 - PIN12                     (analog).
 * PE13 - PIN13                     (analog).
 * PE14 - PIN14                     (analog).
 * PE15 - PIN15                     (analog).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ANALOG   (GPIOE_PIN0        ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN1        ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN2        ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN3        ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN4        ) |          \
                                     PIN_MODE_ANALOG   (GPIOE_PIN5        ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN6        ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN7        ) |          \
                                     PIN_MODE_ANALOG   (GPIOE_PIN8        ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN9        ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN10       ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN11       ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN12       ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN13       ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN14       ) |       \
                                     PIN_MODE_ANALOG   (GPIOE_PIN15       ))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9        ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10       ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11       ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12       ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13       ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14       ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15       ))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOE_PIN0        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN1        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN2        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN3        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN4        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN5        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN6        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN7        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN8        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN9        ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN10       ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN11       ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN12       ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN13       ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN14       ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN15       ))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_PIN0        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN7        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN8        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN9        ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10       ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN11       ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN12       ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN13       ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN14       ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15       ))
#define VAL_GPIOE_ODR               (PIN_ODR_LOW(GPIOE_PIN0        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN1        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN2        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN3        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN4        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN5        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN6        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN7        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN8        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN9        ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN10       ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN11       ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN12       ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN13       ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN14       ) |              \
                                     PIN_ODR_LOW(GPIOE_PIN15       ))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN1        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN2        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN3        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN4        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN5        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN6        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN7        , 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN9        , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN10       , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN11       , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN12       , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN13       , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN14       , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN15       , 0U))

/*
 * GPIOF setup:
 *
 * PF0  - OSC_IN                    (analog).
 * PF1  - OSC_OUT                   (analog).
 * PF2  - PIN2                      (analog).
 * PF3  - PIN3                      (analog).
 * PF4  - PIN4                      (analog).
 * PF5  - PIN5                      (analog).
 * PF6  - PIN6                      (analog).
 * PF7  - PIN7                      (analog).
 * PF8  - PIN8                      (analog).
 * PF9  - PIN9                      (analog).
 * PF10 - PIN10                     (analog).
 * PF11 - PIN11                     (analog).
 * PF12 - PIN12                     (analog).
 * PF13 - PIN13                     (analog).
 * PF14 - PIN14                     (analog).
 * PF15 - PIN15                     (analog).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ANALOG(GPIOF_OSC_IN) |        \
                                     PIN_MODE_ANALOG(GPIOF_OSC_OUT) |       \
                                     PIN_MODE_ANALOG(GPIOF_PIN2) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN3) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN4) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN5) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN6) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN7) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN8) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN9) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN10) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN11) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN12) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN13) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN14) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOF_OSC_IN) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_OSC_OUT) |    \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_OSC_OUT) |    \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_OSC_IN) |            \
                                     PIN_ODR_LOW(GPIOF_OSC_OUT) |           \
                                     PIN_ODR_LOW(GPIOF_PIN2) |              \
                                     PIN_ODR_LOW(GPIOF_PIN3) |              \
                                     PIN_ODR_LOW(GPIOF_PIN4) |              \
                                     PIN_ODR_LOW(GPIOF_PIN5) |              \
                                     PIN_ODR_LOW(GPIOF_PIN6) |              \
                                     PIN_ODR_LOW(GPIOF_PIN7) |              \
                                     PIN_ODR_LOW(GPIOF_PIN8) |              \
                                     PIN_ODR_LOW(GPIOF_PIN9) |              \
                                     PIN_ODR_LOW(GPIOF_PIN10) |             \
                                     PIN_ODR_LOW(GPIOF_PIN11) |             \
                                     PIN_ODR_LOW(GPIOF_PIN12) |             \
                                     PIN_ODR_LOW(GPIOF_PIN13) |             \
                                     PIN_ODR_LOW(GPIOF_PIN14) |             \
                                     PIN_ODR_LOW(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (analog).
 * PG1  - PIN1                      (analog).
 * PG2  - PIN2                      (analog).
 * PG3  - PIN3                      (analog).
 * PG4  - PIN4                      (analog).
 * PG5  - PIN5                      (analog).
 * PG6  - PIN6                      (analog).
 * PG7  - PIN7                      (analog).
 * PG8  - PIN8                      (analog).
 * PG9  - PIN9                      (analog).
 * PG10 - PIN10                     (analog).
 * PG11 - PIN11                     (analog).
 * PG12 - PIN12                     (analog).
 * PG13 - PIN13                     (analog).
 * PG14 - PIN14                     (analog).
 * PG15 - PIN15                     (analog).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ANALOG(GPIOG_PIN0) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN1) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN2) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN3) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN4) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN5) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN6) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN7) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN8) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN9) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN10) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN11) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN12) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN13) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN14) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOG_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_LOW(GPIOG_PIN0) |              \
                                     PIN_ODR_LOW(GPIOG_PIN1) |              \
                                     PIN_ODR_LOW(GPIOG_PIN2) |              \
                                     PIN_ODR_LOW(GPIOG_PIN3) |              \
                                     PIN_ODR_LOW(GPIOG_PIN4) |              \
                                     PIN_ODR_LOW(GPIOG_PIN5) |              \
                                     PIN_ODR_LOW(GPIOG_PIN6) |              \
                                     PIN_ODR_LOW(GPIOG_PIN7) |              \
                                     PIN_ODR_LOW(GPIOG_PIN8) |              \
                                     PIN_ODR_LOW(GPIOG_PIN9) |              \
                                     PIN_ODR_LOW(GPIOG_PIN10) |             \
                                     PIN_ODR_LOW(GPIOG_PIN11) |             \
                                     PIN_ODR_LOW(GPIOG_PIN12) |             \
                                     PIN_ODR_LOW(GPIOG_PIN13) |             \
                                     PIN_ODR_LOW(GPIOG_PIN14) |             \
                                     PIN_ODR_LOW(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
