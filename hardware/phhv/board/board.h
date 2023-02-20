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
#define BOARD_NAME                  "UNIMOC_PHHV"

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
#define GPIOA_TEMP_RES0             2U
#define GPIOA_REF_NSTANDBY          3U
#define GPIOA_RES_REF				4U
#define GPIOA_SPI1_SCK              5U
#define GPIOA_SPI1_MISO             6U
#define GPIOA_SPI1_MOSI             7U
#define GPIOA_AIN_TCOOLING          8U
#define GPIOA_VDC_NCS               9U
#define GPIOA_RES_NFAULT            10U
#define GPIOA_USB_DM                11U
#define GPIOA_USB_DP                12U
#define GPIOA_JTAG_TMS              13U
#define GPIOA_JTAG_CLK              14U
#define GPIOA_OVP_C                 15U

#define GPIOB_TEMP_MOTOR            0U
#define GPIOB_AIN_IC                1U
#define GPIOB_OCP_B                 2U
#define GPIOB_TRACESWO              3U
#define GPIOB_PIN4                  4U
#define GPIOB_FDCAN2_RX             5U
#define GPIOB_FDCAN2_TX             6U
#define GPIOB_IABC_NCS              7U
#define GPIOB_PIN8                  8U
#define GPIOB_OCP_HVDC              9U
#define GPIOB_PWM_HWBREAK           10U
#define GPIOB_GATE_EN               11U
#define GPIOB_OVP_B                 12U
#define GPIOB_SPI2_SCK              13U
#define GPIOB_SPI2_MISO             14U
#define GPIOB_SPI2_MOSI             15U

#define GPIOC_RES_SIN               0U
#define GPIOC_RES_COS               1U
#define GPIOC_AIN_24V               2U
#define GPIOC_AIN_15V               3U
#define GPIOC_OCP_A                 4U
#define GPIOC_OVP_A                 5U
#define GPIOC_ENC_A                 6U
#define GPIOC_ENC_B                 7U
#define GPIOC_I2C3_SCL              8U
#define GPIOC_I2C3_SDA              9U
#define GPIOC_SPI3_SCK              10U
#define GPIOC_SPI3_MISO             11U
#define GPIOC_SPI3_MOSI             12U
#define GPIOC_OVP_HVDC		        13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_FDCAN1_RX             0U
#define GPIOD_FDCAN1_TX             1U
#define GPIOD_ENC_Z                 2U
#define GPIOD_OCP_C                 3U
#define GPIOD_RS485_DE              4U
#define GPIOD_RS485_TX              5U
#define GPIOD_RS485_RX              6U
#define GPIOD_PIN7                  7U
#define GPIOD_TEMP_RES1             8U
#define GPIOD_THERMO4               9U
#define GPIOD_AIN_IDC               10U
#define GPIOD_AIN_VDC               11U
#define GPIOD_THERMO1               12U
#define GPIOD_THERMO2               13U
#define GPIOD_THERMO3               14U
#define GPIOD_VAC_NCS               15U

#define GPIOE_DBG_TX                0U
#define GPIOE_DBG_RX                1U
#define GPIOE_SPI4_SCK              2U
#define GPIOE_SPI4_NCS              3U
#define GPIOE_LED_RED               4U
#define GPIOE_SPI4_MISO             5U
#define GPIOE_SPI4_MOSI             6U
#define GPIOE_LED_GREEN             7U
#define GPIOE_PWM_AL                8U
#define GPIOE_PWM_AH                9U
#define GPIOE_PWM_BL                10U
#define GPIOE_PWM_BH                11U
#define GPIOE_PWM_CL                12U
#define GPIOE_PWM_CH                13U
#define GPIOE_PWM_DL                14U
#define GPIOE_PWM_DH                15U

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
#define LINE_VDC_NCS                PAL_LINE(GPIOA, 9U)
#define LINE_IABC_NCS               PAL_LINE(GPIOB, 7U)
#define LINE_LED_RED                PAL_LINE(GPIOE, 4U)
#define LINE_LED_GREEN              PAL_LINE(GPIOE, 7U)
#define LINE_RES_NSTANDBY           PAL_LINE(GPIOA, 3U)
#define LINE_RES_NFAULT             PAL_LINE(GPIOA, 10U)
#define LINE_GATE_EN                PAL_LINE(GPIOB, 11U)
#define LINE_PWM_HWBREAK            PAL_LINE(GPIOB, 10U)


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
 * PA2  - TEMP_RES0                 (analog).
 * PA3  - RES_STANDBY               (output pushpull minimum).
 * PA4  - RES_REF                   (analog output)
 * PA5  - SPI1_SCK                  (alternate 5).
 * PA6  - SPI1_MISO                 (alternate 5).
 * PA7  - SPI1_MOSI                 (alternate 5).
 * PA8  - AIN_TCOOLING              (analog).
 * PA9  - VDC_NCS                   (output pushpull minimum).
 * PA10 - RES_NFAULT                (input pulldown).
 * PA11 - USB_DM                    (analog).
 * PA12 - USB_DP                    (analog).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - OVP_C                     (input pulldown).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_AIN_IA       ) |     \
                                     PIN_MODE_ANALOG(GPIOA_AIN_IB       ) |     \
                                     PIN_MODE_ANALOG(GPIOA_TEMP_RES0    ) |     \
                                     PIN_MODE_OUTPUT(GPIOA_REF_NSTANDBY ) |     \
                                     PIN_MODE_ANALOG(GPIOA_RES_REF      ) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK  ) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO ) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI ) |     \
                                     PIN_MODE_ANALOG(GPIOA_AIN_TCOOLING ) |     \
                                     PIN_MODE_OUTPUT(GPIOA_VDC_NCS      ) |     \
                                     PIN_MODE_INPUT(GPIOA_RES_NFAULT    ) |     \
                                     PIN_MODE_ANALOG(GPIOA_USB_DM       ) |     \
                                     PIN_MODE_ANALOG(GPIOA_USB_DP       ) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TMS  ) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_CLK  ) |     \
                                     PIN_MODE_INPUT(GPIOA_OVP_C         ))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_AIN_IA       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AIN_IB       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TEMP_RES0    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_REF_NSTANDBY ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RES_REF      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AIN_TCOOLING ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_VDC_NCS      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RES_NFAULT   ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TMS     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_CLK     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OVP_C        ))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_AIN_IA       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_AIN_IB       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_TEMP_RES0    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_REF_NSTANDBY ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_RES_REF      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_SPI1_SCK     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_SPI1_MISO    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_SPI1_MOSI    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_AIN_TCOOLING ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_VDC_NCS      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_RES_NFAULT   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_USB_DM       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_USB_DP       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_JTAG_TMS     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_JTAG_CLK     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOA_OVP_C        ))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_AIN_IA       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_AIN_IB       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_TEMP_RES0    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_REF_NSTANDBY ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_RES_REF      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_SCK     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MISO    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MOSI    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_AIN_TCOOLING ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_VDC_NCS      ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_RES_NFAULT   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DM       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DP       ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_JTAG_TMS     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_JTAG_CLK     ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_OVP_C        ))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_AIN_IA       ) |         \
                                     PIN_ODR_LOW(GPIOA_AIN_IB       ) |         \
                                     PIN_ODR_LOW(GPIOA_TEMP_RES0    ) |         \
                                     PIN_ODR_LOW(GPIOA_REF_NSTANDBY ) |         \
                                     PIN_ODR_LOW(GPIOA_RES_REF      ) |         \
                                     PIN_ODR_LOW(GPIOA_SPI1_SCK     ) |         \
                                     PIN_ODR_LOW(GPIOA_SPI1_MISO    ) |         \
                                     PIN_ODR_LOW(GPIOA_SPI1_MOSI    ) |         \
                                     PIN_ODR_LOW(GPIOA_AIN_TCOOLING ) |         \
                                     PIN_ODR_LOW(GPIOA_VDC_NCS      ) |         \
                                     PIN_ODR_LOW(GPIOA_RES_NFAULT   ) |         \
                                     PIN_ODR_LOW(GPIOA_USB_DM       ) |         \
                                     PIN_ODR_LOW(GPIOA_USB_DP       ) |         \
                                     PIN_ODR_LOW(GPIOA_JTAG_TMS     ) |         \
                                     PIN_ODR_LOW(GPIOA_JTAG_CLK     ) |         \
                                     PIN_ODR_LOW(GPIOA_OVP_C        ))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_AIN_IA       , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_AIN_IB       , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_TEMP_RES0    , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_REF_NSTANDBY , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_RES_REF      , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK     , 5U) |     \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO    , 5U) |     \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI    , 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_AIN_TCOOLING , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_VDC_NCS      , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_RES_NFAULT   , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_USB_DM       , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_USB_DP       , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_JTAG_TMS     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_JTAG_CLK     , 0U) |     \
                                     PIN_AFIO_AF(GPIOA_OVP_C        , 0U))

/*
 * GPIOB setup:
 *
 * PB0  - TEMP_MOTOR                (analog).
 * PB1  - AIN_IC                    (analog).
 * PB2  - OCP_B                     (input pulldown).
 * PB3  - TRACESWO                  (alternate 0).
 * PB4  - PIN4                      (analog).
 * PB5  - FDCAN2_RX                 (alternate 9).
 * PB6  - FDCAN2_TX                 (alternate 9).
 * PB7  - IABC_NCS                  (output pushpull minimum).
 * PB8  - PIN8                      (analog).
 * PB9  - OCP_HVDC                  (input pulldown).
 * PB10 - PWM_HWBREAK               (alternate 12).
 * PB11 - GATE_EN                   (output pushpull minimum).
 * PB12 - OVP_B                     (input pulldown).
 * PB13 - SPI2_SCK                  (alternate 5).
 * PB14 - SPI2_MISO                 (alternate 5).
 * PB15 - SPI2_MOSI                 (alternate 5).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_TEMP_MOTOR   ) |  \
                                     PIN_MODE_ANALOG(GPIOB_AIN_IC       ) |  \
                                     PIN_MODE_INPUT(GPIOB_OCP_B         ) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_TRACESWO  ) |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN4         ) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_FDCAN2_RX ) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_FDCAN2_TX ) |  \
                                     PIN_MODE_OUTPUT(GPIOB_IABC_NCS     ) |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN8         ) |  \
                                     PIN_MODE_INPUT(GPIOB_OCP_HVDC      ) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_PWM_HWBREAK)|  \
                                     PIN_MODE_OUTPUT(GPIOB_GATE_EN      ) |  \
                                     PIN_MODE_INPUT(GPIOB_OVP_B         ) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK  ) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO ) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI ))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_TEMP_MOTOR   ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AIN_IC       ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OCP_B        ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_TRACESWO     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4         ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FDCAN2_RX    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FDCAN2_TX    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IABC_NCS     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8         ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OCP_HVDC     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PWM_HWBREAK  ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GATE_EN      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OVP_B        ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_SCK     ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MISO    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MOSI    ))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOB_TEMP_MOTOR   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_AIN_IC       ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_OCP_B        ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_TRACESWO     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN4         ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_FDCAN2_RX    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_FDCAN2_TX    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_IABC_NCS     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN8         ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_OCP_HVDC     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_PWM_HWBREAK  ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_GATE_EN      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_OVP_B        ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_SPI2_SCK     ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_SPI2_MISO    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_SPI2_MOSI    ))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_TEMP_MOTOR   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_AIN_IC       ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_OCP_B        ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_TRACESWO     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN4         ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_FDCAN2_RX    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_FDCAN2_TX    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_IABC_NCS     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8         ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_OCP_HVDC     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PWM_HWBREAK  ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_GATE_EN      ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_OVP_B        ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_SCK     ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MISO    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MOSI    ))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_TEMP_MOTOR   ) |  \
                                     PIN_ODR_LOW(GPIOB_AIN_IC       ) |  \
                                     PIN_ODR_LOW(GPIOB_OCP_B        ) |  \
                                     PIN_ODR_LOW(GPIOB_TRACESWO     ) |  \
                                     PIN_ODR_LOW(GPIOB_PIN4         ) |  \
                                     PIN_ODR_LOW(GPIOB_FDCAN2_RX    ) |  \
                                     PIN_ODR_LOW(GPIOB_FDCAN2_TX    ) |  \
                                     PIN_ODR_LOW(GPIOB_IABC_NCS     ) |  \
                                     PIN_ODR_LOW(GPIOB_PIN8         ) |  \
                                     PIN_ODR_LOW(GPIOB_OCP_HVDC     ) |  \
                                     PIN_ODR_LOW(GPIOB_PWM_HWBREAK  ) |  \
                                     PIN_ODR_LOW(GPIOB_GATE_EN      ) |  \
                                     PIN_ODR_LOW(GPIOB_OVP_B        ) |  \
                                     PIN_ODR_LOW(GPIOB_SPI2_SCK     ) |  \
                                     PIN_ODR_LOW(GPIOB_SPI2_MISO    ) |  \
                                     PIN_ODR_LOW(GPIOB_SPI2_MOSI    ))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_TEMP_MOTOR   , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_AIN_IC       , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_OCP_B        , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_TRACESWO     , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN4         , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_FDCAN2_RX    , 9U) |          \
                                     PIN_AFIO_AF(GPIOB_FDCAN2_TX    , 9U) |          \
                                     PIN_AFIO_AF(GPIOB_IABC_NCS     , 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8         , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_OCP_HVDC     , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PWM_HWBREAK  , 12U)|          \
                                     PIN_AFIO_AF(GPIOB_GATE_EN      , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_OVP_B        , 0U) |          \
                                     PIN_AFIO_AF(GPIOB_SPI2_SCK     , 5U) |          \
                                     PIN_AFIO_AF(GPIOB_SPI2_MISO    , 5U) |          \
                                     PIN_AFIO_AF(GPIOB_SPI2_MOSI    , 5U))

/*
 * GPIOC setup:
 *
 * PC0  - RES_SIN                   (analog).
 * PC1  - RES_COS                   (analog).
 * PC2  - AIN_24V                   (analog).
 * PC3  - AIN_15V                   (analog).
 * PC4  - OCP_A                     (input pulldown).
 * PC5  - OVP_A                     (input pulldown).
 * PC6  - ENC_A                     (alternate 2).
 * PC7  - ENC_B                     (alternate 2).
 * PC8  - I2C3_SCL                  (alternate 8).
 * PC9  - I2C3_SDA                  (alternate 8).
 * PC10 - SPI3_SCK                  (alternate 6).
 * PC11 - SPI3_MISO                 (alternate 6).
 * PC12 - SPI3_MOSI                 (alternate 6).
 * PC13 - OVP_HVDC                  (input pulldown).
 * PC14 - OSC32_IN                  (analog).
 * PC15 - OSC32_OUT                 (analog).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_RES_SIN    ) |     \
                                     PIN_MODE_ANALOG(GPIOC_RES_COS    ) |     \
                                     PIN_MODE_ANALOG(GPIOC_AIN_24V    ) |     \
                                     PIN_MODE_ANALOG(GPIOC_AIN_15V    ) |     \
                                     PIN_MODE_INPUT(GPIOC_OCP_A       ) |     \
                                     PIN_MODE_INPUT(GPIOC_OVP_A       ) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_ENC_A   ) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_ENC_B   ) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_I2C3_SCL) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_I2C3_SDA) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_SCK) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_MISO) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_MOSI) |    \
                                     PIN_MODE_INPUT(GPIOC_OVP_HVDC    ) |     \
                                     PIN_MODE_ANALOG(GPIOC_OSC32_IN   ) |     \
                                     PIN_MODE_ANALOG(GPIOC_OSC32_OUT  ))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_RES_SIN    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RES_COS    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AIN_24V    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AIN_15V    ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OCP_A      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OVP_A      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENC_A      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENC_B      ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_I2C3_SCL   ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_I2C3_SDA   ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_SCK   ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_MISO  ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_MOSI  ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OVP_HVDC   ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN   ) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT  ))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_RES_SIN    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_RES_COS    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_AIN_24V    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_AIN_15V    ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_OCP_A      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_OVP_A      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_ENC_A      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_ENC_B      ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_I2C3_SCL   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_I2C3_SDA   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_SPI3_SCK   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_SPI3_MISO  ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_SPI3_MOSI  ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_OVP_HVDC   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_IN   ) |  \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_OUT  ))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_RES_SIN    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_RES_COS    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_AIN_24V    ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_AIN_15V    ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOC_OCP_A      ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOC_OVP_A      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_ENC_A      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_ENC_B      ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_I2C3_SCL   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_I2C3_SDA   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_SPI3_SCK   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_SPI3_MISO  ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_SPI3_MOSI  ) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOC_OVP_HVDC   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN   ) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT  ))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_RES_SIN    ) |         \
                                     PIN_ODR_LOW(GPIOC_RES_COS    ) |         \
                                     PIN_ODR_LOW(GPIOC_AIN_24V    ) |         \
                                     PIN_ODR_LOW(GPIOC_AIN_15V    ) |         \
                                     PIN_ODR_LOW(GPIOC_OCP_A      ) |         \
                                     PIN_ODR_LOW(GPIOC_OVP_A      ) |         \
                                     PIN_ODR_LOW(GPIOC_ENC_A      ) |         \
                                     PIN_ODR_LOW(GPIOC_ENC_B      ) |         \
                                     PIN_ODR_LOW(GPIOC_I2C3_SCL   ) |         \
                                     PIN_ODR_LOW(GPIOC_I2C3_SDA   ) |         \
                                     PIN_ODR_LOW(GPIOC_SPI3_SCK   ) |         \
                                     PIN_ODR_LOW(GPIOC_SPI3_MISO  ) |         \
                                     PIN_ODR_LOW(GPIOC_SPI3_MOSI  ) |         \
                                     PIN_ODR_LOW(GPIOC_OVP_HVDC   ) |         \
                                     PIN_ODR_LOW(GPIOC_OSC32_IN   ) |         \
                                     PIN_ODR_LOW(GPIOC_OSC32_OUT  ))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_RES_SIN    , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_RES_COS    , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_AIN_24V    , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_AIN_15V    , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_OCP_A      , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_OVP_A      , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_ENC_A      , 2U) |    \
                                     PIN_AFIO_AF(GPIOC_ENC_B      , 2U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_I2C3_SCL   , 8U) |    \
                                     PIN_AFIO_AF(GPIOC_I2C3_SDA   , 8U) |    \
                                     PIN_AFIO_AF(GPIOC_SPI3_SCK   , 6U) |    \
                                     PIN_AFIO_AF(GPIOC_SPI3_MISO  , 6U) |    \
                                     PIN_AFIO_AF(GPIOC_SPI3_MOSI  , 6U) |    \
                                     PIN_AFIO_AF(GPIOC_OVP_HVDC   , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN   , 0U) |    \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT  , 0U))

/*
 * GPIOD setup:
 *
 * PD0  - FDCAN1_RX                 (alternate 9)
 * PD1  - FDCAN1_TX                 (alternate 9).
 * PD2  - ENC_Z                     (alternate 2).
 * PD3  - OCP_C                     (input pulldown).
 * PD4  - RS485_DE                  (alternate 7).
 * PD5  - RS485_TX                  (alternate 7).
 * PD6  - RS485_RX                  (alternate 7).
 * PD7  - PIN7                      (analog).
 * PD8  - TEMP_RES1                 (analog).
 * PD9  - THERMO4                   (analog).
 * PD10 - AIN_IDC                   (analog).
 * PD11 - AIN_VDC                   (analog).
 * PD12 - THERMO1                   (analog).
 * PD13 - THERMO2                   (analog).
 * PD14 - THERMO3                   (analog).
 * PD15 - VAC_NCS                   (output pushpull minimum).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_FDCAN1_RX ) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_FDCAN1_TX ) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_ENC_Z     ) |       \
                                     PIN_MODE_INPUT(GPIOD_OCP_C     ) |           \
                                     PIN_MODE_ALTERNATE(GPIOD_RS485_DE  ) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_RS485_TX  ) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_RS485_RX  ) |       \
                                     PIN_MODE_ANALOG(GPIOD_PIN7      ) |          \
                                     PIN_MODE_ANALOG(GPIOD_TEMP_RES1 ) |          \
                                     PIN_MODE_ANALOG(GPIOD_THERMO4   ) |          \
                                     PIN_MODE_ANALOG(GPIOD_AIN_IDC   ) |          \
                                     PIN_MODE_ANALOG(GPIOD_AIN_VDC   ) |          \
                                     PIN_MODE_ANALOG(GPIOD_THERMO1   ) |          \
                                     PIN_MODE_ANALOG(GPIOD_THERMO2   ) |          \
                                     PIN_MODE_ANALOG(GPIOD_THERMO3   ) |          \
                                     PIN_MODE_OUTPUT(GPIOD_VAC_NCS   ))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_FDCAN1_RX ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FDCAN1_TX ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_ENC_Z     ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OCP_C     ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_RS485_DE  ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_RS485_TX  ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_RS485_RX  ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_TEMP_RES1 ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_THERMO4   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_AIN_IDC   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_AIN_VDC   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_THERMO1   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_THERMO2   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_THERMO3   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_VAC_NCS   ))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOD_FDCAN1_RX ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_FDCAN1_TX ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_ENC_Z     ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_OCP_C     ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_RS485_DE  ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_RS485_TX  ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_RS485_RX  ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN7      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_TEMP_RES1 ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_THERMO4   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_AIN_IDC   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_AIN_VDC   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_THERMO1   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_THERMO2   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_THERMO3   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_VAC_NCS   ))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_FDCAN1_RX ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_FDCAN1_TX ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_ENC_Z     ) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_OCP_C     ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_RS485_DE  ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_RS485_TX  ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_RS485_RX  ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_TEMP_RES1 ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_THERMO4   ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_AIN_IDC   ) |       \
                                     PIN_PUPDR_PULLUP(GPIOD_AIN_VDC   ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_THERMO1   ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_THERMO2   ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_THERMO3   ) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_VAC_NCS   ))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_FDCAN1_RX ) |              \
                                     PIN_ODR_LOW(GPIOD_FDCAN1_TX ) |              \
                                     PIN_ODR_LOW(GPIOD_ENC_Z     ) |              \
                                     PIN_ODR_LOW(GPIOD_OCP_C     ) |              \
                                     PIN_ODR_LOW(GPIOD_RS485_DE  ) |              \
                                     PIN_ODR_LOW(GPIOD_RS485_TX  ) |              \
                                     PIN_ODR_LOW(GPIOD_RS485_RX  ) |              \
                                     PIN_ODR_LOW(GPIOD_PIN7      ) |              \
                                     PIN_ODR_LOW(GPIOD_TEMP_RES1 ) |              \
                                     PIN_ODR_LOW(GPIOD_THERMO4   ) |              \
                                     PIN_ODR_LOW(GPIOD_AIN_IDC   ) |              \
                                     PIN_ODR_LOW(GPIOD_AIN_VDC   ) |              \
                                     PIN_ODR_LOW(GPIOD_THERMO1   ) |              \
                                     PIN_ODR_LOW(GPIOD_THERMO2   ) |              \
                                     PIN_ODR_LOW(GPIOD_THERMO3   ) |              \
                                     PIN_ODR_LOW(GPIOD_VAC_NCS   ))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_FDCAN1_RX , 9U) |          \
                                     PIN_AFIO_AF(GPIOD_FDCAN1_TX , 9U) |          \
                                     PIN_AFIO_AF(GPIOD_ENC_Z     , 2U) |          \
                                     PIN_AFIO_AF(GPIOD_OCP_C     , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_RS485_DE  , 7U) |          \
                                     PIN_AFIO_AF(GPIOD_RS485_TX  , 7U) |          \
                                     PIN_AFIO_AF(GPIOD_RS485_RX  , 8U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN7      , 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_TEMP_RES1 , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_THERMO4   , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_AIN_IDC   , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_AIN_VDC   , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_THERMO1   , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_THERMO2   , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_THERMO3   , 0U) |          \
                                     PIN_AFIO_AF(GPIOD_VAC_NCS   , 0U))

/*
 * GPIOE setup:
 *
 * PE0  - DBG_TX                    (alternate 7).
 * PE1  - DBG_RX                    (alternate 7).
 * PE2  - SPI4_SCK                  (alternate 5).
 * PE3  - SPI4_NCS                  (alternate 5).
 * PE4  - LED_RED                   (output pushpull minimum).
 * PE5  - SPI4_MISO                 (alternate 5).
 * PE6  - SPI4_MOSI                 (alternate 5).
 * PE7  - LED_GREEN                 (output pushpull minimum).
 * PE8  - PWM_AL                    (alternate 2).
 * PE9  - PWM_AH                    (alternate 2).
 * PE10 - PWM_BL                    (alternate 2).
 * PE11 - PWM_BH                    (alternate 2).
 * PE12 - PWM_CL                    (alternate 2).
 * PE13 - PWM_CH                    (alternate 2).
 * PE14 - PWM_DL                    (alternate 2).
 * PE15 - PWM_DH                    (alternate 2).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_DBG_TX      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_DBG_RX      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_SCK    ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_NCS    ) |       \
                                     PIN_MODE_OUTPUT(GPIOE_LED_RED     ) |          \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_MISO   ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_MOSI   ) |       \
                                     PIN_MODE_OUTPUT(GPIOE_LED_GREEN   ) |          \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_AL      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_AH      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_BL      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_BH      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_CL      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_CH      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_DL      ) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_DH      ))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_DBG_TX      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DBG_RX      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_SCK    ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_NCS    ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LED_RED     ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_MISO   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_MOSI   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LED_GREEN   ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_AL      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_AH      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_BL      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_BH      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_CL      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_CH      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_DL      ) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_DH      ))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOE_DBG_TX      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_DBG_RX      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_SPI4_SCK    ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_SPI4_NCS    ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_LED_RED     ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_SPI4_MISO   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_SPI4_MOSI   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_LED_GREEN   ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWM_AL      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWM_AH      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWM_BL      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWM_BH      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWM_CL      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWM_CH      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWM_DL      ) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWM_DH      ))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_DBG_TX      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_DBG_RX      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_SPI4_SCK    ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_SPI4_NCS    ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_LED_RED     ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_SPI4_MISO   ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_SPI4_MOSI   ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_LED_GREEN   ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_AL      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_AH      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_BL      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_BH      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_CL      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_CH      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_DL      ) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_DH      ))
#define VAL_GPIOE_ODR               (PIN_ODR_LOW(GPIOE_DBG_TX      ) |              \
                                     PIN_ODR_LOW(GPIOE_DBG_RX      ) |              \
                                     PIN_ODR_LOW(GPIOE_SPI4_SCK    ) |              \
                                     PIN_ODR_LOW(GPIOE_SPI4_NCS    ) |              \
                                     PIN_ODR_LOW(GPIOE_LED_RED     ) |              \
                                     PIN_ODR_LOW(GPIOE_SPI4_MISO   ) |              \
                                     PIN_ODR_LOW(GPIOE_SPI4_MOSI   ) |              \
                                     PIN_ODR_LOW(GPIOE_LED_GREEN   ) |              \
                                     PIN_ODR_LOW(GPIOE_PWM_AL      ) |              \
                                     PIN_ODR_LOW(GPIOE_PWM_AH      ) |              \
                                     PIN_ODR_LOW(GPIOE_PWM_BL      ) |              \
                                     PIN_ODR_LOW(GPIOE_PWM_BH      ) |              \
                                     PIN_ODR_LOW(GPIOE_PWM_CL      ) |              \
                                     PIN_ODR_LOW(GPIOE_PWM_CH      ) |              \
                                     PIN_ODR_LOW(GPIOE_PWM_DL      ) |              \
                                     PIN_ODR_LOW(GPIOE_PWM_DH      ))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_DBG_TX      , 7U) |          \
                                     PIN_AFIO_AF(GPIOE_DBG_RX      , 7U) |          \
                                     PIN_AFIO_AF(GPIOE_SPI4_SCK    , 5U) |          \
                                     PIN_AFIO_AF(GPIOE_SPI4_NCS    , 5U) |          \
                                     PIN_AFIO_AF(GPIOE_LED_RED     , 0U) |          \
                                     PIN_AFIO_AF(GPIOE_SPI4_MISO   , 5U) |          \
                                     PIN_AFIO_AF(GPIOE_SPI4_MOSI   , 5U) |          \
                                     PIN_AFIO_AF(GPIOE_LED_GREEN   , 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PWM_AL      , 2U) |          \
                                     PIN_AFIO_AF(GPIOE_PWM_AH      , 2U) |          \
                                     PIN_AFIO_AF(GPIOE_PWM_BL      , 2U) |          \
                                     PIN_AFIO_AF(GPIOE_PWM_BH      , 2U) |          \
                                     PIN_AFIO_AF(GPIOE_PWM_CL      , 2U) |          \
                                     PIN_AFIO_AF(GPIOE_PWM_CH      , 2U) |          \
                                     PIN_AFIO_AF(GPIOE_PWM_DL      , 2U) |          \
                                     PIN_AFIO_AF(GPIOE_PWM_DH      , 2U))

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