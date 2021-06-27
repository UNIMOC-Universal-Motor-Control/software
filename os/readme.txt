*****************************************************************************
*** Files Organization                                                    ***
*****************************************************************************

--{root}                        - ChibiOS directory.
  +--readme.txt                 - This file.
  +--documentation.html         - Shortcut to the web documentation page.
  +--license.txt                - GPL license text.
  +--demos/                     - Demo projects, one directory per platform.
  +--docs/                      - Documentation.
  |  +--common/                 - Documentation common build resources.
  |  +--hal/                    - Builders for HAL.
  |  +--nil/                    - Builders for NIL.
  |  +--rt/                     - Builders for RT.
  +--ext/                       - External libraries, not part of ChibiOS.
  +--os/                        - ChibiOS components.
  |  +--common/                 - Shared OS modules.
  |  |  +--abstractions/        - API emulator wrappers.
  |  |  |  +--cmsis_os/         - CMSIS OS emulation layer for RT.
  |  |  |  +--nasa_osal/        - NASA Operating System Abstraction Layer.
  |  |  +--ext/                 - Vendor files used by the OS.
  |  |  +--ports/               - RTOS ports usable by both RT and NIL.
  |  |  +--startup/             - Startup support.
  |  +--ex/                     - EX component.
  |  |  +--dox/                 - EX documentation resources.
  |  |  +--include/             - EX header files.
  |  |  +--devices /            - EX complex drivers.
  |  +--hal/                    - HAL component.
  |  |  +--boards/              - HAL board support files.
  |  |  +--dox/                 - HAL documentation resources.
  |  |  +--include/             - HAL high level headers.
  |  |  +--lib/                 - HAL libraries.
  |  |  |  +--complex/          - HAL collection of complex drivers.
  |  |  |  |  +--mfs/           - HAL managed flash storage driver.
  |  |  |  |  +--serial_nor/    - HAL managed flash storage driver.
  |  |  |  +--fallback/         - HAL fall back software drivers.
  |  |  |  +--peripherals/      - HAL peripherals interfaces.
  |  |  |  +--streams/          - HAL streams.
  |  |  +--osal/                - HAL OSAL implementations.
  |  |  |  +--lib/              - HAL OSAL common modules.
  |  |  +--src/                 - HAL high level source.
  |  |  +--ports/               - HAL ports.
  |  |  +--templates/           - HAL driver template files.
  |  |     +--osal/             - HAL OSAL templates.
  |  +--oslib/                  - RTOS modules usable by both RT and NIL.
  |  |  +--include/             - OSLIB high level headers.
  |  |  +--src/                 - OSLIB high level source.
  |  |  +--templates/           - OSLIB configuration template files.
  |  +--nil/                    - NIL RTOS component.
  |  |  +--dox/                 - NIL documentation resources.
  |  |  +--include/             - NIL high level headers.
  |  |  +--src/                 - NIL high level source.
  |  |  +--templates/           - NIL configuration template files.
  |  +--rt/                     - RT RTOS component.
  |  |  +--dox/                 - RT documentation resources.
  |  |  +--include/             - RT high level headers.
  |  |  +--src/                 - RT high level source.
  |  |  +--templates/           - RT configuration template files.
  |  +--various/                - Various portable support files.
  +--test/                      - Kernel test suite source code.
  |  +--lib/                    - Portable test engine.
  |  +--hal/                    - HAL test suites.
  |  |  +--testbuild/           - HAL build test and MISRA check.
  |  +--nil/                    - NIL test suites.
  |  |  +--testbuild/           - NIL build test and MISRA check.
  |  +--rt/                     - RT test suites.
  |  |  +--testbuild/           - RT build test and MISRA check.
  |  |  +--coverage/            - RT code coverage project.
  +--testex/                    - EX integration test demos.
  +--testhal/                   - HAL integration test demos.

*****************************************************************************
*** Releases and Change Log                                               ***
*****************************************************************************

*** 21.6.0 ***
- NEW: Simplified test XML schema.
- NEW: Simplified interface between ports and RT/NIL.
- NEW: Removed duplicated files for ARM ports: chtypes.h.
- NEW: Removed duplicated files for all ports: chcore_timer.h.
- NEW: STM32 ADCv3, USARTv2, USARTv3, USBv1 updated for dynamic clocking.
- NEW: Improved PWR settings for STM32G0, STM32G4 and STM32L4+.
- NEW: Dynamic support implemented for STM32G0, STM32G4, STM32L4+, STM32WL.
- NEW: Dynamic clocks support in HAL.
- NEW: Reload feature added to RT virtual timers.
- NEW: Upgraded the clock initialization for STM32G0, STM32L4 and STM32L4++
       to the new standard (started with STM32G4).
- NEW: Added support for STM32L422.
- NEW: Added support for STM32WLx5.
- NEW: Added initial support for RP2040.
- NEW: Added support for STM32WB55.
- NEW: Added chscanf() and buffered streams, contributed by Alex Lewontin.
- NEW: Added option to LWIP bindings to use memory pools instead of heap
       allocator.
- NEW: Added MACv2 driver for STM32H7xx.
- NEW: Added support for UART9 and UART10 in STM32 USARTv1 drivers.
- NEW: Added board support for ST_STM32G474RE_DISCOVERY_DPOW1, added demo.
- NEW: Improved the STM32G4xx clock initialization to use the shared
       mini-drivers in STM32/LLD/RCCv1, code is greatly simplified.
- NEW: Updated STM32F4xx platform with new IRQ handling, enabled the missing
       timers.
- NEW: Added mcuconf.h updater for STM32F401, STM32F410, STM32F411, STM32F412,
       F427, F429, F437, F439, F446, F469, F479.
- NEW: SIO STM32 implementation for USARTs without FIFO in STM32/LLD/USARTv2,
       implementation with FIFO in STM32/LLD/USARTv3.
- NEW: Updated SIO driver model to support more use cases.
- NEW: Simplified USART units use collision detection in STM32 HAL, now it is
       done like for TIM units for consistency.
- NEW: Identification macros such as__CHIBIOS_RT__ are now prefixed by
       two underscores.
- NEW: Support for 3 analog watchdogs in ADCv3 (STM32F3, L4, L4+, G4).
- NEW: Support for 3 analog watchdogs in ADCv5 (STM32G0).
- NEW: Stand-alone ARMv8-M-ML-TZ port.
- NEW: Stand-alone ARMv8-M-ML port.
- NEW: Stand-alone ARMv7-M port.
- NEW: Stand-alone ARMv6-M port.
- NEW: Merged RT7.
- NEW: New API in RT for high resolution monotonic time stamps.
- NEW: Updated FatFS to version 0.14.
- NEW: Added a new setting to STM32 USBv1 allowing for some clock deviation
       from 48MHz. Renamed setting USB_HOST_WAKEUP_DURATION to
       STM32_USB_HOST_WAKEUP_DURATION for consistency.
- NEW: Added entry for STM32L475 in STM32L4 registry header, updated all
       configuration files.
- NEW: Updated CMSIS headers for STM32F7, G0, G4, H7, L0, L4, L4+.
- NEW: Implemented tickless mode on ADuCM36x family
- NEW: STM32 ICU driver now allows to setup the ARR register in the
       configuration structure, the default value should be 0xFFFFFFFFU.
- NEW: More time conversion macros added to HAL OSAL.
- NEW: Updated debug tools to be independent from the toolchain position:
       they now rely on the environment variable CHIBISTUDIO.
- NEW: Mail Queues test implementation in CMSIS RTOS wrapper.
- NEW: Added dynamic reconfiguration API to lwIP bindings.
- LIB: Reorganized static initializer macros.
- NIL: Reorganized static initializer macros.
- RT:  Reorganized static initializer macros.
- RT:  Relocated the "ctx" field in the thread structure in order to save
       some RAM, it caused unused space in the "ch" variable.
- EX:  Implemented cache handling in the ADXL355 device driver.
- EX:  Added support for ADXL355 Low Noise, Low Drift, Low Power, 3-Axis
       MEMS Accelerometers.
- NEW: Safer messages mechanism for sandboxes.
- NEW: Added latency measurement test application.
- FIX: Fixed STM32 SDMMCv2 driver invalid initial clock settings (bug #1160)
       (backported to 20.3.4).
- FIX: Fixed wrong wait states calculation in STM32G4xx, insufficient
       boost settings (bug #1159)(backported to 20.3.4).
- FIX: Fixed warning in STM32 ADCv4 (bug #1158)
       (backported to 20.3.4)(backported to 19.1.5).
- FIX: Fixed wrong check on HAL_USE_RTC in STM32G4 clock initialization
       (bug #1157)(backported to 20.3.4).
- FIX: Fixed wrong checks related to PLLSAI2 on L4 and L4+ (bug #1156)
       (backported to 20.3.4)(backported to 19.1.5).
- FIX: Fixed STM32G431 DMA defines error (bug #1155)
       (backported to 20.3.4).
- FIX: Fixed errors in STM32L4xx registry (bug #1154)
       (backported to 20.3.4)(backported to 19.1.5).
- FIX: Fixed QUADSPI errata fix applied to all platforms (bug #1153)
       (backported to 20.3.4)(backported to 19.1.5).
- FIX: Fixed (again) LPUART1 support for STM32H7xx (bug #1113)
       (backported to 20.3.4).
- FIX: Fixed wrong errors handling in STM32 ADC drivers (bug #1152).
- FIX: Fixed wrong behavior in Serial-USB driver (bug #1151)
       (backported to 20.3.4)(backported to 19.1.5).
- FIX: Fixed L0x2 series DAC not allocated in registry (bug #1150)
       (backported to 20.3.4)(backported to 19.1.5).
- FIX: Fixed inconsistent naming of SAI DMAMUX defines for STM32H7 (bug #1149)
       (backported to 20.3.4).
- FIX: Fixed TIM register layout difference for STM32G4 series (bug #1148)
       (backported to 20.3.4).
- FIX: Fixed STM32 QUADSPI driver problem when used with DMAv2 (bug #1147)
       (backported to 20.3.4).
- FIX: Fixed incorrect IRQ vector for PVM (bug #1146)
       (backported to 20.3.4).
- FIX: Fixed missing STM32F765 from registry (bug #1145)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed wrong macro check on STM32 SPIv3 (bug #1144)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed wrong check on STM32 TIM9 timer (bug #1143)
       (backported to 20.3.3).
- FIX: Fixed FAT time problem in RTC driver (bug #1142)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed Heap allocation of aligned FIFO objects in chFactory (bug #1141)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed chsnprintf() sign mode/filler mode conflict (bug #1140)
       (backported to 20.3.3).
- FIX: Fixed GCC 10 causes warning in factory module (bug #1139)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed STM32H7xx Missing CRC RCC macros (bug #1137)
       (backported to 20.3.3).
- FIX: Fixed STM32L0x wrong ISR names for USART 4 and 5 (bug #1136)
       (backported to 20.3.3).
- FIX: Fixed OTG_FS error on STM32H7 (bug #1135)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed compile error of STM32 CRYPv1 driver when DMAMU is present
       (bug #1134)(backported to 20.3.3).
- FIX: Fixed moved define into hal_wspi_lld.c (bug #1133)
       (backported to 20.3.3).
- FIX: Fixed various bugs in MDMAv1 driver (bug #1132)
       (backported to 20.3.3).
- FIX: Fixed wrong check on LSI on all STM32 platforms (bug #1131)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed OSAL does not allow 64 bits resolution values (bug #1128)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed wrong SDMMC RCC macros for STM32H7xx (bug #1127)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed STM32 ADCv3 hanging on initialization (bug #1126)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed I2S-related problems in STM32F4xx registry (bug #1124)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed STM32 EXTIv1 driver unable to enable/disable fixed lines
       (bug #1123)(backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed incorrect STM32 iWDG initialization in windowed mode (bug #1122)
       (backported to 20.3.3)(backported to 19.1.5).
- FIX: Fixed ignored HSIDIV setting on STM32G0xx (bug #1121)
       (backported to 20.3.3).
- FIX: Fixed incorrect variable name in recursive mutex handling (bug #1119).
- FIX: Fixed several problems in STM32 ADCv4 (bug #1116)
       (backported to 20.3.2).
- FIX: Fixed STM32 QSPI errata workaround (bug #1117)
       (backported to 20.3.2).
- FIX: Fixed wrong condition in STM32 BDMAv1 driver (bug #1115)
       (backported to 20.3.2).
- FIX: Fixed HSI48 not getting enabled on STM32H7 (bug #1114)
       (backported to 20.3.2).
- FIX: Fixed LPUART1 support for STM32H7xx (bug #1113)
       (backported to 20.3.2).
- FIX: Fixed wrong sector count in EFL driver for L4+ dual bank configuration 
       (bug #1112)(backported to 20.3.2).
- FIX: Fixed wrong preprocessor checks in STM32 TIMv1 ICU driver (bug #1111)
       (backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed wrong revisions handling in STM32H743 HAL (bug #1110)
       (backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed missing STM32_I2C_BDMA_REQUIRED definition in I2Cv3 driver
       (bug #1109)(backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed wrong definitions in SPC563M board files (bug #1108)
       (backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed cortex-M vectors table alignment problem (bug #1107)
       (backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed extra condition in MAC driver macWaitTransmitDescriptor() function
       (bug #1106)(backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed schedule anomaly when CH_CFG_TIME_QUANTUM is greater than zero
       (bug #1105)(backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed Virtual Timers corner case (bug #1104)
       (backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed GCC6 problem breaks Cortex-M0 port (bug #985)
       (backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed a wrong management of the SPI TX buffer in the ADUCM port 
       (bug #1103)(backported to 20.3.2).
- FIX: Fixed STM32F4 EFL sector bug (bug #1102)
       (backported to 20.3.2).
- FIX: Fixed differences in STM32 EXTI (bug #1101)
       (backported to 20.3.2).
- FIX: Fixed STM32 DACv1 driver regressed because DMA changes (bug #1100)
       (backported to 20.3.2).
- FIX: Fixed STM32L0 missing LPUART IRQ initialization (bug #1099)
       (backported to 20.3.2).
- FIX: Fixed invalid EXTI definitions for STM32L0xx (bug #1098)
       (backported to 20.3.2).
- FIX: Fixed compilation error in file nvic.c (bug #1097)
       (backported to 20.3.2).
- FIX: Fixed STM32_DMAx_CH8_HANDLER not defined for DMAv1 (bug #1096)
       (backported to 20.3.2).
- FIX: Fixed STM32G4 demos compile fails if smart mode is disabled (bug #1094)
       (backported to 20.3.2).
- FIX: Fixed failure in chSemReset() function when counter is equal to MAXINT
       (bug #1093)(backported to 20.3.2)(backported to 19.1.5).
- FIX: Fixed swapped definition in ST_STM32F746G_DISCOVERY board files
       (bug #1092)(backported to 20.3.1)(backported to 19.1.5).
- FIX: Fixed missing symbols in GCC scatter files (bug #1091)
       (backported to 20.3.1).
- FIX: Fixed wrong SAI1 clock selection for STM32G4xx (bug #1090)
       (backported to 20.3.1).
- FIX: Fixed STM32H7xx ADC problem in dual mode (bug #1089)
       (backported to 20.3.1)(backported to 19.1.4).
- FIX: Fixed invalid CHSEL DMA setting in STM32 UART drivers (bug #1088)
       (backported to 20.3.1)(backported to 19.1.4).
- FIX: Fixed wrong arguments for the cacheBufferInvalidate in the STM32 SPI 
       demo (bug #1086)(backported to 20.3.1)(backported to 19.1.4).
- FIX: Fixed sector count incorrect in STM32G07/8 EFL driver (bug #1085)
       (backported to 20.3.1).
- FIX: Fixed sector size incorrect in STM32F413 EFL driver (bug #1084)
       (backported to 20.3.1).
- FIX: Fixed race condition in HAL MAC driver (bug #1083)
       (backported to 20.3.1)(backported to 19.1.4).
- FIX: Fixed STM32H7 compile fails for I2C4 (bug #1082)
       (backported to 20.3.1).
- FIX: Fixed early interrupts enable in ARMv7-M port (bug #1081)
       (backported to 20.3.1).
- FIX: Fixed I2CD4 interrupt vectors are swapped versus I2CD1-I2CD3 (bug #1080)
       (backported to 20.3.1).
- FIX: Fixed incorrect clock check when using PLLSAI1R in ADCv3 (bug #1079)
       (backported to 20.3.1).
- FIX: Fixed missing checks in TIM6 and TIM7 STM32 mini drivers (bug #1078)
       (backported to 20.3.1).
- FIX: Fixed error in EXTIv1 ISRs (bug #1077)
       (backported to 20.3.1).
- FIX: Fixed problem in chMtxUnlockAllS() (bug #1076).
       (backported to 20.3.1)(backported to 19.1.4)(backported to 18.2.3).
