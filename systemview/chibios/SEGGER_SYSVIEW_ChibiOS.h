/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio.
              Copyright (C) 2019 Diego Ismirlian, (dismirlian(at)google's mail)

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
 *  To use:
 *
 *  1)
 *  #include this file at the bottom of chconf.h. You may need to
 *	redefine some of the hooks in chconf.h, for example
 *
 *	    CH_CFG_THREAD_INIT_HOOK => _CH_CFG_THREAD_INIT_HOOK.
 *
 *  If you don't use those hooks in your original code, you may just delete
 *  them from chconf.h
 *
 *
 *  2)
 *  Copy the SEGGER_RTT_Conf.h and SEGGER_SYSVIEW_Conf.h files from the
 *  segger_bindings/example_configurations/ directory to the project's
 *  cfg directory.
 *
 *  You can tune the config files to suit your needs; see the SEGGER RTT and
 *  SystemView documentation for details.
 *
 *
 *  3)
 *  Add the following call to main():
 *    SYSVIEW_ChibiOS_Start(STM32_SYSCLK, STM32_SYSCLK, "I#15=SysTick");
 *
 *  The first parameter, SysFreq, is the time base for all the timestamps. It
 *  must match SEGGER_SYSVIEW_GET_TIMESTAMP in SEGGER_SYSVIEW_Conf.h. By
 *  default, SEGGER_SYSVIEW_GET_TIMESTAMP is configured to use the DWT cycle
 *  counter, so this parameter should match the CPU frequency (eg.
 *  STM32_SYSCLK).
 *
 *  The second parameter, CPUFreq, appears to be just for information.
 *
 *  The third parameter can be used to name the interrupts in the system.
 *  For example, on the Cortex-M*, when using the classic periodic tick for
 *  ChibiOS (CH_CFG_ST_TIMEDELTA == 0), this parameter should include
 *  "I#15=OSTick" (interrupt #15 is the SysTick). When using the tick-less
 *  mode, this parameter could be tuned to show the ISR name of the timer
 *  module used as the OS timer.
 *
 *  Also, you can include all other interrupts in this configuration string
 *  (eg. "I#15=OSTick,I#54=USART2").
 *
 *  See the SystemView documentation for more details.
 *
 *
 *  4)
 *  Copy the file SYSVIEW_ChibiOS.txt (in the segger_bindings directory) to
 *  the following directory:
 *
 *    Path\to\SystemView\Description\
 *
 *  This will allow SystemView to map the ChibiOS's task state values to names.
 *
 */

#ifndef SYSVIEW_CHIBIOS_H
#define SYSVIEW_CHIBIOS_H

#include "SEGGER_SYSVIEW.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void SYSVIEW_ChibiOS_SendTaskInfo(const void *_tp);
extern void SYSVIEW_ChibiOS_Start(U32 SysFreq, U32 CPUFreq, const char *isr_description);

#ifdef __cplusplus
}
#endif

/********************************************************************/
/*      Checks                                                      */
/********************************************************************/
#if !(CH_CFG_USE_REGISTRY == TRUE)
#error "SYSVIEW integration requires CH_CFG_USE_REGISTRY"
#endif

#endif
