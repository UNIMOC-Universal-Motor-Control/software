/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
#ifndef HARDWARE_COMMON_CFG_SHELLCONF_H_
#define HARDWARE_COMMON_CFG_SHELLCONF_H_


/**
 * @file    shellconf.h
 * @brief   Simple CLI shell conf header.
 *
 * @addtogroup SHELL
 * @{
 */

/**
 * @brief   Shell maximum input line length.
 */
#define SHELL_MAX_LINE_LENGTH       64

/**
 * @brief   Shell maximum arguments per command.
 */
#define SHELL_MAX_ARGUMENTS         4

/**
 * @brief   Shell maximum command history.
 */
#define SHELL_MAX_HIST_BUFF         8 * SHELL_MAX_LINE_LENGTH

/**
 * @brief   Enable shell command history
 */
#define SHELL_USE_HISTORY           FALSE

/**
 * @brief   Enable shell command completion
 */
#define SHELL_USE_COMPLETION        TRUE

/**
 * @brief   Shell Maximum Completions (Set to max commands with common prefix)
 */
#define SHELL_MAX_COMPLETIONS       8

/**
 * @brief   Enable shell escape sequence processing
 */
#define SHELL_USE_ESC_SEQ           FALSE

/**
 * @brief   Prompt string
 */
#define SHELL_PROMPT_STR            "unimoc> "

/**
 * @brief   Newline string
 */
#define SHELL_NEWLINE_STR            "\r\n"

/**
 * @brief   Default shell thread name.
 */
#define SHELL_THREAD_NAME           "shell"

/**
 * @brief   Enable Test command
 */

#define SHELL_CMD_TEST_ENABLED      FALSE

/** @} */
#endif /* HARDWARE_COMMON_CFG_SHELLCONF_H_ */
