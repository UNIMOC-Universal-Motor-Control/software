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
#include <stdlib.h>
#include "ch.hpp"
#include "hal.h"
#include "usbcfg.h"
#include "terminal.hpp"
#include "hardware_interface.hpp"

///< Shell Thread working area
constexpr size_t SHELL_WA_SIZE = 2048;
static THD_WORKING_AREA(wa_shell_thread, SHELL_WA_SIZE);

///< Shell history buffer
static char history_buffer[SHELL_MAX_HIST_BUFF];

///< Shell completion buffer
static char *completion_buffer[SHELL_MAX_COMPLETIONS];


/**
 * shell command to display mcu usage in the terminal.
 * @param chp	Serial channel for display
 * @param argc	argument count
 * @param argv	argument strings
 */
static void cpu_load(BaseSequentialStream *chp, int argc, char *argv[])
{
	thread_t *tp;
	uint64_t sum=0;
	uint16_t tmp1, tmp2;

	(void)argc;
	(void)argv;

	tp = chRegFirstThread();
	do {
		sum += tp->stats.cumulative;
		tp = chRegNextThread(tp);
	} while (tp != NULL);
	sum += ch.kernel_stats.m_crit_thd.cumulative;
	sum += ch.kernel_stats.m_crit_isr.cumulative;

	tp = chRegFirstThread();
	do {
		tmp1 = (uint16_t)(tp->stats.cumulative*10000/sum);
		chprintf(chp, "%12s %u.%u%%\r\n", tp->name, tmp1/100, tmp1%100);
		tp = chRegNextThread(tp);
	} while (tp != NULL);

	tmp1 = (uint16_t)(ch.kernel_stats.m_crit_thd.cumulative*10000/sum);
	tmp2 = (uint16_t)(ch.kernel_stats.m_crit_isr.cumulative*10000/sum);

	chprintf(chp, "thd:%u.%u%%   isr:%u.%u%%\r\n",
			tmp1/100, tmp1%100,tmp2/100, tmp2%100);
	chprintf(chp, "\r\n");
}

///< shell command list
static const ShellCommand commands[] = {
		{"procs", cpu_load},
		{NULL, NULL}
};

///< Shell configuration
const ShellConfig shell_cfg = {
		(BaseSequentialStream *)&SDU2,
		commands,
		history_buffer,
		sizeof(history_buffer),
		completion_buffer
};

/**
 * Initialize terminal with shell
 */
void unimoc::terminal::Init(void)
{
	/*
	 * Shell manager initialization.
	 */
	shellInit();

	chThdCreateStatic(wa_shell_thread, SHELL_WA_SIZE, NORMALPRIO +1,
			shellThread, (void *)&shell_cfg);
}


