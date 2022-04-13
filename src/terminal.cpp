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
#include <stdint.h>
#include "ch.hpp"
#include "shell.h"
#include "terminal.hpp"

using namespace chibios_rt;

/**
 * @namespace shell data and api
 */
namespace terminal
{
	static THD_WORKING_AREA(wa_shell, SHELL_WA_SIZE);


	static const ShellCommand commands[] = {
		{"write", cmd_write},
		{NULL, NULL}
	};

	static const ShellConfig shell_cfg1 = {
		(BaseSequentialStream *)&SDU1,
		commands
	};



	void Init(void)
	{
		/*
		 * Initializes two serial-over-USB CDC drivers.
		 */
		sduObjectInit(&SDU1);
		sduStart(&SDU1, &serusbcfg);

		/*
		 * Activates the USB driver and then the USB bus pull-up on D+.
		 * Note, a delay is inserted in order to not have to disconnect the cable
		 * after a reset.
		 */
		usbDisconnectBus(serusbcfg.usbp);
		chThdSleepMilliseconds(1500);
		usbStart(serusbcfg.usbp, &usbcfg);
		usbConnectBus(serusbcfg.usbp);

		/*
		 * Shell manager initialization.
		 */
		shellInit();

		chThdCreateStatic(&wa_shell, sizeof(wa_shell),
				SHELL_THREAD_NAME, NORMALPRIO + 1,
				shellThread, (void *)&shell_cfg1);
	}

} /* namespace terminal */

/**
 * @brief Thread main function
 */
void terminal::oszi::main(void)
{
	setName("Oszi");

	deadline = System::getTime();

	/*
	 * Normal main() thread activity
	 */
	while (TRUE)
	{
		deadline = System::getTime();
		sleepUntilWindowed(deadline, deadline + CYCLE_TIME);
	}
}




