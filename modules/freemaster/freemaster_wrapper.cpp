/*
    UNIMOC - Universal Motor Control  2019 Alexander <tecnologic86@gmail.com>

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
#include "freemaster_wrapper.hpp"
#include <string.h>
#include "freemaster.h"

using namespace unimoc::modules::freemaster;


/**
 * @brief Thread main function
 */
void Thread::main(void)
{
	setName("FreeMaster");

	/*
	 * initialize FreeMaster
	 */
	FMSTR_Init();

	/*
	 * Normal main() thread activity, sleeping in a loop.
	 */
	while (TRUE)
	{
		if (SDU1.config->usbp->state == USB_ACTIVE)
		{
			// The FreeMASTER poll call must be called in the main application loop
			// to handle the communication interface and protocol.
			// In LONG_INTR FreeMASTER interrupt mode, all the processing is done
			// during the communication interrupt routine and the FMSTR_Poll() is
			// compiled empty.
			FMSTR_Poll();
		}
		else
		{
			sleep(TIME_MS2I(1000));
		}
	}

}

/**
 * @brief recorder function
 * needs to be called in control thread
 */
void Recorder(void)
{
	if (SDU1.config->usbp->state == USB_ACTIVE)
	{
		// This call should be placed in the timer interrupt or anywhere where
		// the recorder sampling should occur.
		FMSTR_Recorder();
	}
}

/**
 * @brief constructor of freemaster thread
 */
Thread::Thread(void){}


/** \} **/ /* end of doxygen group */
/*-- EOF --*/
