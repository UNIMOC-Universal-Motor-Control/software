/*
    UNIMOC - Universal Motor Control  2020 Alexander <tecnologic86@gmail.com> Brand

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
#include <stdint.h>
#include "ch.hpp"
#include "hal.h"
#include "hardware_interface.hpp"
#include "management.hpp"

using namespace chibios_rt;

/**
 * @namespace controller management classes
 */
namespace management
{
	/**
	 * generic constructor
	 */
	thread::thread(): deadline(0), sequencer(STARTUP)
	{};

	/**
	 * @brief Thread main function
	 */
	void thread::main(void)
	{
		setName("Management");


		deadline = chibios_rt::System::getTime();
		/*
		 * Normal main() thread activity
		 */
		while (TRUE)
		{

			sleepUntil(deadline + CYCLE_TIME);
		}
	}

} /* namespace management */

