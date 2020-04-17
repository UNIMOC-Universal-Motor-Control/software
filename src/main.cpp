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
#include "usbcfg.h"
#include "hardware_interface.hpp"
#include "freemaster_wrapper.hpp"
#include "management.hpp"
#include "uavcan.hpp"
#include "main.hpp"

using namespace chibios_rt;

static modules::freemaster::thread freemaster;
static control::thread controller;
static management::thread manager;

/**
 * Code entry point
 * @return never
 */
int main(void)
{
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	System::init();

	/*
	 * initialize hardware with no control thread
	 */
	hardware::memory::Init();
	hardware::pwm::Init();
	hardware::adc::Init();
	uavcan::Init();

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

	// set node operational
	values.uavcan.mode = uavcan::mode_e::UAVCAN_NODE_MODE_OPERATIONAL;

	chThdSetPriority(LOWPRIO);

	/*
	 * start threads
	 */
	freemaster.start(NORMALPRIO);
	hardware::control_thread = controller.start(HIGHPRIO);
	manager.start(NORMALPRIO + 1);

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (true)
	{
		uavcan::Run();
	}
}
