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
#include <cstring>
#include "ch.hpp"
#include "hal.h"
#include "usbcfg.h"
#include "terminal.hpp"
#include "hardware_interface.hpp"
#include "freemaster_wrapper.hpp"

using namespace chibios_rt;

static unimoc::modules::freemaster::Thread freemaster;

float dutys[unimoc::hardware::PHASES] = {0.0f, 0.0f, 0.0f};

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
	 * Initializes two serial-over-USB CDC drivers.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg1);
	sduObjectInit(&SDU2);
	sduStart(&SDU2, &serusbcfg2);

	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusbcfg1.usbp);
	chThdSleepMilliseconds(1500);
	usbStart(serusbcfg1.usbp, &usbcfg);
	usbConnectBus(serusbcfg1.usbp);

	/*
	 * start freemaster thread
	 */
	freemaster.start(NORMALPRIO);

	/*
	 * Set hall inputs to outputs for debugging
	 */
	palSetLineMode(LINE_HALL_A, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_HALL_B, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_HALL_C, PAL_MODE_OUTPUT_PUSHPULL);

	unimoc::hardware::memory::Init();
	unimoc::hardware::pwm::Init();
	unimoc::hardware::adc::Init();
	unimoc::terminal::Init();

	unimoc::hardware::pwm::SetDutys(dutys);
	unimoc::hardware::pwm::EnableOutputs();

	/// Memory Tests
	{
		using namespace unimoc::hardware::memory;
		uint32_t write_data[6] = {0xDEADBEEF, 0xAFFEAFFE, 0x13377331, 0xBAAD5555, 0xA5A5A5A5, 0x5A5A5A5A};
		uint32_t read_data[6] = {0};
		uint8_t result;
		uint32_t address = 0, size = sizeof(write_data);

		chprintf((BaseSequentialStream*)&SDU2, TCOL_RED "NON-VOLATILE MEMORY TEST\n" TCOL_RESET);
		chprintf((BaseSequentialStream*)&SDU2, "Memory Size: %d\n", unimoc::hardware::memory::GetSize());

		for(uint32_t i = address; i < unimoc::hardware::memory::GetSize() - size; i++)
		{
			result = Write(address, (const void*)write_data, size);
			chprintf((BaseSequentialStream*)&SDU2, "Write %d Bytes to Address %d with Result %d\n", size, address, result);
			result = Read(address, (const void*)read_data, size);
			chprintf((BaseSequentialStream*)&SDU2, "Read %d Bytes from Address %d with Result %d, Data is %s\n",
					size, address, result, std::memcmp(read_data, write_data, size) ? TCOL_RED "DIFFERENT" TCOL_RESET : TCOL_GREEN "SAME" TCOL_RESET);
			BaseThread::sleep(TIME_MS2I(50));
		}
	}
	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (true)
	{
		unimoc::hardware::pwm::SetDutys(dutys);
		BaseThread::sleep(TIME_MS2I(500));
	}
}
