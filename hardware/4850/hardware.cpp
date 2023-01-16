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

#include "hardware_interface.hpp"
#include "hal.h"
#include "hal_mfs.h"

MFSConfig mfscfg1 = {
	.flashp           = (BaseFlash *)&EFLD1,
	.erased           = 0xFFFFFFFFU,
	.bank_size        = 16384U,
	.bank0_start      = 1U,
	.bank0_sectors    = 1U,
	.bank1_start      = 2U,
	.bank1_sectors    = 1U
};

///< CAN Driver instances if redundant. Instance 0 is always master CAN
CANDriver* pcan[HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES] = {&CAND1};


extern void hardware_pwm_Init(void);
extern void hardware_analog_Init(void);
extern void hardware_can_Init(void);

/**
 * Initialize hardware with outputs disabled!
 */
void hardware::Init()
{
	hardware_pwm_Init();
	hardware_analog_Init();
	hardware_can_Init();
}


/**
 * get the angle which is represented by the hall sensors
 * @param[out] sincos angle of the halls represented as sin/cos values
 * @return true on hall signal error
 */
uint8_t hardware::digital::hall::State(void)
{
	return palReadGroup(GPIOC, 0x0007, 13);
}

