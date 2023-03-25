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


///< CAN Driver instances if redundant. Instance 0 is always master CAN
CANDriver* pcan[HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES] = {&CAND3};

///< embedded flash eeprom driver config
MFSConfig mfscfg1 = {
  .flashp           = (BaseFlash *)&EFLD1,
  .erased           = 0xFFFFFFFFU,
  .bank_size        = 2048U,
  .bank0_start      = 1U,
  .bank0_sectors    = 1U,
  .bank1_start      = 2U,
  .bank1_sectors    = 1U
};

///< Option Bytes flash unlock keys
constexpr uint32_t FLASH_OPTKEY1 = 0x08192A3BU;
constexpr uint32_t FLASH_OPTKEY2 = 0x4C5D6E7FU;

extern void hardware_pwm_Init(void);
extern void hardware_analog_Init(void);
extern void hardware_cordic_Init(void);
extern void hardware_can_Init(void);

/**
 * Initialize hardware with outputs disabled!
 */
void hardware::Init()
{
	/*
	 * The boot pin is used as PWM IO which leads to boot issues, that messup the clock tree config.
	 * So fix the boot config to ignore BOOT0 hw pin
	 */
	if (FLASH->OPTR & FLASH_OPTR_nSWBOOT0)
	{
		FLASH->OPTKEYR |= FLASH_OPTKEY1;
		FLASH->OPTKEYR |= FLASH_OPTKEY2;

		// wait for optlock to clear
		while (FLASH->CR & FLASH_CR_OPTLOCK);

		// fix boot from main flash
		FLASH->OPTR |=  (FLASH_OPTR_nBOOT0);
		FLASH->OPTR &= ~(FLASH_OPTR_nSWBOOT0);

		FLASH->OPTR |= FLASH_CR_OPTSTRT;

		/* Wait for busy bit clear.*/
		while ((FLASH->SR & FLASH_SR_BSY) != 0U);

		FLASH->OPTR |= FLASH_CR_OBL_LAUNCH;
	}

	hardware_pwm_Init();
	hardware_analog_Init();
	hardware_cordic_Init();
	hardware_can_Init();

	trngInit();
	trngStart(&TRNGD1, NULL);
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

/**
 * Generate random number of variable size
 * @param buffer pointer to the send buffer
 * @param length of the send buffer in bytes
 */
bool hardware::random::Generate(std::uint8_t* buffer, const std::uint32_t size)
{
	return trngGenerate(&TRNGD1, size, buffer);
}

