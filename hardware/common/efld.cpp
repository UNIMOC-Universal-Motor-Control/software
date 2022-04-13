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
#include <cstdint>
#include <cstring>
#include "hardware_interface.hpp"
#include "hal.h"
#include "hal_mfs.h"

///< CRC seed
constexpr uint32_t CRC32MASK   		= 0x04C11DB7;

///< embedded flash eeprom driver config need to be defined by each hardware
extern MFSConfig mfscfg1;

///< Managed flash driver instance for embedded flash eeprom emulation
MFSDriver mfs1;

/**
 * Calculate CRC32 checksum of a buffer.
 * @param buffer Pointer to the buffer to calculate the CRC of, bytes 0 to 3 are for CRC32
 * @param length Length of the buffer to calculate the CRC of
 * @return
 */
uint32_t hardware::memory::Crc32(const void* const buffer, const uint32_t length)
{
	uint32_t byte = 0, bit = 0, crc32 = 0;

	uint8_t* ptr = (uint8_t*)buffer;

	// Bytes 0 to 3 are CRC
	for (byte = 4; byte < length; byte++)
	{
		// apply bit wise CRC mask
		for(bit = 0; bit < 8; bit++)
		{
			if ( ((crc32 >> 31U) & 1U) != ((ptr[byte] >> bit) & 1U))
				crc32 = (crc32 << 1) ^ CRC32MASK;
			else
				crc32 = (crc32 << 1);
		}
	}
	return crc32;
}


/**
 * Read buffer from non-volatile memory
 * @param buffer Pointer to the buffer to read data to
 * @param length Length of the buffer to read to
 * @return 0 = success
 */
bool hardware::memory::Read(const void* const buffer, const uint32_t length)
{
	bool result = false;
	mfs_error_t error = MFS_NO_ERROR;
	size_t size = length;

	/* Starting EFL driver.*/
	eflStart(&EFLD1, NULL);

	error = mfsStart(&mfs1, &mfscfg1);

	if(!error) error = mfsReadRecord(&mfs1, 1, &size, (std::uint8_t*)buffer);

	mfsStop(&mfs1);

	eflStop(&EFLD1);

	if(error || size != length) result = true;

	return result;
}

/**
 * Write buffer to non-volatile memory
 *
 * @note EEPROM may need 5ms to write a page
 *
 * @param buffer Pointer to the buffer to write to
 * @param length Length of the buffer to write to
 * @return false = success
 */
bool hardware::memory::Write(void const * buffer, const std::uint32_t length)
{
	bool result = false;
	mfs_error_t error = MFS_NO_ERROR;

	/* Starting EFL driver.*/
	eflStart(&EFLD1, NULL);

	error = mfsStart(&mfs1, &mfscfg1);

	if(!error) error = mfsWriteRecord(&mfs1, 1, length, (std::uint8_t*)buffer);

	mfsStop(&mfs1);

	eflStop(&EFLD1);

	if(error) result = true;

	return result;
}

/**
 * Get the size of the non-volatile memory
 * @return size of non-volatile memory in bytes
 */
uint32_t hardware::memory::Size(void)
{
	return mfscfg1.bank_size;
}
