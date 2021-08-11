/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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


constexpr uint32_t CRC32MASK   		= 0x04C11DB7;

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
 * initialize non volatile memory
 */
void hardware::i2c::Init(void)
{

}

/**
 * Read buffer from non-volatile memory
 * @param address Start address of the read in non-volatile memory, addressing starts with 0
 * @param buffer Pointer to the buffer to read data to
 * @param length Length of the buffer to read to
 * @return 0 = success
 */
std::uint8_t hardware::memory::Read(const uint32_t address, const void* const buffer, const uint32_t length)
{
	return 0;
}

/**
 * Write buffer to non-volatile memory
 *
 * @note EEPROM may need 5ms to write a page
 *
 * @param address Start address of the read in non-volatile memory, addressing starts with 0
 * @param buffer Pointer to the buffer to write to
 * @param length Length of the buffer to write to
 * @return 0 = success
 */
std::uint8_t hardware::memory::Write(const std::uint32_t address, void const * buffer, const std::uint32_t length)
{
	return 0;
}

/**
 * Get the size of the non-volatile memory
 * @return size of non-volatile memory in bytes
 */
uint32_t hardware::memory::Size(void)
{
	return 0;
}

