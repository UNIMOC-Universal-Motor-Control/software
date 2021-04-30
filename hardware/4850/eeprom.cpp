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
#include <cstdint>
#include <cstring>
#include "hardware_interface.hpp"
#include "hal.h"


constexpr i2caddr_t RW_ADDRESS		= 0xA0>>1;	// the driver adds the R/W bit

constexpr uint32_t PAGE_SIZE		= 32;
constexpr uint32_t PAGE_NBR			= 512;
constexpr uint32_t SIZE				= PAGE_SIZE * PAGE_NBR; 	// 24CW1280

constexpr uint32_t CRC32MASK   		= 0x04C11DB7;

constexpr uint32_t READ_TIMEOUT		= TIME_MS2I(1000);
constexpr uint32_t WRITE_TIMEOUT	= TIME_MS2I(1000);

/**
 * I2C configuration for EEPROM bus.
 */
static const I2CConfig i2ccfg = {
  STM32_TIMINGR_PRESC(0U) |
  STM32_TIMINGR_SCLDEL(5U) | STM32_TIMINGR_SDADEL(1U) |
  STM32_TIMINGR_SCLH(5U)  | STM32_TIMINGR_SCLL(20U),
  I2C_CR1_DNF,
  0
};

I2CDriver* const hardware::i2c::instance = &I2CD1;
volatile i2cflags_t i2c_error = 0;

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
	/* Initialize I2C */
	i2cStart(instance, &i2ccfg);
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
	std::uint8_t result = 0;
	std::uint16_t wordaddr = 0;
	std::uint32_t read_length = 0;
	msg_t status = MSG_OK;

	osalDbgAssert((address + length) < SIZE, "EEPROM: Read out of bounds");
	osalDbgAssert(buffer != NULL, "EEPROM: Read buffer not existing");

	i2cAcquireBus(i2c::instance);

	wordaddr = address;
	read_length = length;

	status |= i2cMasterTransmitTimeout(i2c::instance, RW_ADDRESS,
			(std::uint8_t*)&wordaddr, sizeof(wordaddr), (std::uint8_t*)buffer, read_length, READ_TIMEOUT);


	// buffer needs to be invalidated because data was written from dma
	cacheBufferInvalidate(buffer, length);


	if(status == MSG_OK)
	{
		result = 0;
	}
	else
	{
		i2c_error = i2cGetErrors(i2c::instance);
		result = 0xFF;
	}

	i2cReleaseBus(i2c::instance);

	return result;
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
	std::uint8_t result = 0;
	std::uint16_t write_addr = address;
	std::uint8_t* buffer_addr = (std::uint8_t*)buffer;
	std::uint32_t written_bytes = 0;
	std::uint32_t write_length = 0;
	msg_t status = MSG_OK;
	// is accessed via dma and needs to be flushed
	__attribute__((aligned (32))) std::array<std::uint8_t, PAGE_SIZE + sizeof(write_addr)> write_buffer = {0};

	osalDbgAssert((address + length) < SIZE, "EEPROM: Write out of bounds");
	osalDbgAssert(buffer != NULL, "EEPROM: Write buffer not existing");

	i2cAcquireBus(i2c::instance);

	while (written_bytes < length && status == MSG_OK)
	{
		write_addr = address + written_bytes;
		write_length = length - written_bytes;

		// In case address is within the first page to write.
		// do a write to the next page boundary
		if(write_addr % PAGE_SIZE)
		{
			std::uint32_t rem_page_bytes = PAGE_SIZE - write_addr%PAGE_SIZE;
			if(write_length > rem_page_bytes)
			{
				write_length = rem_page_bytes;
			}
		}

		if(write_length > PAGE_SIZE) write_length = PAGE_SIZE;


		buffer_addr = (std::uint8_t*)buffer + written_bytes;

		write_buffer[0] = write_addr >> 8;
		write_buffer[1] = write_addr & 0x00FF;

		std::memcpy(write_buffer.data() + sizeof(write_addr), buffer_addr, write_length);

		// write buffer out to ram so that dma can access it.
		cacheBufferFlush(write_buffer.data(), sizeof(write_buffer));

		status |= i2cMasterTransmitTimeout(i2c::instance, RW_ADDRESS,
				write_buffer.data(), write_length + sizeof(write_addr), nullptr, 0, WRITE_TIMEOUT);

		written_bytes += write_length;

		// EEPROM may need 5ms to write a page
		osalThreadSleepMilliseconds(10);
	}

	if(status == MSG_OK)
	{
		result = 0;
	}
	else
	{
		i2c_error = i2cGetErrors(i2c::instance);
		result = 0xFF;
	}

	i2cReleaseBus(i2c::instance);

	return result;
}

/**
 * Get the size of the non-volatile memory
 * @return size of non-volatile memory in bytes
 */
uint32_t hardware::memory::Size(void)
{
	return SIZE;
}


