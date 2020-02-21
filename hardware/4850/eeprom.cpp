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

static void select_half(const uint16_t half);

/**
 * EEPROM constants
 *
 * The AT34C04 will respond to two unique device type identifiers. The device type identifier of ‘1010’(Ah) is
 * necessary to select the device for reading or writing. The device type identifier of ‘0110’(6h) has multiple
 * purposes. First, it is used to access the page address function which determines what the internal address
 * counter is set to. For more information on accessing the page address function, please refer to Section 6.1.1.
 * The device type identifier of ‘0110’(6h) is also used to access the software write protection feature of the
 * device. Information on the software write protection functionality can be found in Section 7.
 *
 * EEPROM Read/Write 				1 0 1 0 A2 A1 A0 R/W
 *
 * Write Protection and
 * Page Address Functions 			0 1 1 0 A2 A1 A0 R/W
 */
constexpr uint8_t RW_ADDRESS		= 0x50;	// the driver adds the R/W bit

/**
 * To provide the greatest flexibility and backwards compatibility with the previous generations of SPD devices, the
 * AT34C04 memory organization is organized into two independent 2-Kbit memory arrays. Each 2-Kbit (256-byte)
 * section is internally organized into two independent quadrants of 128 bytes with each quadrant comprised of
 * eight pages of 16 bytes. Including both memory sections, there are four 128-byte quadrants totaling 512 bytes.
 * The memory array organization details are shown in Section 2. on page 4 and Table 6-1.
 *
 * Setting the Set Page Address (SPA) value selects the desired half of the EEPROM for performing Write or Read
 * operations. This is done by sending the SPA as seen in Figure 6-1. The SPA command sequence requires the
 * Master to transmit a Start condition followed by sending a control byte of ‘011011*0’ where the ‘*’ in the
 * bit 7 position will dictate which half of the EEPROM is being addressed. A ‘0’ in this position (or 6Ch) is
 * required to set the page address to the first half of the memory and a ‘1’ (or 6Eh) is necessary to set the page
 * address to the second half of the memory. After receiving the control byte, the AT34C04 should return an ACK
 * and the Master should follow by sending two data bytes of don’t care values.
 */
constexpr uint8_t SPA(uint8_t half) { return (0x6C | ((half & 0x01)>>1)); };
constexpr uint8_t PAGE(uint8_t x) 	{ return (x & 0x07) << 1; };
constexpr uint32_t MAX_WRITE_BYTES	= 16;
constexpr uint32_t PAGE_SIZE		= 16;
constexpr uint32_t PAGE_NBR			= 8;
constexpr uint32_t SIZE				= 512; 	// AT32C04 4KBit EEPROM

constexpr uint32_t CRC32MASK   		= 0x04C11DB7;

constexpr uint32_t READ_TIMEOUT		= TIME_MS2I(250);
constexpr uint32_t WRITE_TIMEOUT	= TIME_MS2I(300);

/**
 * I2C configuration for EEPROM bus.
 */
static const I2CConfig i2ccfg = {
  STM32_TIMINGR_PRESC(0U) |
  STM32_TIMINGR_SCLDEL(6U) | STM32_TIMINGR_SDADEL(0U) |
  STM32_TIMINGR_SCLH(6U)  | STM32_TIMINGR_SCLL(20U),
  I2C_CR1_DNF,
  0
};

I2CDriver* const I2CP = &I2CD1;
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
	for (byte = 4; byte < (length - 4); byte++)
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
 * Select current half of the EEPROM
 * @param half 0= first half, 1= second half of the EEPROM
 * @return 0 = success
 */
static void select_half(const uint16_t half)
{
	(void)half;
	osalDbgAssert(half < 2, "EEPROM: Half out of bounds");

	uint8_t spa_cmd = SPA(half)>>1;
	uint8_t dummy[2];

	i2cMasterTransmitTimeout(I2CP, spa_cmd , dummy, 2, nullptr, 0, READ_TIMEOUT);
}


/**
 * initialize non volatile memory
 */
void hardware::memory::Init(void)
{
	/* Initialize I2C */
	i2cStart(I2CP, &i2ccfg);
}

/**
 * Read buffer from non-volatile memory
 * @param address Start address of the read in non-volatile memory, addressing starts with 0
 * @param buffer Pointer to the buffer to read data to
 * @param length Length of the buffer to read to
 * @return 0 = success
 */
uint8_t hardware::memory::Read(const uint32_t address, const void* const buffer, const uint32_t length)
{
	uint8_t result = 0;
	uint8_t wordaddr = 0;
	uint32_t read_length = 0;
	msg_t status = MSG_OK;

	osalDbgAssert((address + length) < SIZE, "EEPROM: Read out of bounds");
	osalDbgAssert(buffer != NULL, "EEPROM: Read buffer not existing");


	wordaddr = address;
	read_length = length;

	if(address > SIZE/2)
	{
		select_half(1);
		wordaddr = address - (SIZE/2);
	}
	else
	{
		select_half(0);

		if((address + length) > SIZE/2)
		{
			read_length = SIZE/2 - address;
		}
	}

	status |= i2cMasterTransmitTimeout(I2CP, RW_ADDRESS,
			&wordaddr, 1, nullptr, 0, READ_TIMEOUT);

	osalThreadSleepMilliseconds(5);

	status |= i2cMasterReceiveTimeout(I2CP, RW_ADDRESS, (uint8_t*)buffer, read_length, READ_TIMEOUT);

	// Read the rest of the Data if we started in half 0
	if(read_length < length)
	{
		select_half(1);
		wordaddr = 0;
		status |= i2cMasterTransmitTimeout(I2CP, RW_ADDRESS,
				&wordaddr, 1, nullptr, 0, READ_TIMEOUT);

		osalThreadSleepMilliseconds(5);

		status |= i2cMasterReceiveTimeout(I2CP, RW_ADDRESS, (uint8_t*)buffer, read_length, READ_TIMEOUT);
	}

	if(status == MSG_OK)
	{
		result = 0;
	}
	else
	{
		i2c_error = i2cGetErrors(I2CP);
		result = 0xFF;
	}

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
uint8_t hardware::memory::Write(const uint32_t address, void const * buffer, const uint32_t length)
{
	uint8_t result = 0;
	uint8_t half = 0;
	uint8_t write_addr = (address % (SIZE/2));
	uint32_t written_bytes = 0;
	uint32_t write_length = 0;
	msg_t status = MSG_OK;
	uint8_t write_buffer[PAGE_SIZE + 1] = {0};

	osalDbgAssert((address + length) < SIZE, "EEPROM: Write out of bounds");
	osalDbgAssert(buffer != NULL, "EEPROM: Write buffer not existing");

	if(address > SIZE/2) half = 1;

	select_half(half);

	// In case address is within the first page to write.
	// do a write to the next page boundary
	if(address % PAGE_SIZE)
	{
		write_length = PAGE_SIZE - write_addr%PAGE_SIZE;

		write_buffer[0] = write_addr;
		memcpy(&write_buffer[1], buffer, write_length);
		status |= i2cMasterTransmitTimeout(I2CP, RW_ADDRESS,
				write_buffer, write_length + 1, nullptr, 0, WRITE_TIMEOUT);

		written_bytes += write_length;

		osalThreadSleepMilliseconds(10);
	}

	while (written_bytes < length && status == MSG_OK)
	{
		if(half == 0 && (address + written_bytes) > SIZE/2)
		{
			half = 1;
			select_half(half);
		}

		write_length = length - written_bytes;
		if(write_length > PAGE_SIZE) write_length = PAGE_SIZE;
		write_addr = address + written_bytes;

		write_buffer[0] = write_addr;
		std::memcpy(&write_buffer[1], (uint8_t*)buffer + written_bytes, write_length);
		status |= i2cMasterTransmitTimeout(I2CP, RW_ADDRESS,
				write_buffer, write_length + 1, nullptr, 0, WRITE_TIMEOUT);

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
		i2c_error = i2cGetErrors(I2CP);
		result = 0xFF;
	}

	return result;
}

/**
 * Get the size of the non-volatile memory
 * @return size of non-volatile memory in bytes
 */
uint32_t hardware::memory::GetSize(void)
{
	// AT32C04 4KBit EEPROM
	return SIZE;
}


