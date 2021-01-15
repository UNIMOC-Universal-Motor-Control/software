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
#include "as5048b.hpp"
#include "hardware_interface.hpp"


///< AS5048B Read/Write 			1 0 0 0 0 A1 A0 R/W
constexpr i2caddr_t ADDRESS			= 0x80>>1;	// the driver adds the R/W bit

///< TIMEOUT for I2C communication
constexpr uint32_t TIMEOUT			= TIME_MS2I(1);

/**
 * set new zero position in the sensor
 * @param new_zero zero position to be set
 * @return false = success
 */
bool sensor::as5048b::SetZero(const std::uint16_t new_zero)
{
	bool result = true;
	std::uint8_t buffer[3];

	buffer[0] = 0x16;				// Register Address
	buffer[1] = new_zero >> 8;
	buffer[2] = new_zero & 0x00FF;


	if(MSG_OK == i2cMasterTransmitTimeout(instance, ADDRESS, buffer, 3, nullptr, 0, TIMEOUT))
	{
		result = false;
		zero = new_zero;
	}
	return result;
}

/**
 * get the current position from the sensor
 * @return sensor position with 14bits resolution, 0xFFFF = error
 */
std::uint16_t sensor::as5048b::GetPosition(void)
{
	const std::uint8_t reg_address = 0xFE;
	std::uint8_t buffer[2];

	if(MSG_OK == i2cMasterTransmitTimeout(instance, ADDRESS, &reg_address, 1, buffer, 2, TIMEOUT))
	{
		position = (buffer[0] << 5) + (buffer[1] >> 3);
	}
	else
	{
		position = 0xFFFF;
	}

	return position;
}

/**
 * Get the Status of the sensor
 * @return status byte
 */
std::uint8_t sensor::as5048b::GetStatus(void)
{
	std::uint8_t result;
	const std::uint8_t reg_address = 0xFE;

	if(MSG_OK != i2cMasterTransmitTimeout(instance, ADDRESS, &reg_address, 1, &result, 1, TIMEOUT))
	{
		result = 0xFF;
	}

	return result;
}
