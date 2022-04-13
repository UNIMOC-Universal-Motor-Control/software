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
#include "as5048b.hpp"
#include "hardware_interface.hpp"


///< AS5048B Read/Write 			1 0 0 0 0 A1 A0 R/W
constexpr i2caddr_t ADDRESS			= 0x80>>1;	// the driver adds the R/W bit

///< TIMEOUT for I2C communication
constexpr uint32_t TIMEOUT			= TIME_MS2I(1);

/**
 * set new zero position
 */
void sensor::as5048b::SetZero(const std::uint16_t new_zero)
{
	zero = new_zero;
}

/**
 * get the current position from the sensor
 * @return sensor position with 14bits resolution, 0xFFFF = error
 */
std::uint16_t sensor::as5048b::GetPosition(void)
{
	return position;
}

/**
 * get the current position from the sensor
 * @return sensor position with 14bits resolution, 0xFFFF = error
 */
float sensor::as5048b::GetPosition(const std::uint8_t divider)
{
	const float scale =  math::PI/4096.0f;
	std::int32_t pos_corrected = position - zero;
	while(pos_corrected < 0)  pos_corrected += 8192;

	float angle = (float)(pos_corrected * (std::uint32_t)divider)*scale;

	while((angle > math::_2PI) && (angle > 0.0f)) angle -= math::_2PI;

	return angle;
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


/**
 * Trigger new sensor read
 */
void sensor::as5048b::Read(void)
{
	/* Wakes up the worker thread.*/
	osalSysLock();
	chEvtSignalI(worker.getInner(), (eventmask_t)1);
	osalSysUnlock();
}


/**
 * read the current position from the sensor
 * @return sensor position with 14bits resolution, 0xFFFF = error
 */
std::uint16_t sensor::as5048b::ReadPosition(void)
{
	std::uint16_t position;
	const std::uint8_t reg_address = 0xFE;
	std::uint8_t buffer[2];

	i2cAcquireBus(instance);
	if(MSG_OK == i2cMasterTransmitTimeout(instance, ADDRESS, &reg_address, 1, buffer, 2, TIMEOUT))
	{
		position = (buffer[0] << 5) + (buffer[1] >> 3);
	}
	else
	{
		position = 0xFFFF;
	}
	i2cReleaseBus(instance);

	return position;
}

/**
 * @brief Worker thread main function
 */
void sensor::as5048b::main(void)
{
	setName("AS5048B");

	worker = getSelfX();

	while (TRUE)
	{
		/* Checks if an IRQ happened else wait.*/
		chEvtWaitAny((eventmask_t)1);

		while(instance->state != I2C_READY) chThdSleepMilliseconds(100);

		position = ReadPosition();
	}
}
