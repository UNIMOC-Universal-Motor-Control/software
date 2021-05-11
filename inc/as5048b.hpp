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
#ifndef INC_AS5048B_HPP_
#define INC_AS5048B_HPP_

#include <cstdint>
#include <cmath>
#include <climits>
#include "ch.hpp"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"
#include "hardware_interface.hpp"

/**
 * @namespace sensor classes
 */
namespace sensor
{

	/**
	 * AS5048B 14Bit hall angle sensor
	 */
	class as5048b: public chibios_rt::BaseStaticThread<64>
	{
	private:
		///< I2C Driver instance
		I2CDriver* const instance;

		///< zero position offset
		std::uint16_t zero;

		///< position
		std::uint32_t position;

		///< reference to the worker thread
		chibios_rt::ThreadReference worker;

		/**
		 * get the current position from the sensor
		 * @return sensor position with 14bits resolution
		 */
		std::uint16_t ReadPosition(void);

	protected:
		/**
		 * Worker Thread function
		 */
		virtual void main(void);

	public:

		/**
		 * (Offset Compensation Finished), logic high indicates
		 * the finished Offset Compensation Algorithm. After power
		 * up the flag remains always to logic high.
		 */
		const std::uint8_t OCF = 0x01;
		/**
		 * (CORDIC Overflow), logic high indicates an out of
		 * range error in the CORDIC part. When this bit is set, the
		 * angle and magnitude data is invalid. The absolute output
		 * maintains the last valid angular value.
		 */
		const std::uint8_t COF = 0x02;
		/**
		 * COMP low, indicates a high magnetic field. It is
		 * recommended to monitor in addition the magnitude
		 * value.
		 */
		const std::uint8_t COMP_LOW = 0x04;
		/**
		 * COMP high, indicated a weak magnetic field. It is
		 * recommended to monitor the magnitude value.
		 */
		const std::uint8_t COMP_HIGH = 0x08;

		/**
		 * constructor of as5048b hall angle sensor interface over I2C
		 * @param instance I2C driver instance pointer
		 */
		as5048b(I2CDriver* const instance): instance(instance), zero(0), position(0) {};

		/**
		 * set new zero position in the sensor
		 * @param new_zero zero position to be set
		 */
		void SetZero(const std::uint16_t new_zero);


		/**
		 * get the current position from the sensor
		 * @return sensor position with 14bits resolution
		 */
		std::uint16_t GetPosition(void);

		/**
		 * get the current angle of the rotor from the sensor
		 * @return angle in radans
		 */
		float GetPosition(const std::uint8_t divider);


		/**
		 * Get the Status of the sensor
		 * @return status byte
		 */
		std::uint8_t GetStatus(void);

		/**
		 * Trigger new sensor read
		 */
		void Read(void);

	};
} /* namespace sensor */

#endif /* INC_AS5048B_HPP_ */

