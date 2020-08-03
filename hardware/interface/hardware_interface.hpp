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
#ifndef HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_
#include <cstdint>
#include <array>
#include "ch.hpp"
#include "hal.h"
#include "systems.hpp"


namespace hardware {
	///< Motor Phases. Normally fixed to 3
	const uint8_t PHASES = 3;

	///< reference to thread to be woken up in the hardware control cycle.
	extern chibios_rt::ThreadReference  control_thread;

	namespace pwm {
		/*
		 * This interface defines the functions for pwm handling
		 * which all hardware variants need to implement.
		 */

		///< PWM Timer clock in Hz
		extern const uint32_t TIMER_CLOCK;


		///< deadtime in nano seconds
		extern const uint32_t DEADTIME;

		/**
		 * PWM frequency in Hz
		 *
		 * With center aligned PWM this is the period of PWM half period.
		 */
		extern const uint32_t PERIOD;

		/**
		 * Initialize PWM hardware with outputs disabled!
		 */
		extern void Init();

		/**
		 * Set the normalized duty cycles for each phase
		 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
		 */
		extern void Dutys(const systems::abc& dutys);

		namespace output {
			/**
			 * Enable PWM Outputs.
			 * @note Power output is active after this call
			 */
			extern void Enable(void);

			/**
			 * Disable PWM Outputs.
			 */
			extern void Disable(void);

			/**
			 * Get pwm output state
			 * @return pwm output state, true = pwm active
			 */
			extern bool Active(void);
		} /* namespace output */

	} /* namespace pwm */

	///< Control cycle time
	extern const float Tc;

	///< Control cycle frequency
	extern const float Fc;

	///< Filter Group delay
	extern const float Tf;

	namespace adc
	{
		/*
		 * This interface defines the functions for adc handling
		 * which all hardware variants need to implement.
		 */

		/**
		 * Initialize ADC hardware with outputs disabled!
		 */
		extern void Init();

		namespace current {

			/**
			 * Get current in the last control
			 * cycles
			 * @param currents
			 */
			extern void Value(systems::abc& currents);

			/**
			 * set current offsets
			 * @param offset in A
			 */
			extern void SetOffset(void);

			///< absolute maximum current
			extern const float MAX;

		} /* namespace current*/

		namespace voltage {
			/**
			 * Read the DC Bus voltage
			 * @return DC Bus voltage in Volts
			 */
			extern float DCBus(void);

		} /* namespace voltage */

		namespace temperature {
			/**
			 * Get the temperature of the power electronics
			 * @return Temperature of the power electronics in °C
			 */
			extern float Bridge(void);

			/**
			 * Get the temperature of the motor
			 * @return Temperature of the Motor in °C
			 */
			extern float Motor(void);
		} /* namespace temperature */

		namespace hall {
			/**
			 * get the angle which is represented by the hall sensors
			 * @param[out] sincos angle of the halls represented as sin/cos values
			 * @return true on hall signal error
			 */
			bool Angle(systems::sin_cos& sincos);
		} /* namespace angle */
	} /* namespace adc */

	namespace crank {

		/**
		 * Torque on the crank arm
		 *
		 * @param offset in Volts
		 * @param gain in Nm/V
		 * @return Torque in Nm
		 */
		extern float Torque(const float offset, const float gain);

		/**
		 * Angle of the crank arm
		 *
		 * @param edge_max Number of edges per revolution
		 * @return Angle in rads, range 0 - 2*PI
		 */
		extern float Angle(uint32_t edge_max);

	} /* namespace crank */

	namespace memory
	{
		/**
		 * initialize non volatile memory
		 */
		extern void Init(void);

		/**
		 * Read buffer from non-volatile memory
		 * @param address Start address of the read in non-volatile memory, addressing starts with 0
		 * @param buffer Pointer to the buffer to read data to
		 * @param length Length of the buffer to read to
		 * @return 0 = success
		 */
		extern uint8_t Read(const uint32_t address, const void* const buffer, const uint32_t length);

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
		extern uint8_t Write(const uint32_t address, void const * buffer, const uint32_t length);

		/**
		 * Get the size of the non-volatile memory
		 * @return size of non-volatile memory in bytes
		 */
		extern uint32_t Size(void);

		/**
		 * Calculate CRC32 checksum of a buffer.
		 * @param buffer Pointer to the buffer to calculate the CRC of, bytes 0 to 3 are for CRC32
		 * @param length Length of the buffer to calculate the CRC of
		 * @return return CRC32 value
		 */
		extern uint32_t Crc32(const void* const buffer, const uint32_t length);
	} /* namespace memory*/

	namespace serial {
		/**
		 * Send the buffer via Serial to the Display
		 * @param buffer pointer to the send buffer
		 * @param length of the send buffer in bytes
		 */
		extern void Send(char* buffer, uint32_t length);

		/**
		 * Receive data from the Display
		 * @param buffer pointer to the receive buffer
		 * @param length of the receive buffer.
		 */
		extern void Receive(char* buffer, uint32_t length);
	} /* namespace serial */
} /* namespace hardware */


#endif /* HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_ */

