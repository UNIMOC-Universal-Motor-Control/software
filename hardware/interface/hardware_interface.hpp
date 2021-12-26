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
#ifndef HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_
#include <cstdint>
#include <array>
#include "ch.hpp"
#include "hal.h"
#include "systems.hpp"
#include "hardware.hpp"


#ifndef HARDWARE_NAME
#error "HARDWARE_NAME needs to be defined in hardware.hpp of current hardware."
#endif

#ifndef HARDWARE_CAPABIITY_RANDOM
#error "HARDWARE_CAPABIITY_RANDOM needs to be defined in hardware.hpp of current hardware."
#endif

#ifndef HARDWARE_CAPABIITY_CAN_FD
#error "HARDWARE_CAPABIITY_CAN_FD needs to be defined in hardware.hpp of current hardware."
#endif

namespace hardware {

	///< Motor Phases. Normally fixed to 3
	const uint8_t PHASES = 3;

	///< reference to thread to be woken up in the hardware control cycle.
	extern chibios_rt::ThreadReference  control_thread;

	/**
	 * Initialize hardware with outputs disabled!
	 */
	extern void Init();

	namespace pwm {
		/*
		 * This interface defines the functions for pwm handling
		 * which all hardware variants need to implement.
		 */

		///< PWM Timer clock in Hz
		extern const uint32_t TIMER_CLOCK;
		
		/**
		 * Get Deadtime of PWM
		 * @return deadtime in nano seconds
		 */
		extern std::uint32_t Deadtime(void);

		/**
		 * Set Deadtime of PWM
		 * @param ns set deadtime in nano seconds to this value
		 * @return new deadtime in nano seconds
		 */
		extern std::uint32_t Deadtime(const std::uint32_t ns);

		/**
		 * Get Period of PWM
		 *
		 * @note With center aligned PWM this is the period of PWM half period.
		 * @return Period of PWM in timer counts
		 */
		extern std::uint32_t Period(void);

		/**
		 * Get Frequency of PWM
		 *
		 * @return PWM frequency in Hz
		 */
		extern std::uint32_t Frequency(void);

		/**
		 * Set Frequency of PWM
		 *
		 * @param freq 	new PWM frequency
		 * @return new PWM frequency in Hz
		 */
		extern std::uint32_t Frequency(const std::uint32_t freq);

		/**
		 * Set the normalized duty cycles for each phase
		 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
		 */
		extern void Duty(const systems::abc& dutys);


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

	/**
	 * get control cycle time
	 * @return control cycle frequency in s
	 */
	extern float Tc(void);

	/**
	 * get control cycle frequency
	 * @return control cycle frequency in Hz
	 */
	extern float Fc(void);

	/**
	  @brief         Floating-point sine and cosine function.
	  @param 	     theta   input value in q31
	  @retval	     out     sine and cosine value of theta
	 */
	systems::sin_cos SinCos(const std::int32_t theta);

	/**
	 * analog value releated functions
	 */
	namespace analog
	{
		/*
		 * This interface defines the functions for adc handling
		 * which all hardware variants need to implement.
		 */
		/**
		 * get analog input
		 * @return analog in put 0 - 1
		 */
		float input(void);

		namespace current {

			/**
			 * Get phase currents in the last control cycle
			 * @param currents
			 */
			extern void Phase(systems::abc& currents);

			/**
			 * Get Filters Group delay (Hardware and Software)
			 * @return Group delay of the hole signal chain in s
			 */
			extern float Tf(void);

			/**
			 * set phase current offsets
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

			/**
			 * Get phase voltages in the last control cycle
			 * @param currents
			 */
			extern void Phase(systems::abc& voltages);

			/**
			 * Get Filters Group delay (Hardware and Software)
			 * @return Group delay of the hole signal chain in s
			 */
			extern float Tf(void);

			/**
			 * set phase voltage offsets
			 */
			extern void SetOffset(void);

			///< absolute maximum voltage
			extern const float MAX;

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
	} /* namespace analog */

	/**
	 * digital I/O releated functions
	 */
	namespace digital
	{
		namespace hall {
			/**
			 * get the sector which is represented by the hall sensors
			 * @return sector of the halls
			 */
			uint8_t State(void);
		} /* namespace hall */
	} /* namespace digital */

	namespace memory
	{
		/**
		 * Read buffer from non-volatile memory
		 * @param buffer Pointer to the buffer to read data to
		 * @param length Length of the buffer to read to
		 * @return false = success
		 */
		extern bool Read(const void* const buffer, const uint32_t length);

		/**
		 * Write buffer to non-volatile memory
		 *
		 * @param buffer Pointer to the buffer to write to
		 * @param length Length of the buffer to write to
		 * @return false = success
		 */
		extern bool Write(void const * buffer, const uint32_t length);

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

#if HARDWARE_CAPABIITY_RANDOM == TRUE
	namespace random {
		/**
		 * Generate random number of variable size
		 * @param buffer pointer to the send buffer
		 * @param length of the send buffer in bytes
		 * @retval true on error
		 */
		extern bool Generate(std::uint8_t* buffer, const std::uint32_t size);
	}
#endif

} /* namespace hardware */


#endif /* HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_ */

