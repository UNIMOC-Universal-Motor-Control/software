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
constexpr uint8_t PHASES = 3;

///< reference to thread to be woken up in the hardware control cycle.
extern chibios_rt::ThreadReference  control_thread;

namespace pwm {
/*
 * This interface defines the functions for pwm handling
 * which all hardware variants need to implement.
 */

///< PWM Timer clock in Hz
constexpr uint32_t TIMER_CLOCK = STM32_TIMCLK2;


///< deadtime in nano seconds
constexpr uint32_t DEADTIME = 300;

/**
 * PWM frequency in Hz
 *
 * The ADC sampling relies on the PWM frequency!
 * Each ADC samples 16 samples with a sample time of 15clk + 12clk for conversion
 * + 1clk for triggering. ADC clock is divided by 8 from PWM clock.
 *
 * So Period of the PWM needs to be greater than 16*28*8=3584clk.
 * With center aligned PWM this is the period of PWM half period.
 */
constexpr uint32_t PERIOD = 3600;

///< current injection cycles used for low speed position estimation
constexpr uint32_t INJECTION_CYCLES = 4;

/**
 * Initialize PWM hardware with outputs disabled!
 */
extern void Init();

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

/**
 * Set the normalized duty cycles for each phase
 * @param dutys -1 = LOW, 0 = 50%, 1=HIGH
 */
extern void Dutys(const std::array<systems::abc, INJECTION_CYCLES> dutys);

} /* namespace pwm */

///< Control cycle time
constexpr float Tc = ((float)(pwm::PERIOD + 1) / (float)pwm::TIMER_CLOCK)*pwm::INJECTION_CYCLES;

///< Control cycle frequency
constexpr float Fc = 1.0f/Tc;

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

/**
 * prepare samples for evaluation
 *
 * @note on a cache MCU this invalidates cache for samples
 */
extern void PrepareSamples(void);

namespace current {

/**
 * Get current means of the current in the last control
 * cycles
 * @param currents references to the current mean samples
 */
extern void Mean(std::array<systems::abc, pwm::INJECTION_CYCLES>& currents);

/**
 * Get current injection samples in the last control cycle
 * @param currents points to the current injection samples
 */
extern void Injection(std::array<systems::abc, pwm::INJECTION_CYCLES>& currents);

namespace offset {

	/**
	 * set dc current offsets
	 * @param offset in A
	 */
	extern void DC(systems::abc& offset);

	/**
	 * set AC current offsetsS
	 * @param offset in A
	 */
	extern void AC(systems::abc& offset);
} /* namespace offset */

namespace gain {

	/**
	 * set dc current gains
	 * @param gain in A/A
	 */
	extern void DC(systems::abc& gain);
	/**
	 * set ac current gains
	 * @param gain in A/A
	 */
	extern void AC(systems::abc& gain);
} /* namespace gain */
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

/**
 * Get the external throttle command value
 * @return Throttle in a range of -1 to 1
 */
extern float Throttle(void);

} /* namespace adc */

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
} /* namespace hardware */


#endif /* HARDWARE_INTERFACE_HARDWARE_INTERFACE_HPP_ */

