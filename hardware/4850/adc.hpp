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
#ifndef HARDWARE_INTERFACE_4850_ADC_HPP_
#define HARDWARE_INTERFACE_4850_ADC_HPP_
#include <cstdint>
#include "hardware_interface.hpp"
#include "ch.hpp"
#include "hal.h"


namespace hardware {
	namespace adc {

	///< Samples in ADC sequence.
	/// Caution: samples are 16bit but the hole sequence must be 32 bit aligned!
	///          so even length of sequence is best choice.
	constexpr uint32_t LENGTH_ADC_SEQ = 16;

	///< ADC sequences in buffer.
	/// Caution: samples are 16bit but the hole sequence must be 32 bit aligned!
	///          so even length of sequence is best choice.
	constexpr uint32_t ADC_SEQ_BUFFERED = 16;

	///< # of ADCs
	constexpr uint32_t NUM_OF_ADC = 3;

	///< dma accessed buffer for the adcs, probably cached
	extern adcsample_t samples[NUM_OF_ADC][ADC_SEQ_BUFFERED][LENGTH_ADC_SEQ];
	///< cycle index for cached working buffer
	extern uint32_t samples_index;
	} /* namespace adc */
} /* namespace hardware */


#endif /* HARDWARE_INTERFACE_4850_ADC_HPP_ */
