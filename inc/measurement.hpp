/*
    UNIMOC - Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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
#ifndef INC_MEASUREMENT_HPP_
#define INC_MEASUREMENT_HPP_


#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"


/**
 * @namespace measurement data
 */
namespace measurement
{
	/**
	 * @namespace hall sensor measurement values for control cycle update
	 */
	namespace hall
	{
		/**
		 * @namespace hall sensor transition variables
		 */
		namespace transition
		{
			///< hall state change edge detection
			extern std::uint8_t old;
			///< previous hall state before transition
			extern std::uint8_t prev;
			///< transition angle
			extern std::int32_t phi;
		}
	}
}



#endif /* INC_MEASUREMENT_HPP_ */
