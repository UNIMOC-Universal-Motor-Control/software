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
#ifndef INC_HALL_SENSOR_HPP_
#define INC_HALL_SENSOR_HPP_


#include <cstdint>
#include "systems.hpp"


/**
 * @namespace hall sensor data and exported functions
 */
namespace hall
{
	/**
	 * @namespace hall sensor transition variables
	 */
	namespace transition
	{
		/**
		 * Update the transition variables
		 */
		extern void Update(void);
	}

	/**
	 * @namespace hall sensor observer helper function
	 */
	namespace observer
	{

	    /**
	     * @brief Get sine and cosine values from hall for estimation in rotor frame.
	     * @retval est hall sensor signal in rotor frame
	     */
	    extern void GetFluxVector(systems::dq& est);
	}
}



#endif /* INC_HALL_SENSOR_HPP_ */
