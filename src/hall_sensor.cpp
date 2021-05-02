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

#include <cstdint>
#include <array>
#include "values.hpp"
#include "hall_sensor.hpp"
#include "settings.hpp"



/**
 * @namespace hallsensor measurement values
 */
namespace hall
{

	/**
	 * @namespace hall sensor transition variables
	 */
	namespace transition
	{
		///< hall state change edge detection
		std::uint8_t old = 0;
		///< previous hall state before transition
		std::uint8_t prev = 0;
		///< transition angle
		std::int32_t phi = 0;
	} /* namespace transition */
} /* namespace hall */

/**
 * Update the transition variables
 */
void hall::transition::Update(void)
{
	// update the old hall states
	if(values::motor::hall::state != old && prev == 0)
	{
		prev = old; // update the last hall state
		old = values::motor::hall::state; // update the edge detection
		phi = values::motor::rotor::phi;
	}
}

/**
 * @brief Get sine and cosine values from hall for estimation in rotor frame.
 * @retval est hall sensor signal in rotor frame
 */
void hall::observer::GetFluxVector(systems::dq& est)
{
	systems::alpha_beta tab = {settings.motor.psi, 0.0f};
	systems::sin_cos sc = systems::SinCos(settings.observer.hall.map[values::motor::hall::state][hall::transition::prev]);

	// bring hall angle vector to rotor frame
	est = systems::transform::Park(tab, sc);
}
