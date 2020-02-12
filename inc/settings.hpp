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
#ifndef INC_SETTINGS_HPP_
#define INC_SETTINGS_HPP_

#include <cstdint>
#include "systems.hpp"

/**
 * @namespace settings to be saved in non volatile memory
 */
namespace settings
{

	/**
	 * @namespace mechanical properties of the system
	 */
	namespace mechanics
	{
		///< inertia of of rotor and connected known mechanics
		extern float J;
	} /* namespace mechanics */

	/**
	 * @namespace motor properties
	 */
	namespace motor
	{
		///< Stator resistance
		extern float Rs;

		///< anisotropic inductance vector
		extern systems::dq L;

		///< velocity constant Kv
		extern float Psi;

		///< number of pole pairs
		extern uint32_t P;

		///< square injection voltage amplitude
		extern float u_inj;

		/**
		 * @namespace motor limits
		 */
		namespace limits
		{
			///< maximum coil current
			extern float current;

			///< maximum angular velocity
			extern float w;

			///< maximum motor temperature
			extern float temperature;
		} /* namespace limits */
	} /* namespace motor */

	/**
	 * @namespace battery properties
	 */
	namespace battery
	{
		///< internal resistance
		extern float Ri;
		/**
		 * @namespace battery limits
		 */
		namespace limits
		{
			///< maximum drive current
			extern float drive_current;

			///< maximum charge current
			extern float charge_current;
		} /* namespace limits */
	} /* namespace battery */

	/**
	 * @namespace control settings
	 */
	namespace control
	{
		///< current control switch
		extern bool current;

	} /* namespace control */

	/**
	 * @namespace observer settings
	 */
	namespace observer
	{
		///< flux observer switch
		extern bool flux;

		///< admittance observer switch
		extern bool admittance;

		///< mechanic observer switch
		extern bool mechanic;

		///< modell variance
		extern float Q;

		///< measurement variance
		extern float R;

		///< flux observer feedback gains
		extern systems::dq C;
	} /* namespace observer */

	/**
	 * @namespace converter settings
	 */
	namespace converter
	{
		///< control period
		extern float ts;
		/**
		 * @namespace converter limits
		 */
		namespace limits
		{
			///< maximum phase current
			extern float current;

			///< maximum motor temperature
			extern float temperature;
		} /* namespace limits */
	} /* namespace converter */
} /* namespace setting */

#endif /* INC_SETTINGS_HPP_ */

