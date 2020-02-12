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
#include "settings.hpp"
#include "hardware_interface.hpp"

/**
 * @namespace system settings
 */
namespace settings
{

	/**
	 * @namespace mechanic system settings
	 */
	namespace mechanics
	{
		///< inertia of of rotor and connected known mechanics
		float J = 1e-4f;
	} /* namespace mechanics */

	/**
	 * @namespace motor settings
	 */
	namespace motor
	{
		///< Stator resistance
		float Rs = 80e-3f;

		///< anisotropic inductance vector
		systems::dq L = {50e-6f, 60e-6f};

		///< number of pole pairs
		uint32_t P = 14 / 2;

		///< magnetic flux inducted voltage in rotor
		float Psi = unit::RpmV2VsRad(192.0f) / (float)P;

		///< square injection voltage amplitude
		float u_inj = 0.04f;

		/**
		 * @namespace motor limit settings
		 */
		namespace limits
		{
			///< maximum coil current
			float current = 120.0f;

			///< maximum angular velocity
			float w = unit::RadS(15000.0f);

			///< maximum motor temperature
			float temperature = 80.0f;
		} /* namespace limits */
	} /* namespace motor */

	/**
	 * @namespace battery settings
	 */
	namespace battery
	{
		/**
		 * @namespace battery limits
		 */
		namespace limits
		{
			///< maximum drive current
			float drive_current = 30.0f;

			///< maximum charge current
			float charge_current = 10.0f;

			///< maximum battery temperature
			float temperature = 80.0f;
		} /* namespace limits */
	} /* namespace battery */

	/**
	 * @namespace control settings
	 */
	namespace control
	{
		///< current control switch
		bool current = true;

	} /* namespace control */

	/**
	 * @namespace observer settings
	 */
	namespace observer
	{
		///< flux observer switch
		bool flux = false;

		///< admittance observer switch
		bool admittance = false;

		///< modell variance
		float Q = 1e-5f;

		///< measurement variance
		float R = 1e-4f;

		///< flux observer feedback gains
		systems::dq C = {50.0f, 1.0f};

	} /* namespace observer */

	/**
	 * @namespace converter settings
	 */
	namespace converter
	{
		///< control period
		float ts = hardware::Tc;
		/**
		 * @namespace converter limits
		 */
		namespace limits
		{
			///< maximum phase current
			float current = 100.0f;

			///< maximum motor temperature
			float temperature = 90.0f;
		} /* namespace limits */
	} /* namespace converter */
} /* namespace setting */



