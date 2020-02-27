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
 * settings to be saved in non volatile memory
 */
typedef struct settings_s
{

	/**
	 * mechanical properties of the system
	 */
	struct mechanics_s
	{
		///< inertia of of rotor and connected known mechanics
		float J;
	}  mechanics;

	/**
	 * motor properties
	 */
	struct motor_s
	{
		///< Stator resistance
		float rs;

		///< anisotropic inductance vector
		systems::dq l;

		///< velocity constant Kv
		float psi;

		///< number of pole pairs
		uint32_t P;

		///< square injection voltage amplitude
		float u_inj;

		/**
		 * motor limits
		 */
		struct limits_s
		{
			///< maximum coil current
			float current;

			///< maximum angular velocity
			float w;

			///< maximum motor temperature
			float temperature;
		} limits;
	} motor;

	/**
	 * battery properties
	 */
	struct battery_s
	{
		///< internal resistance
		float Ri;

		/**
		 * battery limits
		 */
		struct limits_s
		{
			///< maximum drive current
			float drive_current;

			///< maximum charge current
			float charge_current;
		} limits;
	} battery;

	/**
	 * control settings
	 */
	struct control_s
	{
		///< current control switch
		bool current;

	} control;

	/**
	 * observer settings
	 */
	struct observer_s
	{
		///< flux observer switch
		bool flux;

		///< admittance observer switch
		bool admittance;

		///< modell variance
		float Q;

		///< measurement variance
		float R;

		///< flux observer feedback gains
		systems::dq C;
	} observer;

	/**
	 * converter settings
	 */
	struct converter_s
	{
		///< control period
		float ts;

		/**
		 * converter limits
		 */
		struct limits_s
		{
			///< maximum phase current
			float current;

			///< maximum power stage temperature
			float temperature;
		} limits;
	}  converter;

	///< crc32 value for the hole settings
	uint32_t crc;


	/**
	 * Save setting to non volatile memory
	 */
	void Save(void);

	/**
	 * Load settings from non volatile memory
	 * @return false for crc error
	 */
	bool Load(void);
} settings_ts;

extern settings_ts settings;

#endif /* INC_SETTINGS_HPP_ */

