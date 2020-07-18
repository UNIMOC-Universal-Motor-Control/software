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
#include <cstring>
#include "hardware_interface.hpp"

/**
 * @namespace system settings
 *
 * @note aligned to 32bytes boundarys for better cache handling
 */
settings_ts settings =
{
	/**
	 * mechanic system settings
	 */
	.mechanics =
	{
		///< inertia of of rotor and connected known mechanics
		.J = 1e-4f,
	},

	/**
	 * motor settings
	 */
	.motor =
	{
		///< Stator resistance
		.rs = 250e-3f,

		///< anisotropic inductance vector
		.l = {400e-6f, 450e-6f},

		///< magnetic flux inducted voltage in rotor
		.psi = unit::RpmV2VsRad(75.0f) / (20.0f / 2.0f),

		///< number of pole pairs
		.P = 20 / 2,

		/**
		 * motor limit settings
		 */
		.limits =
		{
			///< maximum coil current
			.current = 100.0f,

			///< maximum angular velocity
			.w = unit::RadS(15000.0f),

			///< maximum motor temperature
			.temperature = 80.0f,
		},
	},

	/**
	 * battery settings
	 */
	.battery =
	{
		///< internal resistance
		.Ri = 75e-3,

		/**
		 * battery limits
		 */
		.limits =
		{
			///< maximum drive current
			.drive_current = 30.0f,

			///< maximum charge current
			.charge_current = 10.0f,
		},
	},

	/**
	 * control settings
	 */
	.control =
	{
		/**
		 * current controller settings
		 */
		.current =
		{
			///< current control switch
			.active = false,

			///< gain of the current controller
			.gain = 1.0f,
		},
	},


	/**
	 * observer settings
	 */
	.observer =
	{
		///< flux observer switch
		.flux = false,

		///< modell variance
		.Q = 1e-5f,

		///< measurement variance
		.R = 1e-4f,

		///< flux observer feedback gains
		.C = {50.0f, 1.0f},
	},

	/**
	 * converter settings
	 */
	.converter =
	{
		/**
		 * converter limits
		 */
		.limits =
		{
			///< maximum phase current
			.current = 110.0f,

			///< maximum power stage temperature
			.temperature = 90.0f,
		},
	},


	/**
	 * uavcan settings
	 */
	.uavcan =
	{
		///< node id
		.node_id = 2,

		///< drive id for commands 0 to 3
		.drive_id = 0,
	},


	///< crc32 value for the hole settings
	.crc = 0,
};


/**
 * Save setting to non volatile memory
 */
void settings_s::Save(void)
{
	settings.crc = hardware::memory::Crc32(&settings, sizeof(settings_ts) - sizeof(uint32_t));

	hardware::memory::Write(0, &settings, sizeof(settings_ts));
}

/**
 * Load settings from non volatile memory
 * @return false for crc error
 */
bool settings_s::Load(void)
{
	bool result = false;
	// align to cache lines for better cache handling
	__attribute__((aligned (32))) settings_ts tmp;

	hardware::memory::Read(0, &tmp, sizeof(settings_ts));

	if(tmp.crc == hardware::memory::Crc32(&tmp, sizeof(settings_ts) - sizeof(uint32_t)))
	{
		std::memcpy(&settings, &tmp, sizeof(settings_ts));
		result = true;
	}
	return result;
}

const parameter_ts parameters[] =
{
	{.setting = (void*)&settings.motor.rs, .name = "motor.rs", .default_val = 0.2f, .min_val = 1e-3f, .max_val = 10.0f, .type = parameter_s::type_e::PARAM_TYPE_FLOAT},
};

