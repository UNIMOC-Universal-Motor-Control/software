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
#include <cstddef>
#include "hardware_interface.hpp"

/**
 * @namespace system settings
 *
 * @note aligned to 32bytes boundarys for better cache handling
 */
__attribute__((aligned (32))) settings_ts settings =
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
		.rs = 0.243666f,

		///< anisotropic inductance vector
		.l = {0.000246604f, 0.000335957f},

		///< magnetic flux inducted voltage in rotor
		.psi = unit::RpmV2VsRad(75.0f) / (30.0f / 2.0f),

		///< number of pole pairs
		.P = 30 / 2,

		///< starting direct current
		.i_start = 0.0f,

		///< hall sensor state change angles table
		.hall_table = {0.0f},

		/**
		 * motor limit settings
		 */
		.limits =
		{
			///< maximum coil current
			.i = 10.0f,

			///< maximum angular velocity
			.omega = 1000.0f,

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
			///< low voltage battery limit
			.voltage = 29.0f,

			.i =
			{
				///< maximum drive current
				.drive = 2.0f,

				///< maximum charge current
				.charge = 0.0f,
			}
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

			///< feedforward omega
			.feedforward = true,

			///< proportional gain
			.kp = 0.0f,

			///< controller time constant
			.tn = 1000.0f,
		},
	},


	/**
	 * observer settings
	 */
	.observer =
	{
		/**
		 * rotor flux observer settings
		 */
		.flux =
		{
			///< enable observer switch
			.enable = false,

			///< modell variance
			.Q = 1e-5f,

			///< measurement variance
			.R = 1e-4f,

			///< flux observer feedback gains
			.C = {50.0f, 1.0f},
		},

		/**
		 * hall observer settings
		 */
		.hall =
		{
			///< enable observer switch
			.enable = false,

			///< modell variance
			.Q = 1e-5f,

			///< measurement variance
			.R = 1e5f,
		},

		/**
		 * mechanic observer settings
		 */
		.mech =
		{
			///< electrical torque minimal current
			.i_min = 2.0f,
		},
	},

	/**
	 * crank settings
	 */
	.crank =
	{
		///< crank torque sensor gain
		.gain = 0.3f,

		///< crank torque sensor offset
		.offset = 0.84f,

		///< torque sensor command enable
		.enable = false,

		.pas =
		{
			///< pas counts per revolution both edges
			.counts = 32,

			///< pas mode enable
			.enable = false,
		},
	},

	/**
	 * converter settings
	 */
	.converter =
	{
		///< dead time in PWM switching in ns
		.deadtime = 800,

		///< minimal current for full dead time compensation
		.dt_i_min = 1.0f,

		///< pwm frequency
		.frequency = 16000,

		/**
		 * converter derating settings
		 */
		.derating =
		{
			///< temperature derating starts x °C before temp limit.
			.temprature = 10.0f,

			///< voltage derating starts x Volts before Voltage limit.
			.voltage = 3.0f,

			///< omega derating starts x rad/s before omega limit.
			.omega = 50.0f,
		},

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

	///< crc32 value for the hole settings
	.crc = 0,
};


/**
 * Save setting to non volatile memory
 */
void settings_s::Save(void)
{
	settings.crc = hardware::memory::Crc32(&settings, offsetof(settings_ts, crc));

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

	if(tmp.crc == hardware::memory::Crc32(&tmp, offsetof(settings_ts, crc)))
	{
		std::memcpy(&settings, &tmp, sizeof(settings_ts));
		result = true;
	}
	return result;
}

