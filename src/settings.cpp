/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2022 Alexander <tecnologic86@gmail.com> Evers

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

#pragma GCC push_options
#pragma GCC optimize ("-O0")

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
		.rs = 0.15f,

		///< anisotropic inductance vector
		.l = {0.00022f, 0.00025f},

		///< magnetic flux inducted voltage in rotor
		.psi = 0.0019,

		///< number of pole pairs
		.P = 7,

		/**
		 * motor limit settings
		 */
		.limits =
		{
			///< maximum coil current
			.i = 40.0f,

			/**
			 * motor speed limits
			 */
			.omega =
			{
				///< backwards limit
				.backwards = -1000.0f,

				///< forwards limit
				.forwards = 1000.0f,
			},

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
			/**
			 * battery limits
			 */
			.voltage =
			{
				///< low voltage limit
				.low = 29.0f,

				///< high voltage limit
				.high = 45.0f,
			},

			.i =
			{
				///< maximum drive current
				.drive = 15.0f,

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

			///< flux observer feedback gains
			.C = {50.0f, 1.0f},
		},

		/**
		 * high frequency injection observer settings
		 */
		.hfi =
		{
			///< enable observer switch
			.enable = false,

			///< injection current in A
			.current = 1.5f,

			///< maximum speed where hfi is active
			.omega = 150.0f,

			///< hyseresis around maximum speed where observer is switched
			.hysteresis = 50.0f,
		},

		/**
		 * hall observer settings
		 */
		.hall =
		{
			///< enable observer switch
			.enable = false,

			///< maps the hall sensor inputs to phases
			.map =
			{
				///< mapps hall sensor to phase a
				.a = 0b001,
				///< mapps hall sensor to phase a
				.b = 0b010,
				///< mapps hall sensor to phase a
				.c = 0b100,

				.unused = 0,
			},
			///< maximum speed where hall is active
			.omega = 150.0f,

			///< hyseresis around maximum speed where observer is switched
			.hysteresis = 50.0f,

			///< flux observer feedback gains in hall mode
			.C = {25.0f, 25.0f},

			///< angular offset for the hall sensor signals in q31 (-180 - 180)
			.offset = 0,
		},

		/**
		 * mechanic observer settings
		 */
		.mech =
		{
			///< electrical torque minimal current
			.i_min = 2.0f,

			///< modell variance
			.Q = 1e-5f,

			///< measurement variance
			.R = 1e-4f,
		},
	},

	/**
	 * converter settings
	 */
	.converter =
	{
		///< dead time in PWM switching in ns
		.deadtime = 1000,

		///< pwm frequency
		.frequency = 8000,

		///< maps the phase wires to internal phases
		.map =
		{
			///< mapps phase wire to internal phase a
			.a = 0b001,
			///< mapps phase wire to internal phase b
			.b = 0b010,
			///< mapps phase wire to internal phase c
			.c = 0b100,
			.unused = 0,
		},

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

	/**
	 * motor throttle setting
	 */
	.throttle =
	{
		/**
		 * motor throttle deadzone
		 */
		.deadzone =
		{
			///< low deadzone
			.low = 0.0f,

			///< high deadzone
			.high = 1.0f,
		},

		///< Throttle signal source selection
		.sel = settings_ts::throttle_s::NONE,
	},

	/**
	 * crank settings
	 */
	.crank =
	{
		///< crank torque sensor gain in Nm/Volts
		.gain = 40.0f*9.81f*0.17f,

		///< crank torque sensor offset in Volts
		.offset = 1.55f,

		///< torque sensor command enable
		.enable = true,

		.pas =
		{
			///< pas counts per revolution both edges
			.counts = 32,

			///< pas mode enable
			.enable = true,
		},
	},

	/**
	 * @struct cyphal_s
	 * @brief cyphal specific values
	 */
	.cyphal =
	{
		///< ID of the node in UAVCAN
		.node_id = 42,//std::numeric_limits<std::uint16_t>::max(),

		///< CAN Bus nominal bit rate.
		.nbitrate = 500000,

		///< true if CAN-FD is enabled.
		.fd = false,

		///< CAN Bus data bit rate.
		.dbitrate = 4000000,

		/**
		 * @struct subject_s
		 * @brief setable subject IDs of messages
		 *
		 */
		.subject =
		{
			/**
			 * @struct servo_s
			 * @brief subject IDs of servo related messages
			 *
			 */
			.servo =
			{
				///< subject ID for servo feedback message
				.feedback = std::numeric_limits<std::uint16_t>::max(),

				///< subject ID for servo dynamics message
				.dynamics = std::numeric_limits<std::uint16_t>::max(),

				///< subject ID for servo power message
				.power = std::numeric_limits<std::uint16_t>::max(),

				///< subject ID for servo setpoint
				.setpoint = std::numeric_limits<std::uint16_t>::max(),

				///< subject ID for servo readiness (Armed state)
				.readiness = std::numeric_limits<std::uint16_t>::max(),
			},
		},
	},

	///< crc32 value for the hole settings
	.crc = 0,
};

#pragma GCC pop_options


/**
 * Save setting to non volatile memory
 */
void settings_s::Save(void)
{
	settings.crc = hardware::memory::Crc32(&settings, offsetof(settings_ts, crc));

	hardware::memory::Write(&settings, sizeof(settings_ts));
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

 	hardware::memory::Read(&tmp, sizeof(settings_ts));

	if(tmp.crc == hardware::memory::Crc32(&tmp, offsetof(settings_ts, crc)))
	{
		std::memcpy(&settings, &tmp, sizeof(settings_ts));
		result = true;
	}
	return result;
}

