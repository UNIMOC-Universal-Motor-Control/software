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

		///< feedback sensor zero position value
		std::uint16_t zero_pos;
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
		std::uint32_t P;

		/**
		 * motor limits
		 */
		struct limits_s
		{
			///< maximum coil current
			float i;

			/**
			 * motor speed limits
			 */
			struct omega_s
			{
				///< backwards limit
				float backwards;

				///< forwards limit
				float forwards;
			} omega;

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
			/**
			 * battery limits
			 */
			struct voltage_s
			{
				///< low voltage limit
				float low;

				///< high voltage limit
				float high;
			} voltage;

			/**
			 * current limits
			 */
			struct i_s
			{
				///< maximum drive current
				float drive;

				///< maximum charge current
				float charge;
			}i;
		} limits;
	} battery;

	/**
	 * control settings
	 */
	struct control_s
	{
		/**
		 * current controller settings
		 */
		struct current_s
		{
			///< current control switch
			bool active;

			///< feedforward omega
			bool feedforward;

			///< proportional gain
			float kp;

			///< controller time constant
			float tn;
		}current;
	} control;

	/**
	 * observer settings
	 */
	struct observer_s
	{
		/**
		 * rotor flux observer settings
		 */
		struct flux_s
		{
			///< enable observer switch
			bool enable;

			///< flux observer feedback gains
			systems::dq C;
		} flux;

		/**
		 * high frequency injection observer settings
		 */
		struct hfi_s
		{
			///< enable observer switch
			bool enable;

			///< injection current in A
			float current;

			///< maximum speed where hfi is active
			float omega_max;
		} hfi;

		/**
		 * hall observer settings
		 */
		struct hall_s
		{
			///< enable observer switch
			bool enable;

			///< maps the hall sensor inputs to phases
			struct map_s
			{
				///< mapps hall sensor to phase a
				std::uint8_t a;
				///< mapps hall sensor to phase a
				std::uint8_t b;
				///< mapps hall sensor to phase a
				std::uint8_t c;

				std::uint8_t unused;
			} map;

			///< maximum speed where hall is active
			float omega_max;

			///< flux observer feedback gains in hall mode
			systems::dq C;

			///< angular offset for the hall sensor signals in q31 (-180 - 180)
			std::int32_t offset;
		} hall;

		/**
		 * mechanic observer settings
		 */
		struct mech_s
		{
			///< electrical torque minimal current
			float i_min;

			///< modell variance
			float Q;

			///< measurement variance
			float R;
		} mech;
	} observer;

	/**
	 * converter settings
	 */
	struct converter_s
	{
		///< compensated dead time in PWM switching -1 represents 0 and 1 represents 1
		std::uint32_t deadtime;

		///< pwm frequency
		std::uint32_t frequency;

		///< maps the phase wires to internal phases
		struct map_s
		{
			///< mapps phase wire to internal phase a
			std::uint8_t a;
			///< mapps phase wire to internal phase b
			std::uint8_t b;
			///< mapps phase wire to internal phase c
			std::uint8_t c;

			std::uint8_t unused;
		} map;

		/**
		 * converter derating settings
		 */
		struct derating_s
		{
			///< temperature derating starts x °C before temp limit.
			float temprature;

			///< voltage derating starts x Volts before Voltage limit.
			float voltage;

			///< omega derating starts x rad/s before omega limit.
			float omega;
		} derating;

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

	/**
	 * motor throttle setting
	 */
	struct throttle_s
	{
		/**
		 * motor throttle deadzone
		 */
		struct deadzone_s
		{
			///< low deadzone
			float low;

			///< high deadzone
			float high;
		}deadzone;

		///< Throttle signal source selection
		enum select_e
		{
			NONE,							// no throttle at all
			ANALOG_BIDIRECTIONAL,			// analog input as bidirectional Throttle with midpoint 0
			ANALOG_FORWARDS,				// analog input as unidirectional Throttle running only forwards
			ANALOG_BACKWARDS,				// analog input as unidirectional Throttle running only backwards
			ANALOG_SWITCH,					// analog input as unidirectional Throttle with switch input to select direction
			PAS,							// Pedal assist power as throttle
		} sel;

	} throttle;

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

