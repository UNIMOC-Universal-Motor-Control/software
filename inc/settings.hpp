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

		///< starting direct current
		float i_start;

		/**
		 * motor limits
		 */
		struct limits_s
		{
			///< maximum coil current
			float i;

			///< maximum angular velocity
			float omega;

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
			///< low voltage battery limit
			float voltage;
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

			///< modell variance
			float Q;

			///< measurement variance
			float R;

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

			///< modell variance
			float Q;

			///< measurement variance
			float R;

			///< injection frequency in rad/s
			float frequency;

			///< injection current in A
			float current;
		} hfi;

		/**
		 * mechanic observer settings
		 */
		struct mech_s
		{
			///< electrical torque minimal current
			float i_min;
		} mech;
	} observer;

	/**
	 * crank settings
	 */
	struct crank_s
	{
		///< crank torque sensor gain in Nm/Volts
		float gain;

		///< crank torque sensor offset in Volts
		float offset;

		///< torque sensor command enable
		bool enable;

		struct pas_s
		{
			///< pas counts per revolution both edges
			std::uint32_t counts;

			///< pas mode enable
			bool enable;
		} pas;
	} crank;

	/**
	 * converter settings
	 */
	struct converter_s
	{
		///< compensated dead time in PWM switching -1 represents 0 and 1 represents 1
		float dt;

		///< minimal current for full dead time compensation
		float dt_i_min;

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

/**
 * parameter definition type for uavcan communication
 */
typedef struct parameter_s
{
	///< max length of name string in frame
	static constexpr std::size_t PARAM_MAX_NAME_LEN = 92;

	///< uavcan parameter type
	enum class type_e : std::uint_fast8_t
	{
		PARAM_TYPE_FLOAT,
		PARAM_TYPE_INT,
		PARAM_TYPE_BOOL
	};

	///< pointer to the setting value behind the parameter
	void* setting;

	///< name of the parameter
	const char* name;

	///< parameter default value
	float default_val;

	///< parameter min value
	float min_val;

	///< parameter max value
	float max_val;

	///< parameter type
	type_e type;
} parameter_ts;

extern const parameter_ts parameters[];



#endif /* INC_SETTINGS_HPP_ */

