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
#include "values.hpp"



values_ts values =
{
	.motor =
	{
		///< Admittance mean
		.y = {0.0f, 0.0f},

		///< electric torque
		.m_el = 0.0f,

		///< external load torque
		.m_l = 0.0f,

		///< motor temperature
		.temp = 0.0f,

		///< motor phase current
		.i = {0.0f, 0.0f, 0.0f},

		///< motor rotor system values
		.rotor =
		{
			///< Current in rotor frame
			.i = {0.0f, 0.0f},

			///< Voltage in rotor frame
			.u = {0.0f, 0.0f},

			///< Admittance vector in rotor frame
			.y = {0.0f, 0.0f},

			///< angular velocity in rotor frame
			.omega = 0.0f,

			///< rotor angle
			.phi = 0.0f,

			///< motor rotor system setpoints
			.setpoint =
			{
				///< Current setpoint in rotor frame
				.i = {0.0f, 0.0f},

				///< angular velocity setpoint in rotor frame
				.omega = 0.0f,

				///< rotor angle setpoint
				.phi = 0.0f,
			},
		},
	},

	///< battery values
	.battery =
	{
		///< Battery voltage
		.u = 10.0f,

		///< Battery current
		.i = 0.0f,
	},

	///< converter values
	.converter =
	{
		///< power stage temperature
		.temp = 0.0f,
	},
};



