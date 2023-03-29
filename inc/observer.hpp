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
#ifndef INC_OBSERVER_HPP_
#define INC_OBSERVER_HPP_

#include "systems.hpp"
#include "filter.hpp"
#include "controller.hpp"
#include "hardware_interface.hpp"

/**
 * @namespace different observer classes
 */
namespace observer
{

	/**
	 * observer for the mechanic system
	 */
	class mechanic
	{
	private:
		///< new covariance matrix
		std::array<std::array<float, 3>, 3> p;
		///< current covariance matrix
		std::array<std::array<float, 3>, 3> pk;
		///< kalman gain
		std::array<float, 3> k;

		float s;
		float sigma;

		std::array<float, 3> out_error;

		float& torque_drive;
		float& torque_load;

		float& omega;
		std::int32_t& phi;

		float& J;
		float& Q;
		float& R;

		float& omega_limit_positive;
		float& omega_limit_negative;

	public:
		/**
		 * constructor with all internal references
		 * @param drive	torque input
		 * @param load torque output
		 * @param omega angular velocity
		 * @param phi angular position Q31
		 * @param J inertia
		 * @param Q kalman Q gain
		 * @param R kalman R gain
		 * @param P kalman initial covariance
		 * @param omega_limit_positive positive limit for omega
		 * @param omega_limit_negative negative limit for omega
		 */
		mechanic(float& drive,
				float& load,
				float& omega,
				std::int32_t& phi,
				float& J,
				float& Q,
				float& R,
				const float P,
				float& omega_limit_positive,
				float& omega_limit_negative);

		/**
		 * @brief calculate the mechanic model to estimate rotor angle.
		 * @param i rotor currents
		 */
		void Predict(void);

		/**
		 * @brief calculate the kalman correction.
		 * @param q model uncertainty
		 * @param r measurement uncertainty
		 * @param   angle error signal
		 * @retval  model error correction
		 */
		void Update(const float angle_error);
	};

	/**
	 * rotor flux based observer
	 */
	class flux
	{
	private:
		typedef struct {
			///< back emf vector
			systems::dq bemf;
			///< flux error feedback vector
			systems::dq feedback;
		} rotor_s;

		///< Rotor frame values
		rotor_s rotor;

		typedef struct {
			///< back emf vector
			systems::alpha_beta bemf;
			///< estimated flux without inducted flux
			systems::alpha_beta flux;
		} stator_s;

		///< stationary frame values
		stator_s stator;
	public:
		/**
		 * @brief flux observers trivial constructor
		 */
		flux(void);

	    /**
	     * @brief Get angular error from flux estimation.
	     * @param set_flux expected flux vector
	     * @param C feedback gains
	     * @retval out_flux estimated flux without inducted flux
	     */
	    void Calculate(systems::dq& set_flux, systems::dq& out_flux, const systems::dq C);

	};


	/**
	 * hall sensor based observer
	 */
	class hall
	{
	private:

		///< current theta offset
		std::int32_t offset;

		///< offsets sine cosine values for rotation
		systems::sin_cos sc_offset;
	public:
		/**
		 * @brief hall observers trivial constructor
		 */
		hall(void);

		/**
		 * Update the angle offset
		 * @param new_offset
		 */
		void SetOffset(const std::int32_t new_offset);

	    /**
	     * @brief Get sine and cosine values from hall for estimation in rotor frame.
	     * @retval est hall sensor signal in rotor frame
	     */
	    void Calculate(systems::dq& est);
	};
} /* namespace observer */

#endif /* INC_OBSERVER_HPP_ */


