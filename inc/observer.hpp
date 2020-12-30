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
#ifndef INC_OBSERVER_HPP_
#define INC_OBSERVER_HPP_

#include "systems.hpp"
#include "filter.hpp"
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
		///< 1.5
		static constexpr float _3by2 = 3.0f/2.0f;

		///< new covariance matrix
		float p[3][3];
		///< current covariance matrix
		float pk[3][3];
		///< kalman gain
		float k[3];

		float s;
		float sigma;


	public:
		/**
		 * @brief minimal constructor
		 */
		mechanic(void);

		/**
		 * @brief calculate the mechanic model to estimate rotor angle.
		 * @param i rotor currents
		 */
		static void Predict(const systems::dq& i);

		/**
		 * @brief correct the mechanic model to estimate rotor angle.
		 *
		 * @param error  state error feedback
		 */
		static void Correct(const std::array<float, 3> error);

		/**
		 * @brief calculate the kalman correction.
		 * @param q model uncertainty
		 * @param r measurement uncertainty
		 * @param   angle error signal
		 * @retval  model error correction
		 */
		void Update(const float Q, const float R, const float angle_error, std::array<float, 3>& out_error);
	};

	/**
	 * rotor flux based observer
	 */
	class flux
	{
	public:
	private:
		typedef struct {
			///< back emf vector
			systems::dq bemf;
			///< flux error feedback vector
			systems::dq feedback;
			///< estimated flux vector
			systems::dq flux;
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

		///< kalman filter for flux error signal
		observer::mechanic mech;
	public:
		/**
		 * @brief flux observers trivial constructor
		 */
		flux(void);

		/**
		 * @brief Get angular error from flux estimation.
		 * @param sin_cos sine and cosine of the actual rotor angle
		 * @retval kalman correction vector
		 */
		void Calculate(const systems::sin_cos& sin_cos, std::array<float, 3>& correction);
	};

} /* namespace observer */

#endif /* INC_OBSERVER_HPP_ */


