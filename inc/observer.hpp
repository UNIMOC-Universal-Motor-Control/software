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
	private:
		///< back emf vector
		systems::alpha_beta bemf;
		///< stator flux vector vector
		systems::alpha_beta stator_flux;
		///< flux error feedback vector
		systems::alpha_beta feedback;
		///< feedback controller for alpha flux
		control::pi ctrl_alpha;
		///< feedback controller for beta flux
		control::pi ctrl_beta;
	public:
		/**
		 * @brief flux observers trivial constructor
		 */
		flux(void);

	    /**
	     * @brief Get angular error from flux estimation.
	     * @param set_flux expected flux vector
	     * @retval out_flux estimated flux without inducted flux
	     */
		void Calculate(systems::alpha_beta& set_flux, systems::alpha_beta& out_flux);

		/**
		 * @brief set controller dynamic parameters.
		 *
		 *
		 * @param d		dampening of the observers bandpass
		 * @param wm	mid frequency of the observers bandpass
		 * @param ts 	sampling time
		 */
		constexpr inline void SetParameters(const float d, const float wm, const float ts)
		{
			ctrl_alpha.SetParameters(2.0f*d*wm, 2.0f*d/wm, ts);
			ctrl_beta.SetParameters(2.0f*d*wm, 2.0f*d/wm, ts);
		};

		///< @brief Reset controller and integral part to 0
		inline void Reset(void) { ctrl_alpha.Reset(); ctrl_beta.Reset();};
	};

	/**
	 * high frequency injection based observer
	 */
	class hfi
	{
	private:

	public:
	    /**
	     * add the injection pattern to the output voltage
	     * @param u_in stator frame voltage vector
	     * @param u_out stator frame voltage vectors with injection
	     */
	    static void Injection(const systems::alpha_beta u_in, std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& u_out);

		/**
		 * @brief calculate the mean stator admittance.
		 *
		 * @note call only once per control cycle
		 *
		 * @retval mean stator admittance.
		 */
		static systems::alpha_beta GetMean(std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& ad);

		/**
		 * @brief calculate the stator admittance vector.
		 *
		 * @note call only once per control cycle
		 *
		 * @retval stator admittance vector.
		 */
		static systems::alpha_beta GetVector(std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& ad);
	};

	/**
	 * hall sensor based observer
	 */
	class hall
	{
	private:

		///< current theta offset
		float offset;

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
		void SetOffset(const float new_offset);

	    /**
	     * @brief Get sine and cosine values from hall for estimation.
	     * @retval sin_cos sine and cosine of the hall sensors
	     */
	    void Calculate(systems::sin_cos& sc);
	};
} /* namespace observer */

#endif /* INC_OBSERVER_HPP_ */


