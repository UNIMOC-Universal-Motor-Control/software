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
#ifndef INC_CONTROLLER_HPP_
#define INC_CONTROLLER_HPP_

#include <cstdint>
#include <cmath>
#include <climits>
#include "ch.hpp"
#include "filter.hpp"
#include "systems.hpp"

/**
 * @namespace controller classes
 */
namespace control
{

	/**
	 * pi controller with anti windup
	 */
	class pi
	{
	private:
		///< sample time (call timing of controller )
		const float 	ts;
		///< integral error sum
		float 			error_sum;
		///< unlimited controller output
		float 			output_unlimited;
		///< limited controller output
		float 			output;
		///< proportional controller gain
		float			kp;
		///< integral action time  (compensated time constant)
		float			ki;			// integral gain

		///< internal equation for ki based on kp and tn (already takes sample time into account)
		constexpr static inline
		float SetKi(float kp, float ts, float new_tn) { return kp*ts/new_tn; };
	public:
		///< positive output limit (not necessarily positive)
		float			positive_limit;
		///< negative output limit (not necessarily negative)
		float			negative_limit;

		/**
		 * @brief Pi controller constructor with all essential parameters.
		 *
		 * @param new_ts                set sample time.
		 * @param new_kp                proportional gain.
		 * @param new_tn                integral action time.
		 * @param new_positive_limit    positive output limit.
		 * @param new_negative_limit    negative output limit.
		 */
		pi(const float ts, const float kp, const float tn,
				const float positive_limit, const float negative_limit);
		/**
		 * @brief calculate regulator equation with feed forward and anti windup.
		 *
		 * @param setpoint				setpoint
		 * @param act					actual
		 * @param feed_forward  feed forward control input.
		 * @retval controller output
		 */
		float Calculate(const float setpoint, const float actual, const float feedforward);

		/**
		 * @brief set controller dynamic parameters.
		 *
		 * @param new_kp				proportional gain.
		 * @param new_tn				integral action time.
		 */
		constexpr inline void SetParameters(const float new_kp, const float new_tn) { kp = new_kp; ki = SetKi(kp, ts, new_tn); };

		///< @brief Reset controller and integral part to 0
		constexpr inline void Reset(void) {error_sum = 0.0f;  output_unlimited = 0.0f; output = 0.0f;};
	};

	/**
	 * complex current controller for rotor frame
	 *
	 * The motor in rotor frame is G_m = 1/(L_s (s + j w) + R_s)
	 * The controller in rotor frame is G_c = (K_p (s + j w) + K_i)/s
	 *
	 * so to get G_o = 1/s for the open loop, K_p = L_s and K_i = R_s
	 */
	class complex_current
	{
	private:
		///< sample time (call timing of controller )
		const float 	ts;
		///< integral error sum
		systems::dq		error_sum;
		///< unlimited controller output
		systems::dq		output_unlimited;
		///< limited controller output
		systems::dq		output;
		///< proportional controller gain
		systems::dq		kp;
		///< integral action time  (compensated time constant)
		float			ki;			// integral gain
		///< rotor flux constant for feed forward
		float 			psi;
	public:
		///< output limit
		float			limit;


		/**
		 * @brief complex current controller constructor with all essential parameters.
		 *
		 * The motor in rotor frame is G_m = 1/(L_s (s + j w) + R_s)
		 * The controller in rotor frame is G_c = (K_p (s + j w) + K_i)/s
		 *
		 * so to get G_o = 1/s for the open loop, K_p = L_s and K_i = R_s
		 *
		 * @param new_ts                set sample time.
		 * @param rs	                series resistance of the winding
		 * @param l                		series inductance of the winding
		 * @param psi                	rotor flux constant
		 * @param new_limit   			output limit.
		 */
		complex_current(const float new_ts, const float rs, const systems::dq l, const float new_psi, const float new_limit);

		/**
		 * @brief calculate regulator equation with feed forward and anti windup.
		 * @param setpoint				setpoint vector
		 * @param act					actual current vector
		 * @param omega					angular velocity in rad/s of the motor
		 * @return						controllers output voltage vector
		 */
		systems::dq Calculate(const systems::dq setpoint, const systems::dq act, const float omega);

		/**
		 * @brief set controller dynamic parameters.
		 *
		 * The motor in rotor frame is G_m = 1/(L_s (s + j w) + R_s)
		 * The controller in rotor frame is G_c = (K_p (s + j w) + K_i)/s
		 *
		 * so to get G_o = 1/s for the open loop, K_p = L_s and K_i = R_s
		 *
		 * @param rs	                series resistance of the winding
		 * @param l                		series inductance of the winding
		 * @param psi                	rotor flux constant
		 */
		constexpr inline void SetParameters(const float rs, const systems::dq l, const float new_psi) { kp.d = l.d; kp.q = l.q; ki = rs; psi = new_psi;};

		///< @brief Reset controller and integral part to 0
		constexpr inline void Reset(void) {error_sum = {0.0f, 0.0f};  output_unlimited = {0.0f, 0.0f}; output = {0.0f, 0.0f};};
	};

	/**
	 * smith predictor with pi controller and anti windup
	 */
	class smith_predictor_current
	{
	private:
		///< sample time (call timing of controller )
		const float ts;

		///< series resistance of the winding
		float rs;

		///< series inductance of the winding
		systems::dq l;

		///< rotor flux constant
		float psi;

		///< quality factor of the hardware filter model
		const float hwQ;

		///< corner frequency of the hardware filter model
		const float hwFc;

		///< quality factor of the software filter and its model
		const float fQ;

		///< corner frequency of the software filter and its model
		const float fFc;

		///< current-controller
		complex_current ctrl;

		///< current filter d-axis
		filter::biquad fd;

		///< current filter q-axis
		filter::biquad fq;

		///< omega filter for feed forward
		filter::biquad fomega;

		///< hardware current filter d-axis
		filter::biquad hwd;

		///< hardware current filter q-axis
		filter::biquad hwq;

		///< smith predictor model of the current filter d-axis
		filter::biquad sd;

		///< smith predictor model of the current filter q-axis
		filter::biquad sq;

		///< output voltage vector
		systems::dq u;

		///< current estimate of the smith predictor model
		systems::dq i;

		///< current estimate of the smith predictor model after the filter
		systems::dq i_filtered;

	public:
		///< output limit
		float			limit;

		/**
		 * @brief smith predictor with complex current controller
		 * 		  constructor with all essential parameters.
		 *
		 * The motor in rotor frame is G_m = 1/(L_s (s + j w) + R_s)
		 * The controller in rotor frame is G_c = (K_p (s + j w) + K_i)/s
		 *
		 * so to get G_o = 1/s for the open loop, K_p = L_s and K_i = R_s
		 *
		 * @param new_ts                set sample time.
		 * @param rs	                series resistance of the winding
		 * @param l                		series inductance of the winding
		 * @param psi                	rotor flux constant
		 * @param new_limit   			output limit.
		 * @param new_hwQ				quality factor of the hardware filter model
		 * @param new_hwFc				corner frequency of the hardware filter model
		 * @param new_fQ				quality factor of the software filter and its model
		 * @param new_fFc				corner frequency of the software filter and its model
		 *
		 */
		smith_predictor_current(const float new_ts, const float new_rs, const systems::dq new_l, const float new_psi,
				const float new_limit, const float new_hwQ, const float new_hwFc, const float new_fQ, const float new_fFc);

		/**
		 * @brief calculate regulator equation with feed forward and anti windup.
		 * @param setpoint				setpoint vector
		 * @param act					actual current vector
		 * @param omega					angular velocity in rad/s of the motor
		 * @return						controllers output voltage vector
		 */
		systems::dq Calculate(const systems::dq setpoint, const systems::dq act, const float omega);

		/**
		 * @brief set controller dynamic parameters.
		 *
		 * The motor in rotor frame is G_m = 1/(L_s (s + j w) + R_s)
		 * The controller in rotor frame is G_c = (K_p (s + j w) + K_i)/s
		 *
		 * so to get G_o = 1/s for the open loop, K_p = L_s and K_i = R_s
		 *
		 * @param rs	                series resistance of the winding
		 * @param l                		series inductance of the winding
		 * @param psi                	rotor flux constant
		 */
		constexpr inline void SetParameters(const float new_rs, const systems::dq new_l, const float new_psi)
		{
			rs = new_rs;
			l = new_l;
			psi = new_psi;
			ctrl.SetParameters(rs, l, psi);
		};

		///< @brief Reset controller and integral part to 0
		constexpr inline void Reset(void) { ctrl.Reset(); };

	};

	/**
	 * field orientated current controller
	 */
	class foc
	{
	private:
		///< current controller with complex pole and smith predictor
		smith_predictor_current ctrl;
	public:
		/**
		 * @brief constructor of the foc with all essential parameters.
		 *
		 * @param hwQ				    quality factor of the hardware filter model
		 * @param hwFc				    corner frequency of the hardware filter model
		 * @param fQ				    quality factor of the software filter and its model
		 * @param fFc				    corner frequency of the software filter and its model
		 *
		 */
		foc(const float hwQ, const float hwFc, const float fQ, const float fFc);
		/**
		 * @brief calculate foc current controller
		 */
		void Calculate(void);

		/**
		 * @brief set controller dynamic parameters.
		 *
		 *
		 * @param rs	                series resistance of the winding
		 * @param l                		series inductance of the winding
		 * @param psi                	rotor flux constant
		 */
		constexpr inline void SetParameters(const float rs, const systems::dq l, const float psi) 	{	ctrl.SetParameters(rs, l, psi);		};

		///< @brief Reset controller and integral part to 0
		inline void Reset(void) { ctrl.Reset(); };
	};


} /* namespace control */

#endif /* INC_CONTROLLER_HPP_ */

