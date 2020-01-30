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

/**
 * @namespace controller classes
 */
namespace controller
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
	static inline
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
	 * @param error         control loop error.
	 * @param feed_forward  feed forward control input.
	 * @retval controller output
	 */
	float Calculate(float  error, float feedforward);

	/**
	 * @brief set controller dynamic parameters.
	 *
	 * @param new_kp				proportional gain.
	 * @param new_tn				integral action time.
	 */
	inline void SetParameters(const float new_kp, const float new_tn) { kp = new_kp; ki = SetKi(kp, ts, new_tn); };

	///< @brief Reset controller and integral part to 0
	inline void Reset(void) {error_sum = 0.0f;  output_unlimited = 0.0f; output = 0.0f;};
};

/**
 * field orientated current controller
 */
class foc
{
private:
	///< d-current controller
	pi d;
	///< q-current controller
	pi q;
public:
	/**
	 * @brief FOC controller constructor with all essential parameters.
	 *
	 * @param new_ts                set sample time.
	 * @param new_kp                proportional gain.
	 * @param new_tn                integral action time.
	 */
	foc(const float new_ts, const float new_kp, const float new_tn);
	/**
	 * @brief calculate foc current controller
	 */
	void Calculate(void);

	/**
	 * @brief set controller dynamic parameters.
	 *
	 * @param new_kp                proportional gain.
	 * @param new_tn                integral action time.
	 */
	inline void SetParameters(const float new_kp, const float new_tn) { d.SetParameters(new_kp, new_tn); q.SetParameters(new_kp, new_tn); };

	///< @brief Reset controller and integral part to 0
	inline void Reset(void) { d.Reset(); q.Reset(); };
};

} /* namespace controller */

#endif /* INC_CONTROLLER_HPP_ */

