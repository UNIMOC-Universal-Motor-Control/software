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
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"
#include "observer.hpp"
#include "management.hpp"
#include "hardware_interface.hpp"

/**
 * @namespace controller classes
 */
namespace control
{


	/**
	 * derate control input envelope
	 * @param limit	value to end derating
	 * @param envelope derating envelope
	 * @param actual actual value
	 * @return 1 when no derating active and 1 to 0 when in envelope and 0 when above limit
	 */
	float Derate(const float limit, const float envelope, const float actual);

	/**
	 * limit the input value
	 * @param in input value
	 * @param min minimal value
	 * @param max maximal value
	 * @return true when value is out of limits
	 */
	bool Limit(float& in, const float min, const float max);

	/**
	 * SVPWM Overmodulation modified to flat bottom.
	 * @param u phase voltages
	 * @param ubat battery voltage
	 * @return dutys in -1 to 1
	 */
	void Overmodulation(systems::abc& u, float ubat, systems::abc& dutys);

	/**
	 * pi controller with anti windup
	 */
	class pi
	{
	private:
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
		float SetKi(const float kp, const float ts, const float new_tn) { return kp*ts/new_tn; };
	public:
		///< positive output limit (not necessarily positive)
		float			positive_limit;
		///< negative output limit (not necessarily negative)
		float			negative_limit;

		/**
		 * @brief Pi controller constructor with all essential parameters.
		 *
		 * @param new_kp                proportional gain.
		 * @param new_tn                integral action time.
		 * @param new_positive_limit    positive output limit.
		 * @param new_negative_limit    negative output limit.
		 * @param ts					sampling time
		 */
		pi(const float kp, const float tn,
				const float positive_limit, const float negative_limit, const float ts);
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
		 * @param ts					sampling time.
		 */
		constexpr inline void SetParameters(const float new_kp, const float new_tn, const float ts) { kp = new_kp; ki = SetKi(kp, ts, new_tn); };

		///< @brief Reset controller and integral part to 0
		constexpr inline void Reset(void) {error_sum = 0.0f;  output_unlimited = 0.0f; output = 0.0f;};
	};

	/**
	 * field orientated current controller
	 */
	class foc
	{
	private:
		///< d axis current controller
		pi ctrl_d;

		///< q axis current controller
		pi ctrl_q;

		static constexpr float _1bysqrt3 = 1.0f / std::sqrt(3.0f);
	public:
		/**
		 * @brief constructor of the foc with all essential parameters.
		 *
		 */
		foc(void);
		/**
		 * @brief calculate foc current controller
		 */
		void Calculate(systems::dq& setpoint);

		/**
		 * @brief set controller dynamic parameters.
		 *
		 *
		 * @param kp	Proportional gain of the PI controllers
		 * @param tn	zero crossing of the PI controller (Time constant)
		 * @param ts 	sampling time
		 */
		constexpr inline void SetParameters(const float kp, const float tn, const float ts)
		{
			ctrl_d.SetParameters(kp, tn, ts);
			ctrl_q.SetParameters(kp, tn, ts);
		};

		///< @brief Reset controller and integral part to 0
		inline void Reset(void) { ctrl_d.Reset(); ctrl_q.Reset();};
	};

	/**
	 * generic FOC controller thread
	 */
	class thread : public chibios_rt::BaseStaticThread<512>
	{
	private:
		static constexpr float _3by2 = 3.0f/2.0f;
		observer::flux       	flux;
		observer::hall       	hall;
		control::foc      		foc;
		sensor::as5048b 		as5048;
		std::array<float, 3>   	correction;
		filter::low_pass		uq;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> i_abc;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> i_abc_ac;
		std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES> i_ab;
		std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES> i_ab_ac;
		std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES> u_ab;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> dutys;

	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		/**
		 * generic constructor
		 */
		thread();
	};
} /* namespace control */

#endif /* INC_CONTROLLER_HPP_ */

