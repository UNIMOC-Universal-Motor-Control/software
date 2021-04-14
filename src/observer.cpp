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
#include "hardware_interface.hpp"
#include "observer.hpp"
#include "settings.hpp"
#include "values.hpp"
#include <cstdint>
#include <cmath>
#include <array>
#include <limits>

/**
 * @namespace observer classes
 */
namespace observer
{
    /**
     * @brief minimal constructor
     *
     * @param q model uncertainty
     * @param r measurement uncertainty
     */
    mechanic::mechanic(void) {}

    /**
     * @brief calculate the mechanic model to estimate rotor angle.
     * @param i rotor currents
     */
    void mechanic::Predict(const systems::dq& i)
    {
    	using namespace values::motor::rotor;
    	using namespace values::motor;

        // update constants
        const float ts = hardware::Tc();
        const float tsj = ts/settings.mechanics.J;

        // electric torque
        m_el = _3by2 * (settings.motor.psi * i.q +
        		(settings.motor.l.d - settings.motor.l.q) * i.d * i.q);

        // omega
        omega += tsj * (m_el - m_l);

        if(std::fabs(omega) > 2.0f * settings.motor.limits.omega)
        {
        	omega = std::copysign(2.0f * settings.motor.limits.omega, omega);
        }

        if(!std::isfinite(omega))
        {
        	omega = 0.0f;
        }

        // integrate omega for phi
        phi += unit::Q31(omega * ts);

        // integrate load
        //values.motor.m_l += 0;
    }

    /**
     * @brief correct the mechanic model to estimate rotor angle.
     *
     * @param error  state error feedback
     */
    void mechanic::Correct(const std::array<float, 3> error)
    {
    	using namespace values::motor::rotor;
    	using namespace values::motor;

        omega += error[0];
        phi += unit::Q31(error[1]);
        m_l += error[2];
    }

	/**
	 * @brief calculate the kalman correction.
	 * @param q model uncertainty
	 * @param r measurement uncertainty
	 * @param   angle error signal
	 * @retval  model error correction
	 */
	void mechanic::Update(const float Q, const float R, const float angle_error, std::array<float, 3>& out_error)
    {
        const float ts = hardware::Tc();
        const float tsj = ts/settings.mechanics.J;

        /// kalman filter for flux error
        pk[0][2] = p[0][2] - p[2][2] * tsj;

        pk[0][0] = p[0][0] + Q - p[2][0] * tsj - pk[0][2] * tsj;
        pk[1][0] = p[1][0] + p[0][0]*ts - tsj*(p[1][2] + p[0][2] * ts);
        pk[2][0] = p[2][0] - tsj*p[2][2];

        pk[0][1] = p[0][1] + ts * (p[0][0] - tsj * p[2][0]) - tsj * p[2][1];
        pk[1][1] = p[1][1] + Q + p[0][1]*ts + ts * (p[1][0] + p[0][0] * ts);
        pk[2][1] = p[2][1] + p[2][0] * ts;

        //p[0][2] precalculated
        pk[1][2] = p[1][2] + p[0][2] * ts;
        pk[2][2] = p[2][2] + Q;

        s = 1.0f / (pk[1][1] + R);
        k[0] = pk[0][1] * s;
        k[1] = pk[1][1] * s;
        k[2] = pk[2][1] * s;

        float k_1 = k[1] - 1.0f;
        p[0][0] = pk[0][0] - k[0] * pk[1][0];
        p[1][0] = - pk[1][0] * k_1;
        p[2][0] = pk[2][0] - k[2] * pk[1][0];

        p[0][1] = pk[0][1] - k[0] * pk[1][1];
        p[1][1] = - pk[1][1] * k_1;
        p[2][1] = pk[2][1] - k[2] * pk[1][1];

        p[0][2] = pk[0][2] - k[0] * pk[1][2];
        p[1][2] = - pk[1][2] * k_1;
        p[2][2] = pk[2][2] - k[2] * pk[1][2];

        out_error[0] = k[0] * angle_error;
        out_error[1] = k[1] * angle_error;
        out_error[2] = k[2] * angle_error;
    }

    /**
     * @brief flux observers trivial constructor
     */
    flux::flux(void):bemf(),feedback(), pi_alpha(0.0f, 0.0f, 0.0f, 0.0f, hardware::Tc()), pi_beta(0.0f, 0.0f, 0.0f, 0.0f, hardware::Tc())
    {}

    /**
     * @brief Get angular error from flux estimation.
     * @param set_flux expected flux vector
     * @param out_flux estimated flux without inducted flux
     * @retval kalman correction vector
     */
    void flux::Calculate(systems::alpha_beta& set_flux, systems::alpha_beta& out_flux)
    {
    	using namespace values::motor::stator;
    	using namespace values::motor;

    	// BEMF voltage and feedback
    	bemf.alpha = u.alpha - settings.motor.rs * i.alpha + feedback.alpha;
    	bemf.beta  = u.beta  - settings.motor.rs * i.beta  + feedback.beta;

    	// integrate bemf to flux
    	out_flux.alpha +=  bemf.alpha * hardware::Tc();
    	out_flux.beta  +=  bemf.beta  * hardware::Tc();

    	// sub the voltage inducted in the inductors of the stator
    	out_flux.alpha -= i.alpha * settings.motor.l.d;
    	out_flux.beta  -= i.beta  * settings.motor.l.q;

    	// compare actual flux with flux parameter
    	feedback.alpha = pi_alpha.Calculate(set_flux.alpha, out_flux.alpha, 0.0f);
    	feedback.beta  = pi_beta.Calculate(set_flux.beta, out_flux.beta, 0.0f);
    }

    /**
     * add the injection pattern to the output voltage
     * @param u_in stator frame voltage vector
     * @param u_out stator frame voltage vectors with injection
     */
    void hfi::Injection(const systems::alpha_beta u_in, std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& u_out)
    {
    	// add the injection pattern to the voltage output
    	float u_inj = settings.observer.hfi.current*hardware::Fc()*settings.motor.l.d;
    	u_out[0] = {u_in.alpha + u_inj, u_in.beta        };
    	u_out[1] = {u_in.alpha        , u_in.beta + u_inj};
    	u_out[2] = {u_in.alpha - u_inj, u_in.beta        };
    	u_out[3] = {u_in.alpha        , u_in.beta - u_inj};
    }


    /**
     * @brief calculate the mean stator admittance.
     *
     * @note call only once per control cycle
     *
     * @retval mean stator admittance.
     */
    systems::alpha_beta hfi::GetMean(std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& ad)
    {
        systems::alpha_beta y;
        // statonary part of the admittance vector, float sampling
        y.alpha = (ad[0].alpha + ad[1].beta  - ad[2].alpha - ad[3].beta);
        y.beta  = (ad[0].beta  - ad[1].alpha - ad[2].beta  + ad[3].alpha);

        return y;
    }

    /**
     * @brief calculate the stator admittance vector.
     *
     * @note call only once per control cycle
     *
     * @retval stator admittance vector.
     */
    systems::alpha_beta hfi::GetVector(std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& ad)
    {
        systems::alpha_beta yd;
        // rotating part of the admittance vector, float sampling
        yd.alpha = (ad[0].beta  + ad[1].alpha - ad[2].beta  - ad[3].alpha);
        yd.beta  = (ad[0].alpha - ad[1].beta  - ad[2].alpha + ad[3].beta);

        return yd;
    }


    /**
     * @brief hall observers trivial constructor
     */
    hall::hall(void)    {}

    /**
     * @brief Get sine and cosine values from hall for estimation.
     * @retval sin_cos sine and cosine of the hall sensors
     */
    void hall::Calculate(systems::sin_cos& sc)
    {
    	constexpr float sqrt3by2 = std::sqrt(3.0f)*0.5f;

    	switch(values::motor::hall::state)
    	{
    	default:
    	case 0b100:	// 0 deg.
    		sc.sin = 0.0f;
    		sc.cos = -1.0f;
    		break;
    	case 0b110:	// 60 deg.
    		sc.sin = -sqrt3by2;
    		sc.cos = -0.5f;
    		break;
    	case 0b010:	// 120 deg.
    		sc.sin = -sqrt3by2;
    		sc.cos = 0.5f;
    		break;
    	case 0b011:	// 180 deg.
    		sc.sin = 0.0f;
    		sc.cos = 1.0f;
    		break;
    	case 0b001:	// 240 deg.
    		sc.sin = sqrt3by2;
    		sc.cos = 0.5f;
    		break;
    	case 0b101:	// 300 deg.
    		sc.sin = sqrt3by2;
    		sc.cos = 0.5f;
    		break;
    	}

    	// advance angle by offset
    	systems::alpha_beta tab = {sc.sin, sc.cos};
    	systems::dq tdq = systems::transform::Park(tab, sc_offset);

    	sc.sin = tdq.d;
    	sc.cos = tdq.q;
    }

	/**
	 * Update the angle offset
	 * @param new_offset
	 */
	void hall::SetOffset(const float new_offset)
	{
		constexpr float eps = 100.0f * std::numeric_limits<float>::epsilon();

		if(std::abs(new_offset-offset) > eps)
		{
			sc_offset = systems::SinCos(unit::Q31(new_offset));
			offset = new_offset;
		}
	}
}/* namespace observer */



