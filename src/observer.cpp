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
    flux::flux(void):rotor(), stator()
    {}

    /**
     * @brief Get angular error from flux estimation.
     * @param set_flux expected flux vector
     * @retval out_flux estimated flux without inducted flux
     */
    void flux::Calculate(systems::dq& set_flux, systems::dq& out_flux)
    {
    	using namespace values::motor::rotor;
    	using namespace values::motor;

    	// BEMF voltage and feedback
    	rotor.bemf.d = u.d - settings.motor.rs * i.d + rotor.feedback.d;
    	rotor.bemf.q = u.q - settings.motor.rs * i.q + rotor.feedback.q;

    	// rotate the bemf to stator system
    	stator.bemf = systems::transform::InversePark(rotor.bemf, values::motor::rotor::sc);

    	// integrate bemf to flux
    	stator.flux.alpha +=  stator.bemf.alpha * hardware::Tc();
    	stator.flux.beta  +=  stator.bemf.beta  * hardware::Tc();

    	// transform flux to rotor system
    	out_flux = systems::transform::Park(stator.flux, values::motor::rotor::sc);

    	// sub the voltage inducted in the inductors of the stator
    	out_flux.d -= i.d * settings.motor.l.d;
    	out_flux.q -= i.q * settings.motor.l.q;

    	// compare actual flux with flux parameter
    	rotor.feedback.d = settings.observer.flux.C.d * (set_flux.d - out_flux.d);
    	rotor.feedback.q = settings.observer.flux.C.q * (set_flux.q - out_flux.q);
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
    hall::hall(void):offset(0.0f), sc_offset{0.0f, 1.0f}   {}

    /**
     * @brief Get sine and cosine values from hall for estimation in rotor frame.
     * @retval est hall sensor signal in rotor frame
     */
    void hall::Calculate(systems::dq& est)
    {
    	systems::alpha_beta tab;
    	systems::abc hall;

    	if(values::motor::hall::state & settings.observer.hall.map.a) hall.a = 1.0f;
    	else hall.a = -1.0f;

    	if(values::motor::hall::state & settings.observer.hall.map.b) hall.b = 1.0f;
    	else hall.b = -1.0f;

    	if(values::motor::hall::state & settings.observer.hall.map.c) hall.c = 1.0f;
    	else hall.c = -1.0f;

    	tab = systems::transform::Clark(hall);

    	// bring hall angle vector to rotor frame
    	est = systems::transform::Park(tab, values::motor::rotor::sc);
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



