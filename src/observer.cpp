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
    mechanic::mechanic(const float q, const float r): Q(q), R(r)
    {}

    /**
     * @brief calculate the mechanic model to estimate rotor angle.
     */
    void mechanic::Predict(void)
    {
        // update constants
        constexpr float ts = hardware::Tc;
        const float tsj = ts/settings.mechanics.J;

        // electric torque
        values.motor.m_el = _3by2 * settings.motor.psi * values.motor.rotor.i.q +
                (settings.motor.l.d - settings.motor.l.q) * values.motor.rotor.i.d *
                values.motor.rotor.i.q;

        // omega
        values.motor.rotor.omega += tsj * (values.motor.m_el - values.motor.m_l);

        // integrate omega for phi
        values.motor.rotor.phi += values.motor.rotor.omega * ts;

        // integrate load
        //values.motor.m_l += 0;

        // limit phi to +- 2 * pi
        values.motor.rotor.phi = fmodf(values.motor.rotor.phi, math::_2PI);
    }

    /**
     * @brief correct the mechanic model to estimate rotor angle.
     *
     * @param error  state error feedback
     */
    void mechanic::Correct(const std::array<float, 3> error)
    {
        values.motor.rotor.omega += error[0];
        values.motor.rotor.phi += error[1];
        values.motor.m_l += error[2];
    }

    /**
     * @brief calculate the kalman correction.
     *
     * @param   angle error signal
     * @retval  model error correction
     */
    void mechanic::Update(const float angle_error, std::array<float, 3>& out_error)
    {
        constexpr float ts = hardware::Tc;
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
    flux::flux(void)    {}

    /**
     * @brief Get angular error from flux estimation.
     *
     * @retval angle error
     */
    float flux::Calculate(const systems::sin_cos& sin_cos )
    {
        // BEMF voltage and feedback
        rotor.bemf.d = values.motor.rotor.u.d - settings.motor.rs * values.motor.rotor.i.d + rotor.feedback.d;
        rotor.bemf.q = values.motor.rotor.u.q - settings.motor.rs * values.motor.rotor.i.q + rotor.feedback.q;

        // rotate the bemf to stator system
        stator.bemf = systems::transform::InversePark(rotor.bemf, sin_cos);

        // integrate bemf to flux
        stator.flux.alpha +=  stator.bemf.alpha * hardware::Tc;
        stator.flux.beta  +=  stator.bemf.beta  * hardware::Tc;

        // transform flux to rotor system
        rotor.flux = systems::transform::Park(stator.flux, sin_cos);

        // sub the voltage inducted in the inductors of the stator
        rotor.flux.d -= values.motor.rotor.i.d * settings.motor.l.d;
        rotor.flux.q -= values.motor.rotor.i.q * settings.motor.l.q;

        // compare actual flux with flux parameter
        rotor.feedback.d = settings.observer.C.d * (settings.motor.psi - rotor.flux.d);
        rotor.feedback.q = settings.observer.C.q * (0.0f - rotor.flux.q);

        return rotor.flux.q / settings.motor.psi;
    }

    /**
     * @brief trivial admittance observer constructor
     */
    admittance::admittance(void){}

    /**
     * @brief calculate the mean stator admittance.
     *
     * @note call only once per control cycle
     *
     * @retval mean stator admittance.
     */
    systems::alpha_beta admittance::GetMean(std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& ad)
    {
    	static systems::alpha_beta last = {0.0f, 0.0f};
        systems::alpha_beta y;
        // statonary part of the admittance vector, float sampling
        y.alpha = ((last.alpha - ad[0].alpha) + (ad[0].beta -  ad[1].beta) -  (ad[1].alpha - ad[2].alpha) - (ad[2].beta  - ad[3].beta));
        y.beta  = ((last.beta -  ad[0].beta) -  (ad[0].alpha - ad[1].alpha) - (ad[1].beta -  ad[2].beta)  + (ad[2].alpha - ad[3].alpha));

        last = ad[3];
        return y;
    }

    /**
     * @brief calculate the stator admittance vector.
     *
     * @note call only once per control cycle
     *
     * @retval stator admittance vector.
     */
    systems::alpha_beta admittance::GetVector(std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES>& ad)
    {
    	static systems::alpha_beta last = {0.0f, 0.0f};
        systems::alpha_beta yd;
        // rotating part of the admittance vector, float sampling
        yd.alpha = ((last.beta -  ad[0].beta)  + (ad[0].alpha - ad[1].alpha) - (ad[1].beta -  ad[2].beta)  - (ad[2].alpha - ad[3].alpha));
        yd.beta  = ((last.alpha - ad[0].alpha) - (ad[0].beta -  ad[1].beta)  - (ad[1].alpha - ad[2].alpha) + (ad[2].beta -  ad[3].beta));

        last = ad[3];
        return yd;
    }

}/* namespace observer */



