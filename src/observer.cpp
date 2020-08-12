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
    mechanic::mechanic(void) {}

    /**
     * @brief calculate the mechanic model to estimate rotor angle.
     */
    void mechanic::Predict(void)
    {
        // update constants
        const float ts = hardware::Tc;
        const float tsj = ts/settings.mechanics.J;

        // take only reasonable currents into account
        if(std::fabs(values.motor.rotor.i.q) > settings.observer.mech.i_min)
        {
        	// electric torque
        	values.motor.m_el = _3by2 * settings.motor.psi * values.motor.rotor.i.q +
        			(settings.motor.l.d - settings.motor.l.q) * values.motor.rotor.i.d *
					values.motor.rotor.i.q;
        }
        else
        {
        	values.motor.m_el = 0.0f;
        }

        // omega
        values.motor.rotor.omega += tsj * (values.motor.m_el - values.motor.m_l);

        // integrate omega for phi
        values.motor.rotor.phi += values.motor.rotor.omega * ts;

        // integrate load
        //values.motor.m_l += 0;

        // limit phi to 2 * pi and count rotations
        if(values.motor.rotor.phi > math::_2PI)
        {
        	values.motor.rotor.phi -= math::_2PI;
        	values.motor.rotor.rotation++;
        }
        else if(values.motor.rotor.phi < 0.0)
        {
        	values.motor.rotor.phi += math::_2PI;
        	values.motor.rotor.rotation--;
        }
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
	 * @param q model uncertainty
	 * @param r measurement uncertainty
	 * @param   angle error signal
	 * @retval  model error correction
	 */
	void mechanic::Update(const float Q, const float R, const float angle_error, std::array<float, 3>& out_error)
    {
        const float ts = hardware::Tc;
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
     * @param sin_cos sine and cosine of the actual rotor angle
     * @retval kalman correction vector
     */
    void flux::Calculate(const systems::sin_cos& sin_cos, std::array<float, 3>& correction)
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
    	rotor.feedback.d = settings.observer.flux.C.d * (settings.motor.psi - rotor.flux.d);
    	rotor.feedback.q = settings.observer.flux.C.q * (0.0f - rotor.flux.q);

    	float error = rotor.flux.q / settings.motor.psi;

    	// Kalman filter on angular error
    	mech.Update(settings.observer.flux.Q, settings.observer.flux.R, error, correction);
    }

    /**
     * @brief hfi observers trivial constructor
     */
    hfi::hfi(void):d(hardware::Tc, 1.0f, math::_2PI * 5.0f/settings.observer.hfi.frequency),
    		q(hardware::Tc, 1.0f, math::_2PI * 5.0f/settings.observer.hfi.frequency) {}

    /**
     * @brief Get angular error from hfi estimation.
     * @param i_ab stationary current vector
     * @retval kalman correction vector
     */
    void hfi::Calculate(const systems::dq& i_dq, std::array<float, 3>& correction)
    {


//    	mech.Update(settings.observer.flux.Q, settings.observer.flux.R, 0.5f * values.motor.rotor.i_hfi.q/length, correction);
    }

	/**
	 * @brief add injection voltage
	 * @return d-Voltage Injection
	 */
	float hfi::Injection(void)
	{
		w = settings.observer.hfi.frequency;
		phi += w * hardware::Tc;

		// calculate voltage amplitude from inductive resistance at injection frequency
		ui = settings.motor.l.d * w * settings.observer.hfi.current;

		// limit phii to 2 * pi
		if(phi > math::_2PI)
		{
			phi -= math::_2PI;
		}
		else if(phi < 0.0)
		{
			values.motor.rotor.phi += math::_2PI;
		}

		// calculate sine and cosine
		systems::SinCos(phi, sc);

		// add injection signal
		return ui*sc.sin;
	}


}/* namespace observer */



