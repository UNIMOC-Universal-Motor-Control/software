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
	 * constructor with all internal references
	 * @param drive	torque input
	 * @param load torque output
	 * @param omega angular velocity
	 * @param phi angular position Q31
	 * @param J inertia
	 * @param Q kalman Q gain
	 * @param R kalman R gain
	 * @param omega_limit_positive positive limit for omega
	 * @param omega_limit_negative negative limit for omega
	 */
	mechanic::mechanic(float& drive,
			float& load,
			float& omega,
			std::int32_t& phi,
			float& J,
			float& Q,
			float& R,
			const float P,
			float& omega_limit_positive,
			float& omega_limit_negative)
			: p{0.0f}, pk{0.0f}, k{0.0f}, s(0.0f), sigma(0.0f), out_error{0.0f},
			  torque_drive(drive), torque_load(load), omega(omega), phi(phi), J(J), Q(Q), R(R),
			  omega_limit_positive(omega_limit_positive), omega_limit_negative(omega_limit_negative)
			{
				// set initial covariance
				p[0][0] = P;
				p[1][1] = P;
				p[2][2] = P;
			}

    /**
     * @brief calculate the mechanic model to estimate rotor angle.
     * @param i rotor currents
     */
    void mechanic::Predict()
    {
    	// update constants
        const float ts = hardware::Tc();
        const float tsj = ts/J;

        // omega
        omega += tsj * (torque_drive - torque_load);

        // apply limits not directly to have margin for control
        if(omega > 2.0f * omega_limit_positive)
        {
        	omega = 2.0f * omega_limit_positive;
        }
        else if(omega < 2.0f * omega_limit_negative)
        {
        	omega = 2.0f * omega_limit_negative;
        }

        if(!std::isfinite(omega))
        {
        	omega = 0.0f;
        }

        // integrate omega for phi
        phi += unit::Q31R(omega * ts);
    }

	/**
	 * @brief calculate the kalman correction.
	 * @param q model uncertainty
	 * @param r measurement uncertainty
	 * @param   angle error signal
	 * @retval  model error correction
	 */
	void mechanic::Update(const float angle_error)
    {
        const float ts = hardware::Tc();
        const float tsj = ts/J;

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

        omega 		+= k[0] * angle_error;
        phi 		+= unit::Q31R(k[1] * angle_error);
        torque_load += k[2] * angle_error;
    }

    /**
     * @brief flux observers trivial constructor
     */
    flux::flux(void):rotor(), stator()
    {}

    /**
     * @brief Get angular error from flux estimation.
     * @param set_flux expected flux vector
     * @param C feedback gains
     * @retval out_flux estimated flux without inducted flux
     */
    void flux::Calculate(systems::dq& set_flux, systems::dq& out_flux, const systems::dq C)
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
    	rotor.feedback.d = C.d * (set_flux.d - out_flux.d);
    	rotor.feedback.q = C.q * (set_flux.q - out_flux.q);
    }

       /**
     * @brief hall observers trivial constructor
     */
    hall::hall(void):offset(0.0f), sc_offset{0.0f, 1.0f} {}

    /**
     * @brief Get sine and cosine values from hall for estimation in rotor frame.
     * @retval est hall sensor signal in rotor frame
     */
    void hall::Calculate(systems::dq& est)
    {
    	systems::alpha_beta tab;
    	systems::dq tdq;
    	systems::abc hall;


    	if(values::motor::hall::state & settings.observer.hall.map.a) hall.a = 1.0f;
    	else hall.a = -1.0f;

    	if(values::motor::hall::state & settings.observer.hall.map.b) hall.b = 1.0f;
    	else hall.b = -1.0f;

    	if(values::motor::hall::state & settings.observer.hall.map.c) hall.c = 1.0f;
    	else hall.c = -1.0f;

    	tab = systems::transform::Clark(hall);

    	// Turn the vector by the hall offset
    	tdq = systems::transform::Park(tab, sc_offset);

    	// scale the vector with the rotor flux
    	tab.alpha = tdq.d * settings.motor.psi;
    	tab.beta = tdq.q * settings.motor.psi;

    	// bring hall angle vector to rotor frame
    	est = systems::transform::Park(tab, values::motor::rotor::sc);
    }

	/**
	 * Update the angle offset
	 * @param new_offset
	 */
	void hall::SetOffset(const std::int32_t new_offset)
	{
		if(offset != new_offset)
		{
			offset = new_offset;
			sc_offset = hardware::SinCos(offset);
		}
	}
}/* namespace observer */



