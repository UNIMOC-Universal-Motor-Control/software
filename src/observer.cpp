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
     * @param i rotor currents
     */
    void mechanic::Predict(const systems::dq& i)
    {
        // update constants
        const float ts = hardware::Tc;
        const float tsj = ts/settings.mechanics.J;

        // electric torque
        values.motor.m_el = _3by2 * (settings.motor.psi * i.q +
        		(settings.motor.l.d - settings.motor.l.q) * i.d * i.q);

        // omega
        values.motor.rotor.omega += tsj * (values.motor.m_el - values.motor.m_l);

        if(std::fabs(values.motor.rotor.omega) > 2.0f * settings.motor.limits.omega)
        	values.motor.rotor.omega = std::copysign(2.0f * settings.motor.limits.omega, values.motor.rotor.omega);

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

	///< high pass FIR filter coefficients
	const std::array<float, 64> hfi::hpf_c =
	{
		-0.117076066148919605   ,
		 0.447877317232674654   ,
		-0.549588574203091040   ,
		 0.051528729858987876   ,
		 0.276482388376549926   ,
		 0.057833468267956704   ,
		-0.154094057438108667   ,
		-0.134405173098699121   ,
		 0.020195571850961717   ,
		 0.117912229130412410   ,
		 0.080358734691764050   ,
		-0.025014173633087403   ,
		-0.087221541380083228   ,
		-0.059746644660306242   ,
		 0.015539095310111797   ,
		 0.064450033761963260   ,
		 0.050055520387486313   ,
		-0.004211651282322758   ,
		-0.046073692369834593   ,
		-0.043066735533787717   ,
		-0.005354729329647237   ,
		 0.030478693777419484   ,
		 0.035879686611226827   ,
		 0.011805666737311715   ,
		-0.017458417215608611   ,
		-0.027916435761154757   ,
		-0.014788839694913722   ,
		 0.007350785543581751   ,
		 0.019693188504408207   ,
		 0.014668054811539686   ,
		-434.3534301953084760E-6,
		-0.012116549113152825   ,
		-0.012314387459981979   ,
		-0.003426001271693080   ,
		 0.006051671000406204   ,
		 0.008894462881814457   ,
		 0.004587700323629035   ,
		-0.001851624220910895   ,
		-0.005271347263479466   ,
		-0.004137190157322559   ,
		-525.3156244348163000E-6,
		 0.002405666165839446   ,
		 0.002901784608475874   ,
		 0.001366162221522491   ,
		-581.8309458139999610E-6,
		-0.001561653062895847   ,
		-0.001258498114288735   ,
		-278.2070651675929300E-6,
		 548.9714769549730140E-6,
		 771.4115283754740630E-6,
		 466.1402123169401650E-6,
		 729.8442446889778240E-9,
		-298.4617683893739010E-6,
		-326.1514244276998510E-6,
		-172.3044081415219180E-6,
		 10.54019981969690890E-6,
		 120.5687530818757120E-6,
		 140.0293744366702530E-6,
		 104.6941972850297020E-6,
		 57.91355996016838500E-6,
		 24.12890008659049100E-6,
		 7.292732347930074080E-6,
		 1.436325867676127470E-6,
		 137.9190309105846950E-9
	};
    /**
     * @brief hfi observers trivial constructor
     */
    hfi::hfi(void): sc{0.0f, 0.0f}, w(0.0f), phi(0.0f), ui(0.0f), mech(), hpf_d(hpf_c), hpf_q(hpf_c) {}

    /**
     * @brief Get angular error from hfi estimation.
     * @param i_ab stationary current vector
     * @retval kalman correction vector
     */
    void hfi::Calculate(systems::dq& i_dq, std::array<float, 3>& correction)
    {
    	values.motor.rotor.i_hfi.d = hpf_d.Calculate(i_dq.d);
    	values.motor.rotor.i_hfi.q = hpf_q.Calculate(i_dq.q);

    	// remove hf part rom current response
    	i_dq.d -= values.motor.rotor.i_hfi.d;
    	i_dq.q -= values.motor.rotor.i_hfi.q;

    	values.motor.rotor.i_hfi.q *= std::fabs((w*settings.motor.l.d*settings.motor.l.q)/((settings.motor.l.q - settings.motor.l.d)*0.5f*ui));

    	// make response correctly signed.
    	values.motor.rotor.i_hfi.q = lpf.Calculate(values.motor.rotor.i_hfi.q * sc.sin);

    	mech.Update(settings.observer.hfi.Q, settings.observer.hfi.R, values.motor.rotor.i_hfi.q, correction);
    }

	/**
	 * @brief add injection voltage
	 * @return d-Voltage Injection
	 */
	float hfi::Injection(void)
	{
		w = settings.observer.hfi.frequency;
		phi += w * hardware::Tc;

		// limit phii to 2 * pi
		if(phi > math::_2PI)
		{
			phi -= math::_2PI;
		}
		else if(phi < 0.0)
		{
			phi += math::_2PI;
		}

		// calculate sine and cosine
		systems::SinCos(phi, sc);

		// calculate voltage amplitude from inductive resistance at injection frequency
		ui = settings.motor.l.d * w * settings.observer.hfi.current;

		// add injection signal
		return ui*sc.sin;
	}


}/* namespace observer */



