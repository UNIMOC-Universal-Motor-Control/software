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
#include <filter.hpp>
#include <cstdint>
#include <systems.hpp>

/**
 * @namespace filter classes
 */
namespace filter
{
	/**
	 * @brief low pass filter constructor with all essential parameters.
	 *
	 * @param new_ts			set sample time.
	 * @param new_k				proportional gain.
	 * @param new_t				time constant.
	 */
	low_pass::low_pass(const float new_ts, const float new_k, const float new_t):
										ts(new_ts), k(new_k), t_tmp(T(new_t, ts))
	{
	}


	/**
	 * @brief calculate filter equation with out memory of old samples.
	 *
	 * @param uk			filter input.
	 * @param yk_1			filter output from last cycle.
	 * @retval yk			filter output.
	 */
	float low_pass::Calculate(const float uk, const float yk_1)
	{
		float yk = t_tmp*(k*uk - yk_1) + yk_1;
		return yk;
	}

	/**
	 * Created by Nigel Redmon on 11/24/12
	 * EarLevel Engineering: earlevel.com
	 * Copyright 2012 Nigel Redmon
	 *
	 * For a complete explanation of the Biquad code:
	 * http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/
	 *
	 * License:
	 *
	 * This source code is provided as is, without warranty.
	 * You may copy and distribute verbatim copies of this document.
	 * You may modify and use this source code to create binary code
	 * for your own purposes, free or commercial.
	 *
	 */

	/**
	 * Constructor of biquad second order IIR filter
	 * @param type	type of filter
	 * @param Fc	corner frequency normed to 1/(sampling frequency)
	 * @param Q		quality factor of the filter response
	 * @param peak_gain gain of the filter.
	 */
	biquad::biquad(const biquad_type_ec type, const float Fc, const float Q, const float peak_gain)
	{
		SetBiquad(type, Fc, Q, peak_gain);
		z1 = z2 = 0.0f;
	}

    /**
     * set type of biquad filter
     * @param type new type of filter
     */
	void biquad::SetType(const biquad_type_ec type)
	{
		this->type = type;
		CalcBiquad();
	}

    /**
     * set the quality factor of the filter
     * @param Q new quality factor
     */
    void biquad::SetQ(const float Q)
	{
		this->Q = Q;
		CalcBiquad();
	}

    /**
     * set new corner frequency
     * @param Fc new corner frequency
     */
    void biquad::SetFc(const float Fc)
    {
		this->Fc = Fc;
		CalcBiquad();
	}

    /**
     * set new gain of the filter
     * @param peak_gain in db
     */
    void biquad::SetPeakGain(const float peak_gain)
    {
		this->peak_gain = peak_gain;
		CalcBiquad();
	}

    /**
     * set all relevant biquad parameters at once
     * @param type new type
     * @param Fc new corner freq.
     * @param Q new quality factor
     * @param peak_gain new gain
     */
    void biquad::SetBiquad(const biquad_type_ec type, const float Fc, const float Q, const float peak_gain)
    {
		this->type = type;
		this->Q = Q;
		this->Fc = Fc;
		SetPeakGain(peak_gain);
	}

    /**
     * calculate internal coefficients
     */
    void biquad::CalcBiquad(void)
    {
		float norm;
		float V = std::pow(10.0f, fabsf(peak_gain) / 20.0f);
		float K = std::tan(M_PI * Fc);

		switch (this->type)
		{
		case biquad_type_ec::lowpass:

			norm = 1.0f / (1.0f + K / Q + K * K);
			a0 = K * K * norm;
			a1 = 2.0f * a0;
			a2 = a0;
			b1 = 2.0f * (K * K - 1.0f) * norm;
			b2 = (1.0f - K / Q + K * K) * norm;
			break;

		case biquad_type_ec::highpass:

			norm = 1.0f / (1.0f + K / Q + K * K);
			a0 = 1.0f * norm;
			a1 = -2.0f * a0;
			a2 = a0;
			b1 = 2.0f * (K * K - 1.0f) * norm;
			b2 = (1.0f - K / Q + K * K) * norm;
			break;

		case biquad_type_ec::bandpass:

			norm = 1.0f / (1.0f + K / Q + K * K);
			a0 = K / Q * norm;
			a1 = 0.0f;
			a2 = -a0;
			b1 = 2.0f * (K * K - 1.0f) * norm;
			b2 = (1.0f - K / Q + K * K) * norm;
			break;

		case biquad_type_ec::notch:

			norm = 1.0f / (1.0f + K / Q + K * K);
			a0 = (1.0f + K * K) * norm;
			a1 = 2.0f * (K * K - 1) * norm;
			a2 = a0;
			b1 = a1;
			b2 = (1.0f - K / Q + K * K) * norm;
			break;

		case biquad_type_ec::peak:

			if (peak_gain >= 0.0f)
			{    // boost
				norm = 1.0f / (1.0f + 1.0f/Q * K + K * K);
				a0 = (1.0f + V/Q * K + K * K) * norm;
				a1 = 2.0f * (K * K - 1.0f) * norm;
				a2 = (1.0f - V/Q * K + K * K) * norm;
				b1 = a1;
				b2 = (1.0f - 1.0f/Q * K + K * K) * norm;
			}
			else
			{    // cut
				norm = 1 / (1 + V/Q * K + K * K);
				a0 = (1 + 1/Q * K + K * K) * norm;
				a1 = 2 * (K * K - 1) * norm;
				a2 = (1 - 1/Q * K + K * K) * norm;
				b1 = a1;
				b2 = (1 - V/Q * K + K * K) * norm;
			}
			break;
		case biquad_type_ec::lowshelf:
			if (peak_gain >= 0)
			{    // boost
				norm = 1.0f / (1 + math::SQRT2 * K + K * K);
				a1 = 2.0f * (V * K * K - 1.0f) * norm;
				a2 = (1.0f - std::sqrt(2.0f*V) * K + V * K * K) * norm;
				b1 = 2.0f * (K * K - 1.0f) * norm;
				b2 = (1.0f - math::SQRT2 * K + K * K) * norm;
			}
			else
			{    // cut
				norm = 1.0f / (1.0f + std::sqrt(2.0f*V) * K + V * K * K);
				a0 = (1.0f + math::SQRT2 * K + K * K) * norm;
				a1 = 2.0f * (K * K - 1.0f) * norm;
				a2 = (1.0f - math::SQRT2 * K + K * K) * norm;
				b1 = 2.0f * (V * K * K - 1.0f) * norm;
				b2 = (1.0f - std::sqrt(2.0f*V) * K + V * K * K) * norm;
			}
			break;
		case biquad_type_ec::highshelf:
			if (peak_gain >= 0)
			{    // boost
				norm = 1.0f / (1.0f + math::SQRT2 * K + K * K);
				a0 = (V + std::sqrt(2.0f*V) * K + K * K) * norm;
				a1 = 2.0f * (K * K - V) * norm;
				a2 = (V - std::sqrt(2.0f*V) * K + K * K) * norm;
				b1 = 2.0f * (K * K - 1.0f) * norm;
				b2 = (1.0f - math::SQRT2 * K + K * K) * norm;
			}
			else
			{    // cut
				norm = 1.0f / (V + std::sqrt(2.0f*V) * K + K * K);
				a0 = (1.0f + math::SQRT2 * K + K * K) * norm;
				a1 = 2.0f * (K * K - 1.0f) * norm;
				a2 = (1.0f - math::SQRT2 * K + K * K) * norm;
				b1 = 2.0f * (K * K - V) * norm;
				b2 = (V - std::sqrt(2.0f*V) * K + K * K) * norm;
			}
			break;
		}
	}


}/* namespace filter */



