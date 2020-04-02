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
#ifndef INC_FILTER_HPP_
#define INC_FILTER_HPP_

#include <cstdint>
#include <cmath>
#include <climits>
#include "systems.hpp"


namespace filter
{
/**
 * first order iir low pass
 */
class low_pass
{

private:
	///< filter sample time
	const float ts;

	///< proportional gain
	const float k;

	///< filter time constant dependent coefficient
	const float t_tmp;

	///< internal equation to pre calculate filter time constant dependent coefficient
	static inline float T( float t, float ts) { return 1.0/(1.0 + t/ts); }

public:
	low_pass(const float new_ts, const float new_k, const float new_t);

	float Calculate(const float uk, const float yk_1);
};


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

///< Filter types for the biquad class
typedef enum biquad_type_e
{
    lowpass = 0,//!< lowpass
    highpass,   //!< highpass
    bandpass,   //!< bandpass
    notch,      //!< notch
    peak,       //!< peak
    lowshelf,   //!< lowshelf
    highshelf   //!< highshelf
} biquad_type_et;

///< internal coefficient structure
typedef struct biquad_coefficients_s
{
	float a0, a1, a2, b1, b2;
} biquad_coefficients_ts;

/**
 * @brief calculate biquad coefficients
 *
 * This function shall only be used at compile time
 * otherwise it pulls a lot of bloat
 *
 * @param type	type of filter
 * @param fc	corner frequency normed to 1/(sampling frequency)
 * @param q		quality factor of the filter response
 * @param peak_gain gain of the filter.
 */
constexpr biquad_coefficients_ts BiquadCalc(const biquad_type_et type, const float fc, const float q, const float peak_gain)
{
	biquad_coefficients_ts __coeff = {.a0 = 0.0f, .a1 = 0.0f, .a2 = 0.0f, .b1 = 0.0f, .b2 = 0.0f};
	float norm = 0.0f;
	float V = std::pow(10.0f, std::fabs(peak_gain) / 20.0f);
	float K = std::tan(math::PI * fc);

	switch (type)
	{
	case biquad_type_et::lowpass:

		norm = 1.0f / (1.0f + K / q + K * K);
		__coeff.a0 = K * K * norm;
		__coeff.a1 = 2.0f * __coeff.a0;
		__coeff.a2 = __coeff.a0;
		__coeff.b1 = 2.0f * (K * K - 1.0f) * norm;
		__coeff.b2 = (1.0f - K / q + K * K) * norm;
		break;

	case biquad_type_et::highpass:

		norm = 1.0f / (1.0f + K / q + K * K);
		__coeff.a0 = 1.0f * norm;
		__coeff.a1 = -2.0f * __coeff.a0;
		__coeff.a2 = __coeff.a0;
		__coeff.b1 = 2.0f * (K * K - 1.0f) * norm;
		__coeff.b2 = (1.0f - K / q + K * K) * norm;
		break;

	case biquad_type_et::bandpass:

		norm = 1.0f / (1.0f + K / q + K * K);
		__coeff.a0 = K / q * norm;
		__coeff.a1 = 0.0f;
		__coeff.a2 = -__coeff.a0;
		__coeff.b1 = 2.0f * (K * K - 1.0f) * norm;
		__coeff.b2 = (1.0f - K / q + K * K) * norm;
		break;

	case biquad_type_et::notch:

		norm = 1.0f / (1.0f + K / q + K * K);
		__coeff.a0 = (1.0f + K * K) * norm;
		__coeff.a1 = 2.0f * (K * K - 1) * norm;
		__coeff.a2 = __coeff.a0;
		__coeff.b1 = __coeff.a1;
		__coeff.b2 = (1.0f - K / q + K * K) * norm;
		break;

	case biquad_type_et::peak:

		if (peak_gain >= 0.0f)
		{    // boost
			norm = 1.0f / (1.0f + 1.0f/q * K + K * K);
			__coeff.a0 = (1.0f + V/q * K + K * K) * norm;
			__coeff.a1 = 2.0f * (K * K - 1.0f) * norm;
			__coeff.a2 = (1.0f - V/q * K + K * K) * norm;
			__coeff.b1 = __coeff.a1;
			__coeff.b2 = (1.0f - 1.0f/q * K + K * K) * norm;
		}
		else
		{    // cut
			norm = 1 / (1 + V/q * K + K * K);
			__coeff.a0 = (1 + 1/q * K + K * K) * norm;
			__coeff.a1 = 2 * (K * K - 1) * norm;
			__coeff.a2 = (1 - 1/q * K + K * K) * norm;
			__coeff.b1 = __coeff.a1;
			__coeff.b2 = (1 - V/q * K + K * K) * norm;
		}
		break;
	case biquad_type_et::lowshelf:
		if (peak_gain >= 0)
		{    // boost
			norm = 1.0f / (1 + math::SQRT2 * K + K * K);
			__coeff.a1 = 2.0f * (V * K * K - 1.0f) * norm;
			__coeff.a2 = (1.0f - std::sqrt(2.0f*V) * K + V * K * K) * norm;
			__coeff.b1 = 2.0f * (K * K - 1.0f) * norm;
			__coeff.b2 = (1.0f - math::SQRT2 * K + K * K) * norm;
		}
		else
		{    // cut
			norm = 1.0f / (1.0f + std::sqrt(2.0f*V) * K + V * K * K);
			__coeff.a0 = (1.0f + math::SQRT2 * K + K * K) * norm;
			__coeff.a1 = 2.0f * (K * K - 1.0f) * norm;
			__coeff.a2 = (1.0f - math::SQRT2 * K + K * K) * norm;
			__coeff.b1 = 2.0f * (V * K * K - 1.0f) * norm;
			__coeff.b2 = (1.0f - std::sqrt(2.0f*V) * K + V * K * K) * norm;
		}
		break;
	case biquad_type_et::highshelf:
		if (peak_gain >= 0)
		{    // boost
			norm = 1.0f / (1.0f + math::SQRT2 * K + K * K);
			__coeff.a0 = (V + std::sqrt(2.0f*V) * K + K * K) * norm;
			__coeff.a1 = 2.0f * (K * K - V) * norm;
			__coeff.a2 = (V - std::sqrt(2.0f*V) * K + K * K) * norm;
			__coeff.b1 = 2.0f * (K * K - 1.0f) * norm;
			__coeff.b2 = (1.0f - math::SQRT2 * K + K * K) * norm;
		}
		else
		{    // cut
			norm = 1.0f / (V + std::sqrt(2.0f*V) * K + K * K);
			__coeff.a0 = (1.0f + math::SQRT2 * K + K * K) * norm;
			__coeff.a1 = 2.0f * (K * K - 1.0f) * norm;
			__coeff.a2 = (1.0f - math::SQRT2 * K + K * K) * norm;
			__coeff.b1 = 2.0f * (K * K - V) * norm;
			__coeff.b2 = (V - std::sqrt(2.0f*V) * K + K * K) * norm;
		}
		break;
	}
	return __coeff;
}

/**
 * Class for second order IIR filter in direct transposed form II
 */
class biquad
{
public:


	/**
	 * Constructor of biquad second order IIR filterj
	 * @param coeff  Coefficients pre calculated
	 */
    biquad(const biquad_coefficients_ts coeff);

    /**
     * calculate the filter
     * @param in new input to the filter
     * @return output of the filter
     */
    float Process(const float in);

protected:
    ///< internal coefficients
    biquad_coefficients_ts coeff;

    ///< internal unit delay states
    float z1, z2;
};

/**
 * calculate the filter
 * @param in new input to the filter
 * @return output of the filter
 */
inline float biquad::Process(const float in)
{
    float out = in * coeff.a0 + z1;
    z1 = in * coeff.a1 + z2 - coeff.b1 * out;
    z2 = in * coeff.a2 - coeff.b2 * out;
    return out;
}

} /* namespace filter */

#endif /* INC_FILTER_HPP_ */

