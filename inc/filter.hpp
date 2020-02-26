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
enum biquad_type_ec
{
    lowpass = 0,//!< lowpass
    highpass,   //!< highpass
    bandpass,   //!< bandpass
    notch,      //!< notch
    peak,       //!< peak
    lowshelf,   //!< lowshelf
    highshelf   //!< highshelf
};

/**
 * Class for second order IIR filter in direct transposed form II
 */
class biquad
{
public:
	/**
	 * Constructor of biquad second order IIR filter
	 * @param type	type of filter
	 * @param Fc	corner frequency normed to 1/(sampling frequency)
	 * @param Q		quality factor of the filter response
	 * @param peak_gain gain of the filter in dB
	 */
    biquad(const biquad_type_ec type, const float Fc, const float Q, const float peak_gain);

    /**
     * set type of biquad filter
     * @param type new type of filter
     */
    void SetType(const biquad_type_ec type);

    /**
     * set the quality factor of the filter
     * @param Q new quality factor
     */
    void SetQ(const float Q);

    /**
     * set new corner frequency
     * @param Fc new corner frequency
     */
    void SetFc(const float Fc);

    /**
     * set new gain of the filter
     * @param peak_gain in db
     */
    void SetPeakGain(const float peak_gain);

    /**
     * set all relevant biquad parameters at once
     * @param type new type
     * @param Fc new corner freq.
     * @param Q new quality factor
     * @param peak_gain new gain
     */
    void SetBiquad(const biquad_type_ec type, const float Fc, const float Q, const float peak_gain);

    /**
     * calculate the filter
     * @param in new input to the filter
     * @return output of the filter
     */
    float Process(const float in);

protected:
    /**
     * calculate internal coefficients
     */
    void CalcBiquad(void);

    ///< type of filter
    biquad_type_ec type;
    ///< internal coefficients
    float a0, a1, a2, b1, b2;
    ///< settings of the filter
    float Fc, Q, peak_gain;
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
    float out = in * a0 + z1;
    z1 = in * a1 + z2 - b1 * out;
    z2 = in * a2 - b2 * out;
    return out;
}

} /* namespace filter */

#endif /* INC_FILTER_HPP_ */

