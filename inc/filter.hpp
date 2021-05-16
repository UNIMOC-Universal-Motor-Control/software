/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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
#include <array>
#include <numeric>
#include <climits>
#include <complex>
#include "systems.hpp"


namespace filter
{
/**
 * first order iir low pass
 */
class low_pass
{

private:
	///< last filter output value
	float y;

	///< proportional gain
	float k;

	///< filter time constant dependent coefficient
	float t_tmp;

	///< internal equation to pre calculate filter time constant dependent coefficient
	static inline constexpr float T(const float t, const float ts) { return 1.0/(1.0 + t/ts); }

public:
	/**
	 * stock constuctor with 1/10 time constant and gain of 1
	 */
	low_pass():y(0.0), k(1.0), t_tmp(0.9) {};

	/**
	 * @brief low pass filter constructor with all essential parameters.
	 *
	 * @param ts			set sample time.
	 * @param k				proportional gain.
	 * @param t				time constant.
	 */
	low_pass(const float ts, const float k, const float t):
										y(0.0f), k(k), t_tmp(T(t, ts))
	{};

	/**
	 * @brief set the time constant of the filter
	 *
	 * @param ts sampling time for the Calculate calls
	 * @param t time constant for the filter.
	 */
	constexpr void SetT(const float ts, const float t) { t_tmp = T(t, ts); };

	/**
	 * \brief set the gain of the filter
	 * \param _k new gain of the filter
	 */
	constexpr void SetK(const float _k) { k = _k; };

	/**
	 * \brief calculate new filter output
	 * \param u new input value for the filter
	 * \return new filter output.
	 */
	float Calculate(const float u);

	/**
	 * \brief get the current filter output without calculating a new sample
	 * \return current output value of the filter
	 */
	float Get(void) { return y; };
};


/**
 * first order iir low pass in dq system
 */
class low_pass_dq
{

private:
	low_pass d;
	low_pass q;

public:
	/**
	 * stock constuctor with 1/10 time constant and gain of 1
	 */
	low_pass_dq() {};

	/**
	 * @brief low pass filter constructor with all essential parameters.
	 *
	 * @param ts			set sample time.
	 * @param k				proportional gain.
	 * @param t				time constant.
	 */
	low_pass_dq(const float ts, const float k, const float t):d(ts, k, t), q(ts, k, t) {};

	/**
	 * @brief set the time constant of the filter
	 *
	 * @param ts sampling time for the Calculate calls
	 * @param t time constant for the filter.
	 */
	constexpr void SetT(const float ts, const float t) { d.SetT(ts, t); q.SetT(ts, t); };

	/**
	 * \brief set the gain of the filter
	 * \param k new gain of the filter
	 */
	constexpr void SetK(const float k) { d.SetK(k); q.SetK(k); };

	/**
	 * \brief calculate new filter output
	 * \param u new input value for the filter
	 * \return new filter output.
	 */
	systems::dq Calculate(const systems::dq u) {systems::dq tmp = {d.Calculate(u.d), q.Calculate(u.q)}; return tmp; };

	/**
	 * \brief get the current filter output without calculating a new sample
	 * \return current output value of the filter
	 */
	systems::dq Get(void) { systems::dq tmp = {d.Get(), q.Get()}; return tmp; };
};
/**
 * moving average filter
 */
template <std::uint32_t N>
class moving_average
{

private:
	///< data storage for input values
	std::array<float, N> buffer;

	///< index for next input
	std::uint32_t index;

	///< sum over all buffer values
	float sum;
public:
	moving_average<N>(void): buffer{0.0f}, index(0), sum(0.0f) {};

	/**
	 * @brief insert uk to the buffer and calculate the average of all buffered uk's
	 * @param uk new input value
	 * @return mean of all buffered input values.
	 */
	float Calculate(const float u)
	{
		float mean = 0.0f;

		buffer[index] = u;
		sum += buffer[index];	// add new sample

		index++;
		if(index >= N) index = 0;

		sum -= buffer[index]; // remove oldest sample

		mean = sum/N;

		return mean;
	};
};


/**
 * FIR Filter
 *
 * @param N number of tabs of the fir filter
 */
template <std::uint32_t N>
class fir
{

private:
	///< data storage for input values
	std::array<float, N> buffer;

	///< index for next input
	std::uint32_t index;


	///< filter coefficients
	const std::array<float, N>& coeffs;

public:
	/**
	 * FIR filter constructor
	 * @param new_coeffs
	 */
	fir<N>(const std::array<float, N>& new_coeffs): buffer{0.0f}, index(0), coeffs(new_coeffs) {};

	/**
	 * @brief insert uk to the buffer and calculate the new filtered output
	 * @param uk new input value
	 * @return filtered output
	 */
	float Calculate(const float uk)
	{
		float output = 0.0f;

		buffer[index] = uk;
		index++;

		if(index >= N) index = 0;

		for(std::uint32_t i = 0; i < N; i++)
		{
			output += buffer[(index + i)%N]*coeffs[i];
		}

		return output;
	};
};

/**
 * FIR Filter with down sampling
 *
 * @param N number of tabs of the fir filter
 * @param S downsampling factor
 */
template <std::uint32_t N, std::uint32_t S>
class fir_downsampled : private fir<N>
{

private:
	///< data storage for downsampling
	std::array<float, S> buffer;

	///< index for next input
	std::uint32_t index;

	///< last output value
	float output;
public:
	/**
	 * downsampled FIR filter constructor
	 * @param new_coeffs
	 */
	fir_downsampled<N, S>(const std::array<float, N>& new_coeffs): fir<N>(new_coeffs), buffer{0.0f}, index(0), output(0.0f) {};

	/**
	 * @brief insert uk to the buffer and calculate the new filtered output
	 * @param uk new input value
	 * @return filtered output
	 */
	float Calculate(const float uk)
	{
		buffer[index] = uk;
		index++;

		if(index >= S)
		{
			float mean = 0.0f;

			index = 0;

			for(std::uint32_t i = 0; i < S; i++)
			{
				mean += buffer[i];
			}

			mean /= S;

			output = fir<N>::Calculate(mean);
		}

		return output;
	};
};

/**
 * compute the linear regression y = a + b*x out of an x and y array
 * @param x array
 * @param y array
 * @param length length of both x and y arrays
 * @param gain b
 * @param offset a
 */
void LinearRegression(const float* const x, const float* const y, const std::uint32_t length, float& gain, float& offset);

/**
 * Calculate the complex signal amplitude of a specific frequency
 * @param x array of n signal values
 * @param i index of the most recent sample in the buffer
 * @param n length of signal values array
 * @param k wave index of the frequency of interest k = freq/(sampling_freq)
 * @return complex amplitude of the signal
 */
std::complex<float> Goertzel(const float* const x, const std::uint32_t i, const std::uint32_t n, const std::uint32_t k);

/**
 * Goertzel algorithm to determine amplitude and phase of a specific frequency in the source signal
 *
 * @param N number of samples
 */
template <std::uint32_t N>
class goertzel
{

private:
	///< data storage for input values
	std::array<float, N> buffer;

	///< index for latest sample in buffer.
	std::uint32_t index;

	///< wave number of interest
	std::uint32_t k;

	///< coefficients for real and imaginary part of the algorithm
	systems::sin_cos sc;

	///< coefficient for the algorithm loop
	float coeff;

	///< real part of the output signal
	float real;

	///< imaginary part of the output signal
	float imag;

	float ac, dc;

public:
	/**
	 * Goertzel algorithm container constructor
	 */
	goertzel<N>(void): buffer{0.0f}, index(0), k(0),
		sc{0.0f, 1.0f}, coeff(2.0f), real(0.0f), imag(0.0f), ac(0.0f), dc(0.0f) {};

	/**
	 * set the frequency of interest.
	 * @note frequency resolution is max Fs/N
	 * @param F frequency to be selected
	 * @param Fs sampling frequency of the source signal
	 */
	void SetFrequency(const float F, const float Fs)
	{
		k = (uint32_t)(F/Fs*(float)N);
		sc = systems::SinCos(unit::Q31R(math::_2PI * (float)k/(float)N));
		coeff = 2.0f*sc.cos;
	}

	/**
	 * sample the signal
	 * @param u actual signal value
	 */
	void Sample(const float u)
	{
		buffer[index] = u;
		index++;

		if(index >= N)
		{
			index = 0;
		}
	}

	/**
	 * assignment operator for new samples
	 * @param u new sample
	 * @return input
	 */
	float operator=(const float u)
	{
		Sample(u);
		return u;
	}

	/**
	 * calculate the goertzel algorithm for the selected frequency
	 */
	void Calculate(void)
	{
		std::array<float, 3> s = {0.0f, 0.0f, 0.0f};
		// thread safety
		std::uint32_t idx = index;

		for (std::uint32_t i = 0; i < N; ++i)
		{
			s[0] = buffer[(idx + i)%N] + s[1]*coeff - s[2];
			s[2] = s[1];
			s[1] = s[0];
		}

		real = (s[1] - sc.cos*s[2])/(float)N;
		imag = (sc.sin*s[2])/(float)N;

		// except for k == 0 there are always a positive and a negative frequency amplitude part of the output
		if(k != 0)
		{
			real *= 2.0f;
			imag *= 2.0f;
		}
	};

	float Real(void) { return real;};
	float Imag(void) { return imag;};

	float Magnitude(void) { return std::sqrt(real*real + imag*imag);};
	float Phase(void) { return std::atan2(imag, real); };
};


} /* namespace filter */

#endif /* INC_FILTER_HPP_ */

