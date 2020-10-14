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
public:
	moving_average<N>(void): buffer{0.0f}, index(0) {};

	/**
	 * @brief insert uk to the buffer and calculate the average of all buffered uk's
	 * @param uk new input value
	 * @return mean of all buffered input values.
	 */
	float Calculate(const float uk)
	{
		float mean = 0.0f;

		buffer[index] = uk;
		index++;

		if(index >= N) index = 0;

		for(std::uint32_t i = 0; i < N; i++)
		{
			mean += buffer[i];
		}

		mean /= N;

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

	///< sampling frequency of the source signal
	const float Fs;

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
	 * @param Fs sampling frequency
	 */
	goertzel<N>(const float Fs): buffer{0.0f}, index(0), Fs(Fs), k(0),
		sc{0.0f, 1.0f}, coeff(2.0f), real(0.0f), imag(0.0f), ac(0.0f), dc(0.0f) {};

	/**
	 * set the frequency of interest.
	 * @note frequency resolution is max Fs/N
	 * @param F frequency to be selected
	 */
	void SetFrequency(const float F)
	{
		k = (uint32_t)(F/Fs*(float)N);
		systems::SinCos(math::_2PI * (float)k/(float)N, sc);
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

	bool Test(void)
	{
		bool result = false;
		const float limit = 1e-1f;
		const float dc = 3.0f;
		const float ac = 0.75f;
		const float omega = 1000.0f*math::_2PI;
		float phi = math::PIby2/3.0f;

		for(uint32_t i = 0; i < N; i++)
		{
			float u = std::sin(phi)*ac + dc;
			Sample(u);
			phi += omega/Fs;
			if(phi > math::_2PI) phi -= math::_2PI;
		}

		SetFrequency(0.0f);
		Calculate();
		this->dc = Magnitude();

		SetFrequency(1000.0f);
		Calculate();
		this->ac = Magnitude();


		return result;
	}
};


} /* namespace filter */

#endif /* INC_FILTER_HPP_ */

