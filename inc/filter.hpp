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

} /* namespace filter */

#endif /* INC_FILTER_HPP_ */

