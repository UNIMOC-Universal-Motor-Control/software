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
	 * compute the linear regression y = a + b*x out of an x and y array
	 * @param x array
	 * @param y array
	 * @param length length of both x and y arrays
	 * @param gain b
	 * @param offset a
	 */
	void LinearRegression(const float* const x, const float* const y, const std::uint32_t length, float& gain, float& offset)
	{
		float x_mean = 0.0f, y_mean = 0.0f, num = 0.0f, den = 0.0f;

		for(std::uint32_t i = 0; i < length; ++i)
		{
			x_mean += x[i];
			y_mean += y[i];
		}
		x_mean /= (float)length;
		y_mean /= (float)length;

		for (std::uint32_t i = 0; i < length; ++i)
		{
			num += (x[i] - x_mean) * (y[i] - y_mean);
			den += (x[i] - x_mean) * (x[i] - x_mean);
		}

		gain = num/den;
		offset = y_mean - x_mean * gain;
	}

	/**
	 * Calculate the complex signal amplitude of a specific frequency
	 * @param x array of n signal values
	 * @param n length of signal values array
	 * @param k wave index of the frequency of interest k = freq/(sampling_freq)
	 * @return complex amplitude of the signal
	 */
	std::complex<float> Goertzel(const float* const x, const std::uint32_t n, const std::uint32_t k)
	{
		std::array<float, 3> s = {0.0f, 0.0f, 0.0f};
		float w = math::_2PI * k;
		systems::sin_cos sc;

		systems::SinCos(w, sc);
		float coeff = 2.0f*sc.cos;

		for (std::uint32_t i = 0; i < n; ++i)
		{
			s[0] = x[i] + s[1]*coeff - s[2];
			s[2] = s[1];
			s[1] = s[0];
		}

		return (s[0] - sc.cos*s[1] + 1i*sc.sin*s[1]);
	}

}/* namespace filter */



