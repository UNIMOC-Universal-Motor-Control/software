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
#include <filter.hpp>
#include <cstdint>
#include <systems.hpp>

/**
 * @namespace filter classes
 */
namespace filter
{

	/**
	 * @brief calculate filter equation.
	 *
	 * @param uk			filter input.
	 * @retval y			filter output.
	 */
	float low_pass::Calculate(const float u)
	{
		y = t_tmp*(k*u - y) + y;
		return y;
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

}/* namespace filter */



