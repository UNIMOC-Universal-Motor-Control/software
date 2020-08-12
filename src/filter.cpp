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


}/* namespace filter */



