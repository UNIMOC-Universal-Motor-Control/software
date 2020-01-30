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

} /* namespace filter */

#endif /* INC_FILTER_HPP_ */

