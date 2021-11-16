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
#include "systems.hpp"



/**
 * @namespace coordinate systems
 */
namespace systems
{
    /**
     * @brief calculate length of dq vector.
     *
     * @param vector    vector in dq-system.
     * @retval length of the vector
     */
    float Length(const dq& vector)
    {
        return std::sqrt(vector.d*vector.d + vector.q*vector.q);
    }

    /**
     * @brief calculate length of alpha beta vector.
     *
     * @param vector    vector in alpha-beta-system.
     * @retval length of the vector
     */
    float Length(const alpha_beta& vector)
    {
        return std::sqrt(vector.alpha*vector.alpha + vector.beta*vector.beta);
    }

    /**
      @brief         Calculate angle difference from sin/cosine values.
      @param 	     a    	sin/cosine of the angle a in c = a - b
      @param 	     b    	sin/cosine of the angle b in c = a - b
      @retval	     c      angle difference (sin(c)) but ok for small values
     */
    float SinCosDiff(const sin_cos& a, const sin_cos& b)
    {
    	return a.sin * b.cos - a.cos * b.sin;
    }

    /**
     * @namespace coordinate system transformations
     */
    namespace transform
    {
        /**
         * @brief transform alpha beta vector to dq vector.
         *
         * @note https://de.wikipedia.org/wiki/D/q-Transformation
         *
         * @param vector    vector in alpha-beta-system.
         * @param angle     sine and cosine of the rotor angle.
         * @retval vector in dq-system
         */
        dq Park(const alpha_beta &vector, const sin_cos& angle)
        {
            dq result;
            result.d = angle.cos * vector.alpha + angle.sin * vector.beta;
            result.q = -angle.sin * vector.alpha + angle.cos * vector.beta;
            return result;
        }

        /**
         * @brief transform abc 3 phase vector to alpha beta vector.
         *
         * @note https://de.wikipedia.org/wiki/Clarke-Transformation
         *
         * @param vector    vector in abc-system.
         * @retval vector in alpha-beta-system
         */
        alpha_beta Clark(const abc& vector)
        {
            constexpr float sqrt3by2 = std::sqrt(3.0f)/2.0f;
            constexpr float _2by3 = 2.0f / 3.0f;

            alpha_beta result;

            result.alpha = _2by3 * (vector.a - (0.5f * vector.b) - (0.5f * vector.c));
            result.beta = _2by3 * ((sqrt3by2 * vector.b) - (sqrt3by2 * vector.c));

            return result;
        }

        /**
         * @brief transform dq vector to alpha beta vector.
         *
         * @note https://de.wikipedia.org/wiki/D/q-Transformation
         *
         * @param vector    vector in dq-system.
         * @param angle     sine and cosine of the rotor angle.
         * @retval vector in alpha-beta-system
         */
        alpha_beta InversePark(const dq &vector, const sin_cos& angle)
        {
            alpha_beta result;

            result.alpha = angle.cos * vector.d - angle.sin * vector.q;
            result.beta = angle.sin * vector.d + angle.cos * vector.q;

            return result;
        }

        /**
         * @brief transform alpha beta vector to abc 3 phase vector.
         *
         * @note https://de.wikipedia.org/wiki/Clarke-Transformation
         *
         * @param vector    vector in alpha beta system.
         * @retval vector in abc-system
         */
        abc InverseClark(const alpha_beta& vector)
        {
            constexpr float sqrt3by2 = std::sqrt(3.0f)/2.0f;
            abc result;

            result.a = vector.alpha;
            result.b = (-0.5f * vector.alpha) + (sqrt3by2 * vector.beta);
            result.c = (-0.5f * vector.alpha) - (sqrt3by2 * vector.beta);

            return result;
        }

    } /* namespace transform */
} /* namespace systems */

