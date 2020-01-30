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
        return sqrt(vector.d*vector.d + vector.q*vector.q);
    }

    /**
     * @brief calculate length of alpha beta vector.
     *
     * @param vector    vector in alpha-beta-system.
     * @retval length of the vector
     */
    float Length(const alpha_beta& vector)
    {
        return sqrt(vector.alpha*vector.alpha + vector.beta*vector.beta);
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
            const float sqrt3by2 = sqrt(3.0f)/2.0f;
            const float _2by3 = 2.0f / 3.0f;

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
            const float sqrt3by2 = sqrt(3.0f)/2.0f;
            abc result;

            result.a = vector.alpha;
            result.b = (-0.5f * vector.alpha) + (sqrt3by2 * vector.beta);
            result.c = (-0.5f * vector.alpha) - (sqrt3by2 * vector.beta);

            return result;
        }

    } /* namespace transform */
} /* namespace systems */

