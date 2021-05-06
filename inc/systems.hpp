/*
    UNIMOC - Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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
#ifndef INC_SYSTEMS_HPP_
#define INC_SYSTEMS_HPP_

#include <cstdint>
#include <cmath>
#include <cstring>
#include <limits>
#include <array>
#include <math.h>

/**
 * @namespace math constants
 */
namespace math {
	constexpr float PI =           M_PI;
	constexpr float E =            2.71828182845904523536f;
	constexpr float _2PI =         2.0f * PI;
	constexpr float PIby2 =        PI / 2.0f;
	constexpr float SQRT2 =        std::sqrt(2.0f);
	constexpr float SQRT1_2 =      1.0f/std::sqrt(2.0f);
};

/**
 * @namespace unit constants
 */
namespace unit
{
    ///< convert internal Vs/rad to rpm/V
    constexpr float VsRad2RpmV(float m) { return 60.0f/(m * math::_2PI);}

    ///< convert rpm/V to Vs/rad
    constexpr float RpmV2VsRad(float m) { return 60.0f/(m * math::_2PI);}

    ///< conversion constant from degree to radians
    constexpr float deg2rad = math::PI / 180.0f;
    ///< conversion constant from radians to degree
    constexpr float rad2deg = 180.0f / math::PI;
    ///< conversion constant from q31 to degree
    constexpr float q312deg = 180.0f / (float)std::numeric_limits<std::int32_t>::max();
    ///< conversion constant from q31 to rad
    constexpr float q312rad = math::PI / (float)std::numeric_limits<std::int32_t>::max();
    ///< conversion constant from rad to q31
    constexpr float rad2q31 = (float)std::numeric_limits<std::int32_t>::max() / math::PI;
    ///< conversion constant from deg to q31
    constexpr float deg2q31 = (float)std::numeric_limits<std::int32_t>::max() / 180.0f;

    ///< convert internal q31 angle to degrees
    constexpr float Degrees(std::int32_t m) { return (float)m * q312deg; }
    ///< convert internal q31 angle to degrees
    constexpr float Rad(std::int32_t m) { return (float)m * q312rad; }
    ///< convert rad angle to internal q31
    constexpr std::int32_t Q31R(float m) { return (std::int32_t)(m * rad2q31); }
    ///< convert rad angle to internal q31
    constexpr std::int32_t Q31D(float m) { return (std::int32_t)(m * deg2q31); }

    ///< scale from radians per second to rpm
    constexpr float rad_s2rpm = 60.0f / math::_2PI;
    ///< scale from rpm to radians per second
    constexpr float rpm2rad_s = 1.0f / rad_s2rpm;

    ///< convert internal omega in rad/s to rpm
    constexpr float RPM(float m) { return m * rad_s2rpm; }

    ///< convert rpm to internal omega in rad/s
    constexpr float RadS(float m) { return m * rpm2rad_s; }
} /* namespace unit */

/**
 * @namespace coordinate systems.
 */
namespace systems
{

	union alpha_beta_u;

    ///< rotating dq system.
    typedef union dq_u {
        std::array<float,2> array;
        struct {
            float d;
            float q;
        };
    } dq;

    ///< rotation angle sine and cosine value.
    typedef union sin_cos_u {
        std::array<float, 2> array;
        struct {
            float sin;
            float cos;
        };
    } sin_cos;

    ///< stationary alpha beta system.
    typedef union alpha_beta_u {
        std::array<float, 2> array;
        struct {
            float alpha;
            float beta;
        };
    } alpha_beta;


    ///< stationary 3 phase a b c system.
    typedef union abc_u {
        std::array<float, 3> array;
        struct {
            float a;
            float b;
            float c;
        };
    } abc;


    /**
     * @brief calculate length of dq vector.
     *
     * @param vector	vector in dq-system.
     * @retval length of the vector
     */
    float Length(const dq& vector);

    /**
     * @brief calculate length of alpha beta vector.
     *
     * @param vector	vector in alpha-beta-system.
     * @retval length of the vector
     */
    float Length(const alpha_beta& vector);

    /**
      @brief         Floating-point sine and cosine function.
      @param 	     theta    input value in rad
      @retval	     out      points to processed sine cosine output
     */
    sin_cos SinCos(const std::int32_t theta);

    /**
      @brief         Calculate angle difference from sin/cosine values.
      @param 	     a    	sin/cosine of the angle a in c = a - b
      @param 	     b    	sin/cosine of the angle b in c = a - b
      @retval	     c      angle difference (sin(c)) but ok for small values
     */
    float SinCosDiff(const sin_cos& a, const sin_cos& b);


    /**
     * @namespace coordinate system transformations.
     */
    namespace transform
    {
        /**
         * @brief transform alpha beta vector to dq vector.
         *
         * @note https://de.wikipedia.org/wiki/D/q-Transformation
         *
         * @param vector	vector in alpha-beta-system.
         * @param angle		sine and cosine of the rotor angle.
         * @retval vector in dq-system
         */
        dq Park(const alpha_beta &vector, const sin_cos& angle);

        /**
         * @brief transform abc 3 phase vector to alpha beta vector.
         *
         * @note https://de.wikipedia.org/wiki/Clarke-Transformation
         *
         * @param vector	vector in abc-system.
         * @retval vector in alpha-beta-system
         */
        alpha_beta Clark(const abc& vector);

        /**
         * @brief transform dq vector to alpha beta vector.
         *
         * @note https://de.wikipedia.org/wiki/D/q-Transformation
         *
         * @param vector	vector in dq-system.
         * @param angle		sine and cosine of the rotor angle.
         * @retval vector in alpha-beta-system
         */
        alpha_beta InversePark(const dq& vector, const sin_cos& angle);

        /**
         * @brief transform alpha beta vector to abc 3 phase vector.
         *
         * @note https://de.wikipedia.org/wiki/Clarke-Transformation
         *
         * @param vector	vector in alpha beta system.
         * @retval vector in abc-system
         */
        abc InverseClark(const alpha_beta& vector);

    } /* namespace transform */
} /* namespace systems */

#endif /* INC_SYSTEMS_HPP_ */

