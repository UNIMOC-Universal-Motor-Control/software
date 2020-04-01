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
#ifndef INC_SYSTEMS_HPP_
#define INC_SYSTEMS_HPP_

#include <cstdint>
#include <cmath>
#include <cstring>
#include <climits>
#include <array>

/**
 * @namespace math constants
 */
namespace math {

	/**
	 * compute sqrt in a fast way with sufficient accuracy
	 *
	 * @ref see https://bits.stephan-brumme.com/squareRoot.html
	 *
	 * @param x			value
	 * @return			sqrt(x)
	 */
	inline float sqrt_fast(float x)
	{
		std::uint32_t i;
		std::memcpy(&i, &x, sizeof(std::uint32_t));

		// adjust bias
		i  += 127 << 23;
		// approximation of square root
		i >>= 1;

		std::memcpy(&x, &i, sizeof(float));
		return x;
	}

	constexpr float PI =           std::atan(1.0f) * 4.0f;
	constexpr float E =            2.71828182845904523536;
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
    constexpr float deg2rad = 180.0f / math::PI;
    ///< conversion constant from radians to degree
    constexpr float rad2deg = math::PI / 180.0f;

    ///< convert internal rad angle to degrees
    constexpr float Degrees(float m) { return m * rad2deg; }

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

        /**
         * Systems casting assignment operator
         * @param dq dq system
         * @return alpha beta system
         */
        dq_u& operator= (alpha_beta_u& ab);
    } dq;

    ///< rotation angle sine and cosine value.
    typedef union sin_cos_u{
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

        /**
         * Systems casting assignment operator
         * @param dq dq system
         * @return alpha beta system
         */
        alpha_beta_u& operator= (dq_u& dq);
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
      @param[in]     theta    input value in rad
      @param[out]    out      points to processed sine cosine output
     */
    void SinCos(const float theta, sin_cos& out);


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

