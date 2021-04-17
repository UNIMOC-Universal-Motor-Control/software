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


constexpr uint32_t FAST_MATH_TABLE_SIZE = 512;
/**
  @par
  Example code for the generation of the floating-point sine table:
  <pre>
  tableSize = 512;
  for (n = 0; n < (tableSize + 1); n++)
  {
 	sinTable[n] = sin(2*PI*n/tableSize);
  }</pre>
 @par
  where PI value is  3.14159265358979
 */
constexpr float SINE_TABLE[FAST_MATH_TABLE_SIZE + 1] =
{
		0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
		0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
		0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
		0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
		0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
		0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
		0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
		0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
		0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
		0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
		0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
		0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
		0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
		0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
		0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
		0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
		0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
		0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
		0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
		0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
		0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
		0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
		0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
		0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
		0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
		0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
		0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
		0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
		0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
		0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
		0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
		0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
		0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
		0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
		0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
		0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
		0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
		0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
		0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
		0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
		0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
		0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
		0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
		-0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
		-0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
		-0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f,
		-0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f,
		-0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f,
		-0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f,
		-0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f,
		-0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f,
		-0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f,
		-0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f,
		-0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f,
		-0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f,
		-0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f,
		-0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f,
		-0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f,
		-0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f,
		-0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f,
		-0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f,
		-0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f,
		-0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f,
		-0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f,
		-0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f,
		-0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f,
		-0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f,
		-0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f,
		-0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f,
		-0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f,
		-0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f,
		-0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f,
		-0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f,
		-0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f,
		-0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f,
		-0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f,
		-0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f,
		-0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f,
		-0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f,
		-0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f,
		-0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f,
		-0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f,
		-0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f,
		-0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f,
		-0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f,
		-0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f,
		-0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f,
		-0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f,
		-0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f,
		-0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f,
		-0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f,
		-0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f,
		-0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f,
		-0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};


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
      @brief         Floating-point sine and cosine function.
      @param 	     theta   input value in q31
      @retval	     out     points to processed sine cosine output
     */
    sin_cos SinCos(const std::int32_t theta)
    {
    	sin_cos out = {0.0f, 0.0f};
    	constexpr float _1by2PI = 0.5f/(float)std::numeric_limits<std::int32_t>::max();
    	float Dn = math::_2PI / FAST_MATH_TABLE_SIZE;    /* delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE */
    	float fract, in;                                 /* Temporary input, output variables */
    	uint16_t indexS, indexC;                         /* Index variable */
    	float f1, f2, d1, d2;                            /* Two nearest output values */
    	float Df;
    	float temp, findex;

    	/* input x is in rad */
    	/* Scale input, divide input by 360 aka 2*pi, for cosine add 0.25 (pi/2) to read sine table */
    	in = theta * _1by2PI;

    	if (in < 0.0f)
    	{
    		in = -in;
    	}

    	in = in - (int32_t)in;

    	/* Calculate the nearest index */
    	findex = (float)FAST_MATH_TABLE_SIZE * in;
    	indexS = ((uint16_t)findex) & 0x1ff;
    	indexC = (indexS + (FAST_MATH_TABLE_SIZE / 4)) & 0x1ff;

    	/* Calculation of fractional value */
    	fract = findex - (float) indexS;

    	/* Read two nearest values of input value from the cos & sin tables */
    	f1 =  SINE_TABLE[indexC  ];
    	f2 =  SINE_TABLE[indexC+1];
    	d1 = -SINE_TABLE[indexS  ];
    	d2 = -SINE_TABLE[indexS+1];

    	temp = (1.0f - fract) * f1 + fract * f2;

    	Df = f2 - f1;          /* delta between the values of the functions */

    	temp = Dn * (d1 + d2) - 2 * Df;
    	temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
    	temp = fract * temp + d1 * Dn;

    	/* Calculation of cosine value */
    	out.cos = fract * temp + f1;

    	/* Read two nearest values of input value from the cos & sin tables */
    	f1 = SINE_TABLE[indexS  ];
    	f2 = SINE_TABLE[indexS+1];
    	d1 = SINE_TABLE[indexC  ];
    	d2 = SINE_TABLE[indexC+1];

    	temp = (1.0f - fract) * f1 + fract * f2;

    	Df = f2 - f1; // delta between the values of the functions
    	temp = Dn * (d1 + d2) - 2 * Df;
    	temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
    	temp = fract * temp + d1 * Dn;

    	/* Calculation of sine value */
    	out.sin = fract * temp + f1;

    	if (theta < 0)
    	{
    		out.sin = -out.sin;
    	}
    	return out;
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

