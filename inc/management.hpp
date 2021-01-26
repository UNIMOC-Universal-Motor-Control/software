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
#ifndef INC_MANAGEMENT_HPP_
#define INC_MANAGEMENT_HPP_


#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"


/**
 * @namespace controller management classes
 */
namespace management
{
	/**
	 * @namespace observer flags
	 */
	namespace observer
	{
		///< release flux observer
		extern bool flux;

		///< release high frequency injection observer
		extern bool hfi;

		///< release mechanic observer
		extern bool mechanic;
	}

	/**
	 * @namespace controller flags
	 */
	namespace control
	{
		///< release current control
		extern bool current;

		///< feedforward omega
		extern bool feedforward;

		///< release speed control
		extern bool speed;

		///< release position control
		extern bool position;
	}

	/**
	 * @namespace measurement data
	 */
	namespace measure
	{
		///< measure all parameters
		extern bool all;

		/**
		 * @namespace resistance measurement values
		 */
		namespace r
		{
			///< enable flag
			extern bool enable;

			///< current measurement voltage
			extern float u;

			///< measure all the phases.
			constexpr std::array<float, 3> PHI_STEPS = {0.0f, unit::deg2rad * 120.0f, unit::deg2rad * 240.0f};

			///< current phi step
			extern std::uint8_t phi_step;

			///< cycle counter
			extern std::uint32_t cycle;
		}

		/**
		 * @namespace inductance measurement values
		 */
		namespace l
		{
		///< measurement current.
		constexpr float CUR = 3.0f;

		///< measurement frequency
		constexpr float FREQ = 1600.0f;

		///< enable flag
		extern bool enable;

		///< current measurement voltage
		extern float u;

		///< cycle counter
		extern std::uint32_t cycle;
		}
	}

	/**
	 * controller management thread
	 */
	class thread : public chibios_rt::BaseStaticThread<450>
	{
	private:
		static constexpr systime_t CYCLE_TIME = TIME_MS2I(1);
		systime_t deadline;
		std::uint32_t delay;

		enum state_e
		{
			STARTUP,
			CURRENT_OFFSETS,
			RUN,
			MEASURE_RS,
			CALCULATE_RS,
			MEASURE_LS,
			CALCULATE_LS,
			MEASURE_PSI,
			CALCULATE_PSI,
		} sequencer;

		/**
		 * calculates the optimal current controller proportional gain
		 * @param inductance of the stator
		 * @param t2 filter time constant
		 * @return kp of the current controller
		 */
		constexpr float CalculateKp(const float inductance, const float t2)
		{
			return (inductance/t2*0.5f);
		}

		/**
		 * calculates the time constant of the pi current controller to cancel
		 * the electrical time constant
		 * @param inductance of the stator
		 * @param resitance of the stator
		 * @return tn of the current controller
		 */
		constexpr float CalculateTn(const float inductance, const float resistance)
		{
			return (inductance/resistance);
		}

	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		/**
		 * generic constructor
		 */
		thread();
	};

} /* namespace management */


#endif /* INC_MANAGEMENT_HPP_ */
