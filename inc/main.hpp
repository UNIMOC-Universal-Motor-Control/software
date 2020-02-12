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
#ifndef INC_MAIN_HPP_
#define INC_MAIN_HPP_


#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "controller.hpp"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"
#include "observer.hpp"

/**
 * @namespace controller classes
 */
namespace control
{

	/**
	 * generic FOC controller thread
	 */
	class thread : public chibios_rt::BaseStaticThread<256>
	{
	private:
		observer::flux       	flux;
		observer::admittance    admittance;
		observer::mechanic   	mech;
		control::foc      		foc;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>  	u_abc;
		systems::alpha_beta  	u_ab;
		systems::abc 			i_abc;
		systems::alpha_beta 	i_ab;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> i_dc;
		std::array<systems::abc, hardware::pwm::INJECTION_CYCLES> i_ac;
		std::array<systems::alpha_beta, hardware::pwm::INJECTION_CYCLES> i_ab_ac;
		systems::alpha_beta 	y_ab;
		systems::dq			 	y_dq;
		std::array<float, 3>   	correction;


		/**
		 * calculate the mean of a hole injection cycle of adc measurements
		 * @param currents referes to the samples of one hole injection cycle
		 * @return the mean per phase of the injection cycle
		 */
		systems::abc InjectionMean(const std::array<systems::abc, hardware::pwm::INJECTION_CYCLES>& currents);

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

} /* namespace control */


#endif /* INC_MAIN_HPP_ */
