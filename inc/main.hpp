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
		observer::mechanic   	mech;
		control::foc      		foc;
		systems::abc         	u_abc;
		systems::alpha_beta  	u_ab;
		systems::abc 			i_abc;
		systems::alpha_beta 	i_ab;
		float               	correction[3];


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
