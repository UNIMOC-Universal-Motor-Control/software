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
#ifndef INC_PAS_HPP_
#define INC_PAS_HPP_


#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "systems.hpp"
#include "controller.hpp"
#include "values.hpp"
#include "settings.hpp"


/**
 * @namespace pedal assist system classes
 */
namespace pas
{
	/**
	 * pedal assist system thread
	 */
	class thread : public chibios_rt::BaseStaticThread<128>
	{
	private:
		static constexpr systime_t 	CYCLE_TIME = TIME_MS2I(1);
		systime_t 					deadline;

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

} /* namespace pas */


#endif /* INC_PAS_HPP_ */
