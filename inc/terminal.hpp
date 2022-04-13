/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2022 Alexander <tecnologic86@gmail.com> Evers

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
#ifndef INC_TERMINAL_HPP_
#define INC_TERMINAL_HPP_

#include <cstdint>
#include "ch.hpp"
#include "hal.h"
#include "usbcfg.h"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"

/**
 * @namespace terminal
 * @brief shell data and api
 *
 */
namespace terminal
{
	/**
	 * controller management thread
	 */
	class oszi : public chibios_rt::BaseStaticThread<1024>
	{
	private:
		static constexpr systime_t CYCLE_TIME = TIME_MS2I(10);
		systime_t 				deadline;
		std::uint32_t			delay;

	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		/**
		 * generic constructor
		 */
		oszi();
	};

} /* namespace terminal */


#endif /* INC_TERMINAL_HPP_ */
