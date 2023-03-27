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

    Credits to stancecoke's EBiCS/EBiCS_Firmware
 */
#ifndef INC_DISPLAY_HPP_
#define INC_DISPLAY_HPP_

#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "hal.h"
#include "controller.hpp"


namespace  display
{
	/**
	 * Display Thread
	 */
	class thread : public chibios_rt::BaseStaticThread<2048>
	{
	private:
		std::uint8_t tx_buffer[12];
		std::uint8_t rx_buffer[13];

		SerialDriver *sdp;
		/**
		 * initialize thread
		 */
		void init(void);

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

} /* namespace display */

#endif /* INC_DISPLAY_HPP_ */

