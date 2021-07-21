/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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
#ifndef UNIMOC_UAVCAN_THREAD_H_
#define UNIMOC_UAVCAN_THREAD_H_


#include <stdint.h>
#include <stdarg.h>
#include "ch.hpp"
#include "hal.h"
#include "canard.h"
#include "o1heap.h"

#define NUNAVUT_ASSERT(x) osalDbgAssert(x, "NUNAVUT");

namespace uavcan
{

	class thread : public chibios_rt::BaseStaticThread<1024>
	{
	private:


	protected:
		/**
		 * Thread function
		 */
		virtual void main(void);

	public:
		thread();
	};

} // namespace uavcan

#endif /* UNIMOC_UAVCAN_THREAD_H_ */

