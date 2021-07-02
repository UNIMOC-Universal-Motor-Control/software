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

namespace uavcan
{

	class thread : public chibios_rt::BaseStaticThread<1024>
	{
	private:

		///< heap size for the o1heap instance for UAVCAN frames
		static constexpr size_t O1HEAP_SIZE = 4096;

		///< heap buffer for o1heap instance
		static char o1heap[O1HEAP_SIZE];

		///< o1heap allocator instance pointer
		static O1HeapInstance* o1allocator;

		///> libcanard instance for UAVCAN communication
		static CanardInstance canard;

		/**
		 * @fn void O1Allocate*(CanardInstance* const, const size_t)
		 * @brief o1heap allocation wrapper for libcanard
		 *
		 * @param ins		libcanard instance
		 * @param amount	size to allocate
		 */
		static void* O1Allocate(CanardInstance* const ins, const size_t amount);

		/**
		 * @fn void O1Free(CanardInstance* const, void* const)
		 * @brief o1heap free memory wrapper for libcanard
		 *
		 * @param ins		libcanard instance
		 * @param pointer	size to allocate
		 */
		static void O1Free(CanardInstance* const ins, void* const pointer);


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

