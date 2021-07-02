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
#include "uavcan.hpp"

///< heap buffer for o1heap instance
char uavcan::thread::o1heap[O1HEAP_SIZE];

///< o1heap allocator instance pointer
O1HeapInstance* uavcan::thread::o1allocator;

///> libcanard instance for UAVCAN communication
CanardInstance uavcan::thread::canard;

/**
 * @fn void O1Allocate*(CanardInstance* const, const size_t)
 * @brief o1heap allocation wrapper for libcanard
 *
 * @param ins		libcanard instance
 * @param amount	size to allocate
 */
void* uavcan::thread::O1Allocate(CanardInstance* const ins, const size_t amount)
{
    (void) ins;
    return o1heapAllocate(o1allocator, amount);
}

/**
 * @fn void O1Free(CanardInstance* const, void* const)
 * @brief o1heap free memory wrapper for libcanard
 *
 * @param ins		libcanard instance
 * @param pointer	size to allocate
 */
void uavcan::thread::O1Free(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    o1heapFree(o1allocator, pointer);
}

/**
 * @brief Thread main function
 */
void uavcan::thread::main(void)
{
	setName("UAVCAN");

	// initialize heap for lib canard
	o1allocator = o1heapInit(o1heap, O1HEAP_SIZE, nullptr, nullptr);

	// map o1heap to canard instance
	canard = canardInit(O1Allocate, O1Free);
	canard.mtu_bytes = CANARD_MTU_CAN_CLASSIC;  // Defaults to 64 (CAN FD); here select Classic CAN.
	canard.node_id   = 42;                      // Defaults to anonymous; can be set up later at any point.

	/*
	 * Normal main() thread activity, sleeping in a loop.
	 */
	while (TRUE)
	{
		sleep(100);
	}

}

/**
 * @brief constructor of UAVCAN thread
 */
uavcan::thread::thread(void){}


/** \} **/ /* end of doxygen group */
/*-- EOF --*/
