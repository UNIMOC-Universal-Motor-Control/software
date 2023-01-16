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
#ifndef HARDWARE_CONFIG_HPP_
#define HARDWARE_CONFIG_HPP_


///< Hardware Node Name
#define HARDWARE_NAME "UNIMIC-PHHVr1"

///< Hardware supports random number generation: STM32G473 has TRNG
#define HARDWARE_CAPABIITY_RANDOM 				TRUE

///< Hardware supports CAN-FD: STM32F446 has CAN-FD
#define HARDWARE_CAPABIITY_CAN_FD				TRUE

///< Hardware supports 1 CAN Interface
#define HARDWARE_CAPABIITY_CAN_NO_OF_INTERFACES 1

#endif /* HARDWARE_CONFIG_HPP_ */
