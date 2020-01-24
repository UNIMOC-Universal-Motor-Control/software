/*
    UNIMOC - Universal Motor Control  2019 Alexander <tecnologic86@gmail.com> Brand

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


#ifndef SRC_TERMINAL_HPP_
#define SRC_TERMINAL_HPP_

#include "ch.hpp"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "chprintf.h"


#define TCOL_RED		"\x1b[31m"
#define TCOL_GREEN		"\x1b[32m"
#define TCOL_YELLOW		"\x1b[33m"
#define TCOL_BOLDCYAN	"\x1b[1;36m"
#define TCOL_CYAN		"\x1b[36m"
#define TCOL_RESET		"\x1b[0m"


namespace unimoc
{
	namespace terminal
	{
		/**
		 * Initialize terminal with shell
		 */
		void Init(void);
	} /* namespace terminal */
} /* namespace unimoc */


#endif /* SRC_TERMINAL_HPP_ */
