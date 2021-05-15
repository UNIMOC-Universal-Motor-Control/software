/*
    UNIMOC - Universal Motor Control  2021 Alexander <tecnologic86@gmail.com> Evers

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
#ifndef FREEMASTER_WRAPPER_H_
#define FREEMASTER_WRAPPER_H_


#include <stdint.h>
#include <stdarg.h>
#include "ch.hpp"
#include "hal.h"

namespace modules
{
	namespace freemaster
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

		/**
		 * @brief recorder function
		 * needs to be called in control thread
		 */
		void Recorder(void);
	} // namespace freemaster
} // namespace modules

#endif /* FREEMASTER_WRAPPER_H_ */

