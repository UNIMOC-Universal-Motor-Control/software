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
#ifndef INC_MANAGEMENT_HPP_
#define INC_MANAGEMENT_HPP_


#include <cstdint>
#include <cmath>
#include <climits>
#include <array>
#include "ch.hpp"
#include "systems.hpp"
#include "values.hpp"
#include "settings.hpp"


/**
 * @namespace controller management classes
 */
namespace management
{
	/**
	 * @namespace observer flags
	 */
	namespace observer
	{
		///< release flux observer
		extern bool flux;

		///< release mechanic observer
		extern bool mechanic;
	}

	/**
	 * @namespace controller flags
	 */
	namespace control
	{
		///< release current control
		extern bool current;

		///< release speed control
		extern bool speed;

		///< release position control
		extern bool position;
	}

	/**
	 * controller management thread
	 */
	class thread : public chibios_rt::BaseStaticThread<350>
	{
	private:
		static constexpr systime_t CYCLE_TIME = TIME_MS2I(1);
		systime_t deadline;
		std::uint32_t delay;

		enum state_e
		{
			STARTUP,
			CURRENT_OFFSETS,
			RUN,
		} sequencer;


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

} /* namespace management */


#endif /* INC_MANAGEMENT_HPP_ */
