/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.hpp"
#include "hal.h"


using namespace chibios_rt;


/* Reference to the server thread.*/
static ThreadReference sref;

/*
 * LED blink sequences.
 * NOTE: Sequences must always be terminated by a GOTO instruction.
 * NOTE: The sequencer language could be easily improved but this is outside
 *       the scope of this demo.
 */
#define SLEEP           0
#define GOTO            1
#define STOP            2
#define BITCLEAR        3
#define BITSET          4
#define MESSAGE         5

typedef struct {
	uint8_t       action;
	union {
		msg_t       msg;
		uint32_t    value;
		ioline_t    line;
	};
} seqop_t;

// Flashing sequence for LED3.
static const seqop_t LED3_sequence[] =
{
		{BITSET,      LINE_LED_RUN},
		{SLEEP,       800},
		{BITCLEAR,    LINE_LED_RUN},
		{SLEEP,       200},
		{GOTO,        0}
};

// Flashing sequence for LED4.
static const seqop_t LED4_sequence[] =
{
		{BITSET,      LINE_LED_MODE},
		{SLEEP,       600},
		{BITCLEAR,    LINE_LED_MODE},
		{SLEEP,       400},
		{GOTO,        0}
};

// Flashing sequence for LED5.
static const seqop_t LED5_sequence[] =
{
		{BITSET,      LINE_LED_ERROR},
		{SLEEP,       400},
		{BITCLEAR,    LINE_LED_ERROR},
		{SLEEP,       600},
		{GOTO,        0}
};

// Flashing sequence for LED6.
static const seqop_t LED6_sequence[] =
{
		{BITSET,      LINE_LED_PWM},
		{SLEEP,       200},
		{BITCLEAR,    LINE_LED_PWM},
		{SLEEP,       800},
		{GOTO,        0}
};


/*
 * Sequencer thread class. It can drive LEDs or other output pins.
 * Any sequencer is just an instance of this class, all the details are
 * totally encapsulated and hidden to the application level.
 */
class SequencerThread : public BaseStaticThread<128>
{
private:
	const seqop_t *base, *curr;                   // Thread local variables.

protected:
	void main(void) override {

		setName("sequencer");

		while (true) {
			switch(curr->action) {
			case SLEEP:
				sleep(TIME_MS2I(curr->value));
				break;
			case GOTO:
				curr = &base[curr->value];
				continue;
			case STOP:
				return;
			case BITCLEAR:
				palClearLine(curr->line);
				break;
			case BITSET:
				palSetLine(curr->line);
				break;
			}
			curr++;
		}
	}

public:
	SequencerThread(const seqop_t *sequence) : BaseStaticThread<128>() {

		base = curr = sequence;
	}
};



/* Static threads instances.*/
static SequencerThread blinker1(LED3_sequence);
static SequencerThread blinker2(LED4_sequence);
static SequencerThread blinker3(LED5_sequence);
static SequencerThread blinker4(LED6_sequence);

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  System::init();

  
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    BaseThread::sleep(TIME_MS2I(500));
  }
}
