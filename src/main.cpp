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

#include "ch.hpp"
#include "hal.h"
#include "usbcfg.h"
#include "hardware_interface.hpp"


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

float dutys[unimoc::hardware::PHASES] = { -0.95f,  0.95f,   0.0f};


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

/**
 * Code entry point
 * @return never
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
	 * Initializes two serial-over-USB CDC drivers.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg1);
	sduObjectInit(&SDU2);
	sduStart(&SDU2, &serusbcfg2);

	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusbcfg1.usbp);
	chThdSleepMilliseconds(1500);
	usbStart(serusbcfg1.usbp, &usbcfg);
	usbConnectBus(serusbcfg1.usbp);

	/*
	 * start led sequencer threads for cpp testing
	 */
	blinker1.start(NORMALPRIO);
	blinker2.start(NORMALPRIO);
	blinker3.start(NORMALPRIO);
	blinker4.start(NORMALPRIO);

	palSetLineMode(LINE_HALL_A, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_HALL_B, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_HALL_C, PAL_MODE_OUTPUT_PUSHPULL);

	unimoc::hardware::pwm::Init();
	unimoc::hardware::adc::Init();


	unimoc::hardware::pwm::SetDutys(dutys);
	unimoc::hardware::pwm::EnableOutputs();
	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (true)
	{

		BaseThread::sleep(TIME_MS2I(2000));
	}
}
