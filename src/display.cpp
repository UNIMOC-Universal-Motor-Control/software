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
#include "display.hpp"


namespace  display
{

///< serial driver configuration
constexpr SerialConfig serial_config =
{
	9600,
	0,
	USART_CR2_STOP1_BITS,
	0
};

/**
 * S-LCD3 to S12SN communication protocol.

Packet consist of 13 bytes. 9600 baud, 8-n-1, byte-by-byte

B0	B1	B2	B3	B4	B5	B6	B7	B8	B9	B10	B11	B12

(e.g: 12 0 149 160 41 102 18 74 4 20 0 50 14)

for the P and C parameters description please see S-LCD3 user manual available at the bmsbattery.com

B0: parameter P5.
B1: assist level, front light.
b7b6b5b4 b3b2b1b0
. . . .  . l2l1l0     assist level 0-5, 6-walk (long push down arrow)
f0. . .  . . . .      bit (mask 0x80) front light, display backlight
B3: parameter P1.
B2 and B4 max speed, wheel size, P2,P3,P4
B2: b7b6b5b4 b3b2b1b0 and B4:b7b6b5b4 b3b2b1b0
    s4s3s2s1 s0. . .         . . s5.  . . . .   max speed minus 10,
    . . . .  . . . .         . . . .  . . . .  km/h;   6bit
    . . . .  . w4w3w2        w1w0. .  . . . .  wheel size:0x0e-10",
    . . . .  . . . .         . . . .  . . . .  0x02-12", 0x06-14",
    . . . .  . . . .         . . . .  . . . .  0x00-16",0x04-18",
    . . . .  . . . .         . . . .  . . . .  0x08-20", 0x0c-22",
    . . . .  . . . .         . . . .  . . . .  0x10-24", 0x14"-26",
    . . . .  . . . .         . . . .  . . . .  0x18-700c
    . . . .  . . . .         . . . .  . p2p1p0  par. P2 (B4&&0x07)
    . . . .  . . . .         . . . .  p0. . .   par. P3 (B4&&0x08)
    . . . .  . . . .         . . . p0 . . . .   par. P4 (B4&&0x10)
Example:
    0 1 1 1  1 . . .         . . 0.   . . . .  25km/h (15+10)
    1 1 1 1  0 . . .         . . 0.   . . . .  40km/h (30+10)
    1 0 0 1  0 . . .         . . 1.   . . . .  60km/h (50+10)
B5: CRC = (xor B0,B1,B2,B3,B4,B6,B7,B8,B9,B10,B11,B12) xor 2.
B6: parameters C1 and C2
b7b6b5b4 b3b2b1b0
. . c2c1 c0. . .       param C1 (mask 0x38)
. . . .  . c2c1c0      param C2 (mask 0x07)
B7: parameter C5 and C7
b7b6b5b4 b3b2b1b0
. . . .  c3c2c1c0      param C5 (mask 0x0F)
. c1c0.  . . . .       param C14 (mask 0x60)
B8: parameter C4
b7b6b5b4 b3b2b1b0
c2c1c0.  . . . .       param C4  (mask 0xE0)
B9: parameter C12
b7b6b5b4 b3b2b1b0
. . . .  c3c2c1c0      param C12  (mask 0x0F)
B10: parameter C13
b7b6b5b4 b3b2b1b0
. . . c2 c1c0. .       param C13  (mask 0x1C)
B11: 50 dec (0x32)
B12: 14 dec (0x0E)
parameters C3, C6, C7, C8, C9, C10 not sent to MCU

if C11 set to 2 (used to copy LCD to LCD), LCD repeatedly sends 23 bytes, byte by byte, no separators, underlines show not identified values


255, wheel diam (in), maxspeed (kmh), level, P1, P2, P3, P4, P5, C1, C2 , C3, C4, C5, C6, C7, C8, 0, 20, C12, C13 ,C14, 55

Example:
255 26 60 0 160 1 1 0 12 2 1 8 0 10 3 0 1 0 20 4 0 2 55
 *
 */

///< Assist Level 0-5, 6-walk (long push down arrow)
std::uint8_t assist_level;

///< Activate Lights
bool light;

///< wheel size in inch
std::uint8_t wheel_size;

///< max speed in km/h
std::uint8_t max_speed;

///< Parameters from the Display FIXME TBD
std::uint8_t p1;
std::uint8_t p2;
std::uint8_t p3;
std::uint8_t p4;
std::uint8_t p5;
std::uint8_t c1;
std::uint8_t c2;
std::uint8_t c4;
std::uint8_t c5;
std::uint8_t c12;
std::uint8_t c13;
std::uint8_t c14;
/**
	S12SN to LCD3 communication protocol.

	Packet consist of 12 bytes. 9600 baud, 8-n-1, byte-by-byte, no separators

	B0	B1	B2	B3	B4	B5	B6	B7	B8	B9	B10	B11

	(e.g: 65 16 48 0 139 0 164 2 13 0 0 0)

	B0: 65 dec (0x41)
	B1: battery level: 0: empty box, 1: border flashing, 2: animated charging, 3: empty, 4: 1 bar, 8: 2 bars, 16: 4 bars (full),
	B2: 48 dec (0x30)
	B3,B4: speed, wheel rotation period, ms; period(ms)=B3*256+B4;
	B5 error info display: 0x20: "0info", 0x21: "6info", 0x22: "1info", 0x23: "2info", 0x24: "3info", 0x25: "0info", 0x26: "4info", 0x28: "0info"
	B6: CRC: xor B0,B1,B2,B3,B4,B5,B7,B8,B9,B10,B11
	B7: moving mode indication, bit
	b7b6b5b4 b3b2b1b0
	. . . .  . . . m0      if set animated circle "throttle"
	. . . .  m0 . . .      if set "C" (cruise) is shown
	. . . m0  . . . .      if set "assist" shown
	. . m0 .  . . . .      if set "brake" shown
			// if (brake) moving_indication |= (1 << 5);
			// if (cruise_control) moving_indication |= (1 << 3);
			// if (throttle) moving_indication |= (1 << 1);
			// if (pas) moving_indication |= (1 << 4);
	B8: power in 13 wt increments (48V version of the controller)
	B9: motor temperature, can be negative or positive,T(C)=(int8)B8+15,
		if temperature > 120C LCD screen is flashing.
		e.g 0xDA T=-23C, 0x34 T=67C
	B10: 0
	B11: 0
 **/
///< Error shown on Display
/// 0x20: "0info", 0x21: "6info", 0x22: "1info", 0x23: "2info", 0x24: "3info", 0x25: "0info", 0x26: "4info", 0x28: "0info"
std::uint8_t error = 0;

///< moving indication in display
enum moving_indication_e
{
	MOVING_INDICATION_NONE = 0,
	MOVING_INDICATION_THOTTLE = 1 << 1,
	MOVING_INDICATION_CRUISE = 1 << 3,
	MOVING_INDICATION_ASSIST = 1 << 4,
	MOVING_INDICATION_BRAKE = 1 << 5,
} moving_indication = MOVING_INDICATION_NONE;

///< battery state of charge
enum soc_e
{
	SOC_EMPTY_BOX = 0,
	SOC_FLASHING = 1,
	SOC_CHARGING = 2,
	SOC_EMPTY = 3,
	SOC_1_BAR = 4,
	SOC_2_BAR = 8,
	SOC_3_BAR = 12,
	SOC_4_BAR = 16,
} soc = SOC_CHARGING;

/**
 * generic constructor
 */
thread::thread()
{}

/**
 * @brief Thread main function
 */
void thread::main(void)
{
	using namespace values;
	setName("Display");

	init();

	soc = SOC_3_BAR;
	moving_indication = MOVING_INDICATION_ASSIST;

	/*
	 * Normal main() thread activity
	 */
	while (TRUE)
	{
		std::uint8_t j;
		std::uint8_t crc = 0;
		std::uint8_t last_XOR = 2;
		msg_t msg = MSG_OK;

		msg = sdReadTimeout(sdp, rx_buffer, sizeof(rx_buffer), OSAL_MS2I(50));
		if (msg == MSG_OK)
		{
			// validation of the package data
			for (j = 0; j < sizeof(rx_buffer); j++)
			{
				if (j == 5) continue; // don't xor B5
				crc ^= rx_buffer[j];
			}

			// check if end of message is OK
			if((	(rx_buffer[11]==0x32)
					||  (rx_buffer[11]==0x37) )
					&&  (rx_buffer[12]==0x0E) )
			{
				// check if CRC is ok
				if (	((crc ^ 10) == rx_buffer[5])
						|| 	((crc ^ last_XOR) == rx_buffer[5]) ) // some versions of CRC LCD5 (??)
				{

					assist_level 				= rx_buffer[1] & 0x07;
					light 						= rx_buffer[1] & 0x08;
					wheel_size 					= ((rx_buffer [4] & 192) >> 6) | ((rx_buffer [2] & 7) << 2);
					max_speed 					= (10 + ((rx_buffer [2] & 248) >> 3)) | (rx_buffer [4] & 32);

					p1 	= rx_buffer[3];
					p2 	= rx_buffer[4] & 0x07;
					p3 	= rx_buffer[4] & 0x08;
					p4 	= rx_buffer[4] & 0x10;
					p5 	= rx_buffer[0];

					c1 	= (rx_buffer[6] & 0x38) >> 3;
					c2 	= (rx_buffer[6] & 0x37);
					c4 	= (rx_buffer[8] & 0xE0) >> 5;
					c5 	= (rx_buffer[7] & 0x0F);
					c12	= (rx_buffer[9] & 0x0F);
					c13	= (rx_buffer[10] & 0x1C) >> 2;
					c14	= (rx_buffer[7] & 0x60) >> 5;

				}//end CRC OK
				else
				{ //search for right last XOR
					crc = 0;

					for (j = 0; j < sizeof(rx_buffer); j++)
					{
						if (j == 5) continue; // don't xor B5
						crc ^= rx_buffer[j];
					}

					for (j = 0; j <= 50; j++)
					{
						if((crc ^ j) == rx_buffer [5]) last_XOR = j;
					}
				}
			}

			std::uint16_t wheel_period_ms = static_cast<std::uint16_t>(1000.0f * settings.motor.P * (math::_2PI) / values::motor::rotor::omega);

			tx_buffer [0] =  65;
			// B1: battery level
			tx_buffer [1] = soc;
			// B2: Controller Voltage, just fixed
			tx_buffer [2] = 48;
			// B3: speed, wheel rotation period, ms; period(ms)=B3*256+B4; wheel speed as ms per rotation
			tx_buffer [3] = (wheel_period_ms >> 8) & 0xff;
			tx_buffer [4] = (wheel_period_ms) & 0xff;

			// B5: error info display
			tx_buffer [5] = error;
			// B6: CRC: xor B1,B2,B3,B4,B5,B7,B8,B9,B10,B11
			// 0 value so no effect on xor operation for now
			tx_buffer [6] = 0;
			// B7: moving mode indication, bit
			// throttle: 2
			tx_buffer [7] = moving_indication;
			// B8: 4x controller current
			// Vbat = 30V:
			// - B8 = 255, LCD shows 1912 watts
			// - B8 = 250, LCD shows 1875 watts
			// - B8 = 100, LCD shows 750 watts
			tx_buffer [8] =  static_cast<uint8_t>(values::crank::power / 13.0f);  //13W pro digit
			// B9: temperature with 15 degrees C offset. 0 => 15C
			tx_buffer [9] = static_cast<int8_t>(values::converter::temp - 15.0f);
			// B10 and B11: 0
			tx_buffer [10] = 0;
			tx_buffer [11] = 0;

			// calculate CRC xor
			crc = 0;
			for (j = 1; j <= 11; j++)
			{
				crc ^= tx_buffer[j];
			}
			tx_buffer [6] = crc;

			msg = sdWriteTimeout(sdp, tx_buffer, sizeof(tx_buffer), OSAL_MS2I(300));
			if (msg != MSG_OK)
			{
				/// FIXME Report error
			}
		}
	}
}

void thread::init(void)
{
	sdp = HARDWARE_DISP_SERIAL;
	/*
	 * Activates the UART driver.
	 */
	sdStart(sdp, &serial_config);
}

} /* namespace display */
