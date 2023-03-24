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

	/*
	 * Normal main() thread activity
	 */
	while (TRUE)
	{
		std::uint8_t j;
		std::uint8_t crc = 0;
		std::uint8_t last_XOR = 0;
		msg_t msg = MSG_OK;
		std::size_t bytes_send = sizeof(tx_buffer);
		std::size_t bytes_received = sizeof(rx_buffer);

		msg = uartReceiveTimeout(uartp, &bytes_received, rx_buffer, OSAL_MS2I(300));
		if (msg != MSG_OK)
		{
			continue;
		}
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

				config.assist_level 				= rx_buffer[1] & 0x07;
				config.light 						= rx_buffer[1] & 0x08;
				config.motor_characteristic 		= rx_buffer [3];
				config.wheel_size 					= ((rx_buffer [4] & 192) >> 6) | ((rx_buffer [2] & 7) << 2);
				config.max_speed 					= (10 + ((rx_buffer [2] & 248) >> 3)) | (rx_buffer [4] & 32);
				config.power_assist_control_mode 	= rx_buffer [4] & 8;
				config.controller_max_current 		= rx_buffer [7] & 15;

				config.p1 	= rx_buffer[3];
				config.p2 	= rx_buffer[4] & 0x07;
				config.p3 	= rx_buffer[4] & 0x08;
				config.p4 	= rx_buffer[4] & 0x10;
				config.p5 	= rx_buffer[0];

				config.c1 	= (rx_buffer[6] & 0x38) >> 3;
				config.c2 	= (rx_buffer[6] & 0x37);
				config.c4 	= (rx_buffer[8] & 0xE0) >> 5;
				config.c5 	= (rx_buffer[7] & 0x0F);
				config.c12	= (rx_buffer[9] & 0x0F);
				config.c13 	= (rx_buffer[10] & 0x1C) >> 2;
				config.c14 	= (rx_buffer[7] & 0x60) >> 5;
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

		// prepare moving indication info
		std::uint8_t moving_indication = 0;
		// if (brake) moving_indication |= (1 << 5);
		// if (cruise_control) moving_indication |= (1 << 3);
		// if (throttle) moving_indication |= (1 << 1);
		// if (pas) moving_indication |= (1 << 4);

		// calc battery pack state of charge (SOC)
		std::uint8_t battery_soc = 3; // empty
		if      (values::battery::u > ((uint16_t) BATTERY_PACK_VOLTS_80)) { battery_soc = 16; } // 4 bars | full
		else if (values::battery::u > ((uint16_t) BATTERY_PACK_VOLTS_60)) { battery_soc = 12; } // 3 bars
		else if (values::battery::u > ((uint16_t) BATTERY_PACK_VOLTS_40)) { battery_soc = 8;  } // 2 bars
		else if (values::battery::u > ((uint16_t) BATTERY_PACK_VOLTS_20)) { battery_soc = 4;  } // 1 bar

		std::uint16_t wheel_period_ms= 4500.0;  // Motor RPM Sensor needs to be added to values
		tx_buffer [0] =  65;
		// B1: battery level
		tx_buffer [1] = battery_soc;
		// B2: 24V controller
		tx_buffer [2] = (uint8_t) COMMUNICATIONS_BATTERY_VOLTAGE;
		// B3: speed, wheel rotation period, ms; period(ms)=B3*256+B4;
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
		tx_buffer [8] =  (uint8_t)(values::battery::u * values::battery::i / 7.5f);  //13W pro digit
		// B9: motor temperature
		tx_buffer [9] = (uint8_t)(values::converter::temp - 15.0); //according to documentation at endless sphere
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

		msg = uartSendFullTimeout(uartp, &bytes_send, tx_buffer, OSAL_MS2I(300));
		if (msg != MSG_OK)
		{
			chSysHalt("Display Send Failed, invalid return code");
		}

		sleep(OSAL_MS2I(100));
	}
}

void thread::init(void)
{
	uartp = HARDWARE_DISP_UART;
	/*
	 * Activates the UART driver 2.
	 */
	uartStart(uartp, &uart_cfg);
}

} /* namespace display */
