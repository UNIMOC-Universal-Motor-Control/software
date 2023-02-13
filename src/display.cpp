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
	setName("Control");

	init();
	/*
	 * Normal main() thread activity
	 */
	while (TRUE)
	{
		std::uint8_t j = 1;
		std::uint8_t crc = 0;
		std::size_t bytes_send = sizeof(tx_buffer);
		std::size_t bytes_received = sizeof(rx_buffer);

		msg = uartReceiveTimeout(uartp, &bytes_received, rx_buffer, OSAL_MS2I(100));
		if (msg != MSG_OK)
		{
			chSysHalt("invalid return code");
		}

		 //printf("Byte recieved \r\n");
		  // validation of the package data
		   ui8_crc = 0;

		   for (ui8_j = 0; ui8_j <= 12; ui8_j++)
		   {
		     if (ui8_j == 5) continue; // don't xor B5 (B7 in our case)
		     ui8_crc ^= ui8_rx_buffer[ui8_j];
		   }

		   // check if end of message is OK
		   if((ui8_rx_buffer[11]==0x32||ui8_rx_buffer[11]==0x37) && ui8_rx_buffer[12]==0x0E ){
			   // check if CRC is ok
		   if (((ui8_crc ^ 10) == ui8_rx_buffer [5]) 	|| // some versions of CRC LCD5 (??)
			((ui8_crc ^ ui8_last_XOR) == ui8_rx_buffer [5])
			)
		   { //printf("message valid \r\n");
		     lcd_configuration_variables.ui8_assist_level = ui8_rx_buffer [1] & 7;
		     lcd_configuration_variables.ui8_light = ui8_rx_buffer [1]>>7 & 1;
		     lcd_configuration_variables.ui8_motor_characteristic = ui8_rx_buffer [3];
		     lcd_configuration_variables.ui8_wheel_size = ((ui8_rx_buffer [4] & 192) >> 6) | ((ui8_rx_buffer [2] & 7) << 2);
		     lcd_configuration_variables.ui8_max_speed = (10 + ((ui8_rx_buffer [2] & 248) >> 3)) | (ui8_rx_buffer [4] & 32);
		     lcd_configuration_variables.ui8_power_assist_control_mode = ui8_rx_buffer [4] & 8;
		     lcd_configuration_variables.ui8_controller_max_current = (ui8_rx_buffer [7] & 15);
		     MS_D->assist_level = lcd_configuration_variables.ui8_assist_level;
		     MP_D->speedLimit = lcd_configuration_variables.ui8_max_speed;

				lcd_configuration_variables.ui8_p1 = ui8_rx_buffer[3];
				lcd_configuration_variables.ui8_p2 = ui8_rx_buffer[4] & 0x07;
				lcd_configuration_variables.ui8_p3 = ui8_rx_buffer[4] & 0x08;
				lcd_configuration_variables.ui8_p4 = ui8_rx_buffer[4] & 0x10;
				lcd_configuration_variables.ui8_p5 = ui8_rx_buffer[0];

				lcd_configuration_variables.ui8_c1 = (ui8_rx_buffer[6] & 0x38) >> 3;
				lcd_configuration_variables.ui8_c2 = (ui8_rx_buffer[6] & 0x37);
				lcd_configuration_variables.ui8_c4 = (ui8_rx_buffer[8] & 0xE0) >> 5;
				lcd_configuration_variables.ui8_c5 = (ui8_rx_buffer[7] & 0x0F);
				lcd_configuration_variables.ui8_c12 = (ui8_rx_buffer[9] & 0x0F);
				lcd_configuration_variables.ui8_c13 = (ui8_rx_buffer[10] & 0x1C) >> 2;
				lcd_configuration_variables.ui8_c14 = (ui8_rx_buffer[7] & 0x60) >> 5;
				if(lcd_configuration_variables.ui8_p1 != ui8_gear_ratio){
						    	 ui8_gear_ratio=lcd_configuration_variables.ui8_p1;
						     }

		     if(lcd_configuration_variables.ui8_light){
		    	 HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
		    	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		     }
		     else{
		    	 HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
		    	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		     }

		     display_update(MS_D);
		     check_recent(); //byte 1 contains the PAS level, that may be changed quite often. Better run only at system shutdown, due to limited possible write cycles to flash
		   }//end CRC OK


		// prepare moving indication info
		moving_indication = 0;
		// if (brake) moving_indication |= (1 << 5);
		// if (cruise_control) moving_indication |= (1 << 3);
		// if (throttle) moving_indication |= (1 << 1);
		// if (pas) moving_indication |= (1 << 4);


		// calc battery pack state of charge (SOC)
		if      (values::battery::u > ((uint16_t) BATTERY_PACK_VOLTS_80)) { battery_soc = 16; } // 4 bars | full
		else if (values::battery::u > ((uint16_t) BATTERY_PACK_VOLTS_60)) { battery_soc = 12; } // 3 bars
		else if (values::battery::u > ((uint16_t) BATTERY_PACK_VOLTS_40)) { battery_soc = 8;  } // 2 bars
		else if (values::battery::u > ((uint16_t) BATTERY_PACK_VOLTS_20)) { battery_soc = 4;  } // 1 bar
		else                                                              { battery_soc = 3;  } // empty

		wheel_period_ms= 4500.0;  // Motor RPM Sensor needs to be added to values
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

		msg = uartSendFullTimeout(uartp, &bytes_send, tx_buffer, OSAL_MS2I(100));
		if (msg != MSG_OK)
		{
			chSysHalt("Display Send Failed, invalid return code");
		}
	}
}

void thread::init(void)
{
	msg_t msg = MSG_OG;
	/*
	 * Activates the UART driver 2.
	 */
	uartStart(uartp, &uart_cfg);
}

/********************************************************************************************/
 // Process received package from the LCD
 //

 // see if we have a received package to be processed
void check_message(MotorState_t* MS_D, MotorParams_t* MP_D)
 {

   else{ //search for right last XOR

	   ui8_crc = 0;

	   for (ui8_j = 0; ui8_j <= 12; ui8_j++)
	   {
	     if (ui8_j == 5) continue; // don't xor B5 (B7 in our case)
	     ui8_crc ^= ui8_rx_buffer[ui8_j];
	   }
	   for (ui8_j = 0; ui8_j <= 50; ui8_j++)
	   {
	     if((ui8_crc ^ ui8_j) == ui8_rx_buffer [5]) ui8_last_XOR = ui8_j;
	   }
   }
   }// end EOT OK
   else{
	  //resyncronize the communication
	       CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
		   DMA1_Channel5->CNDTR=2;
		   SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);

		   if((ui8_rx_buffer[0]==0x32||ui8_rx_buffer[0]==0x37) && ui8_rx_buffer[1]==0x0E ){
	  	   CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
	  	   DMA1_Channel5->CNDTR=13;
	  	   SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
		   }

   }
 }

//check if differences between initial values and recent values, store to emulated EEPROM if necessary
/*
EEPROM_KT_B0_B3
EEPROM_KT_B2_B4
EEPROM_KT_B6_B7
EEPROM_KT_B8_B9
EEPROM_KT_B1_B10
*/
//byte 1 contains the PAS level, that may be changed quite often. Better run only at system shutdown.
void check_recent(void){

	if(ui8_rx_buffer[0]!=ui8_rx_initial_buffer[0] || ui8_rx_buffer[3]!=ui8_rx_initial_buffer[3] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B0_B3,ui8_rx_buffer[0]<<8 | ui8_rx_buffer[3] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[0]=ui8_rx_buffer[0];
	    ui8_rx_initial_buffer[3]=ui8_rx_buffer[3];
	}

	if(ui8_rx_buffer[2]!=ui8_rx_initial_buffer[2] || ui8_rx_buffer[4]!=ui8_rx_initial_buffer[4] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B2_B4,ui8_rx_buffer[2]<<8 | ui8_rx_buffer[4] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[2]=ui8_rx_buffer[2];
	    ui8_rx_initial_buffer[4]=ui8_rx_buffer[4];
	}

	if(ui8_rx_buffer[6]!=ui8_rx_initial_buffer[6] || ui8_rx_buffer[7]!=ui8_rx_initial_buffer[7] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B6_B7,ui8_rx_buffer[6]<<8 | ui8_rx_buffer[7] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[6]=ui8_rx_buffer[6];
	    ui8_rx_initial_buffer[7]=ui8_rx_buffer[7];
	}

	if(ui8_rx_buffer[8]!=ui8_rx_initial_buffer[8] || ui8_rx_buffer[9]!=ui8_rx_initial_buffer[9] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B8_B9,ui8_rx_buffer[8]<<8 | ui8_rx_buffer[9] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[8]=ui8_rx_buffer[8];
	    ui8_rx_initial_buffer[9]=ui8_rx_buffer[9];
	}
//only check Byte 10
	if( ui8_rx_buffer[10]!=ui8_rx_initial_buffer[10] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B1_B10,ui8_rx_buffer[1]<<8 | ui8_rx_buffer[10] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[1]=ui8_rx_buffer[1];
	    ui8_rx_initial_buffer[10]=ui8_rx_buffer[10];
	}


}
