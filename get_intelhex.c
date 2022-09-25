/** @addtogroup 05 dbasemgt Database Management
 *
 * @file <b>Database maintenance routines - Get IntelHex file from console.
 *
 * @brief <b>Database maintenance routines - Get IntelHex file from console.</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 25 September 2022
 *
 * This library executes functions to interface and control a PS/2 keyboard, like:
 * power control of a PS/2 key, general interface to read events and write commands to PS/2
 * keyboard, including interrupt service routines on the STM32F4 and STM32F1 series of ARM
 * Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms ref lgpl_license
 */

/*
 * This file was part of the libopencm3 project, but highly modifyed.
 *
 * Copyright (C) 2021 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

//Use Tab width=2

//This original SW is compiled to a Sharp/Epcom MSX HB-8000 and a brazilian ABNT2 PS/2 keyboard (ID=275)
//But it is possible to update the table sending a Intel Hex File through serial, using the
//following communication configuration:
//Speed: 115200 bps
//8 bits
//No parity
//1 Stop bit
//no hardware flow control


#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "system.h"
#include "get_intelhex.h"
#include "ps2handl.h"
#include "serial.h"


//Processor related sizes and adress:
#define USART_ECHO_EN 						1	//All chars received in Intel Hex are echoed in serial routines
#define STRING_MOUNT_BUFFER_SIZE	128


//Global var area:
bool error_intel_hex;
bool abort_intelhex_reception;
extern bool compatible_database;									//Declared on dbasemgt.c
extern uint32_t *base_of_database;								//Declared on msxmap.cpp
extern uint8_t UNUSED_DATABASE[DB_NUM_COLS];			//Declared on msxmap.cpp
extern int usb_configured;												//Declared on cdcacm.c
extern uint32_t systicks;													//Declared on sys_timer.cpp


//Prototype area:
//Usart operations
void usart_get_string_line(uint8_t*, uint16_t);
//Intel Hex operations
void usart_get_intel_hex(uint8_t*, uint16_t, uint8_t*);
bool validate_intel_hex_record(uint8_t*, uint8_t*, uint8_t*, uint16_t*, uint8_t*);


void get_intelhex_to_RAM(uint8_t *ram_buffer, uint16_t ram_buffer_max_size)
{
	(void) ram_buffer_max_size;
	uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];
	error_intel_hex = true;
	abort_intelhex_reception = false;
	while (error_intel_hex == true)
	{
		usart_get_intel_hex(str_mount, STRING_MOUNT_BUFFER_SIZE, ram_buffer);
		if (error_intel_hex == true)
		{
			con_send_string((uint8_t*)"\r\n\n\n\nERROR in Intel Hex. ERROR\r\n\nPlease resend the Intel Hex...");
		}
		else if (abort_intelhex_reception)
		{
			con_send_string((uint8_t*)"\r\n\n\n\n!!!!!INTERRUPTED!!!!!\r\n\nPlease resend the Intel Hex...");
		}
		else
			return;
	}
}



void usart_get_string_line(uint8_t *ser_inp_line, uint16_t str_max_size)
{ //Reads until CR and returns an ASCIIZ on *ser_inp_line
	uint8_t		sign = 0;
	uint16_t	iter = 0;
	uint32_t	lastsysticks;
	bool			print_message;
	uint8_t		str_mount[STRING_MOUNT_BUFFER_SIZE];

	lastsysticks = systicks;
	print_message = true;
redohere:
	while(iter < str_max_size)
	{
		//wait until next char is available
		while (!con_available_get_char())
		{
			if( ((systicks - lastsysticks) % FREQ_INT_SYSTICK) == 0 )
			{
				sign = (MAX_TIMEOUT2RX_INTEL_HEX - (systicks - lastsysticks)) / FREQ_INT_SYSTICK;
				if(print_message && sign < (MAX_TIMEOUT2RX_INTEL_HEX / FREQ_INT_SYSTICK))
				{
					con_send_string((uint8_t*)"\rTimeout to start to receive Intel Hex in: ");
					conv_uint32_to_dec((uint32_t)sign, str_mount);
					con_send_string(str_mount);
					con_send_string((uint8_t*)"s \r");
					print_message = false;
				}
			}
			else
			{
				print_message = true;
			}
			//Check timeout
			if( (systicks - lastsysticks) > MAX_TIMEOUT2RX_INTEL_HEX )
			{
				//User messages
				con_send_string((uint8_t*)"\r\n\nTimeout to start to receive Intel Hex is reached=>\r\n- Reset requested by the system.\r\n");
				reset_requested();
			}
		}
		lastsysticks = systicks;
		sign = con_get_char();

#if USART_ECHO_EN == 1
		if (sign == 3)
		{
			con_send_string((uint8_t*)"^C");
		}
		else if (sign != '\r')  //if sign == '\r' do nothing
		{
			if (sign == '\n')
			{
				con_send_string((uint8_t*)"\r\n");
			}
			else
			{
				str_mount[0] = sign;
				str_mount[1] = 0;
				con_send_string(str_mount);
			}
		}
#endif

		if(sign == 3)
			abort_intelhex_reception = true;
		else
		{
			//only accepts char between '0' ~ ':', 'A' ~ 'F', 'a' ~ 'f'
			if( ( ((sign > ('/')) && (sign < (':'+1))) ||
					(((sign&0x5F) > ('A'-1)) && ((sign&0x5F) < ('F'+1))))	)
				//assembling the record
				ser_inp_line[iter++] = sign;
			else if( (iter > 10) && ((sign == '\n') || (sign == '\r')) )	//	'\r' or '\n'
			{	//End of record. Close the ASCIIZ.
				ser_inp_line[iter++] = 0;
				break;
			}
			else
			{	
				//The minimum valid record is 11 characters long, so redo.
				iter = 0;
				goto redohere;
			}
		}
	} //while(iter < str_max_size)
	// Indicates that we received new line data, and it is going to be shiftingalidated.
	gpio_toggle(EMBEDDED_LED_PORT, EMBEDDED_LED_PIN);	//Toggle LED each received line
}


void usart_get_intel_hex(uint8_t *ser_inp_line, uint16_t str_max_size, uint8_t *ram_buffer)
{
	uint8_t intel_hex_localreg_data[STRING_MOUNT_BUFFER_SIZE / 2], intel_hex_numofdatabytes, intel_hex_type;
	uint16_t i, count_rx_intelhex_bytes, count_IHdata_record, intel_hex_address, first_data_address_intel_hex;
	uint16_t shifting;
	bool seek_first_intel_hex_address_for_data = true;
	uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];
	
	intel_hex_numofdatabytes = 0; //Force init of uint8_t here
	intel_hex_type = 0; //Force init of uint8_t here
	count_rx_intelhex_bytes = 0; //Force init of uint16_t here
	count_IHdata_record = 0;
	intel_hex_address = 0; //Force init of uint16_t here
	first_data_address_intel_hex = 0; //Force init of uint16_t here
	error_intel_hex = false;
	seek_first_intel_hex_address_for_data = true;

	//Init RAM Buffer
	for(i=0 ; i < (DATABASE_SIZE) ; i++)
	{
		ram_buffer[i] = (uint8_t)0xFF;
	}

	while (intel_hex_type != 1) //Run until intel_hex_type == 1: means end-of-file record
	{
		usart_get_string_line(ser_inp_line, str_max_size);
		if (!abort_intelhex_reception)
		{
			//Is it a valid Intel Hex record?
			if (validate_intel_hex_record(ser_inp_line, &intel_hex_numofdatabytes, &intel_hex_type,
					&intel_hex_address, &intel_hex_localreg_data[0]))
			{
				//":llaaaatt[dd...]cc"	tt is the field that represents the HEX record type, which may be one of the following:
				//											00 - data record
				//											01 - end-of-file record
				//											02 - extended segment address record
				//											04 - extended linear address record
				//											05 - start linear address record (MDK-ARM only)

				switch(intel_hex_type)
				{
					case 0:  //00 - data record
					{
						count_IHdata_record ++;
						if (seek_first_intel_hex_address_for_data)
						{
							first_data_address_intel_hex = intel_hex_address;
							seek_first_intel_hex_address_for_data = false;
						}
						//Now fill in RAM Buffer with data record
						for (i = 0; i < intel_hex_numofdatabytes; i++)
						{
							shifting = ((uint16_t)intel_hex_address - (uint16_t)first_data_address_intel_hex);
							ram_buffer[shifting + i] = intel_hex_localreg_data[i];
							count_rx_intelhex_bytes++;
						}
						break;
					}	//case 0:  //00 - data record
					case 1: //01 - end-of-file record
					{
						//Information to user
						con_send_string((uint8_t*)"\r\n\nReceive concluded. It has been received ");
						conv_uint32_to_dec((uint32_t)(count_IHdata_record), (uint8_t*)&(str_mount));
						con_send_string((uint8_t*)str_mount);
						con_send_string((uint8_t*)" IntelHex data records,\r\nwith an amount of ");
						conv_uint32_to_dec((uint32_t)(count_rx_intelhex_bytes), (uint8_t*)&(str_mount));
						con_send_string((uint8_t*)str_mount);
						con_send_string((uint8_t*)" data bytes.");
						/*// Wait user knowledge
						while(con_available_get_char())
							bin = con_get_char();
						con_send_string((uint8_t*)" data bytes.\r\nPress any key to conclude...");
						while(!con_available_get_char())
							__asm("nop");
						bin = con_get_char();
						bin &= 0xFF;	//Only to avoid "warning: variable 'bin' set but not used"*/
						return;
						break;
					}
					case 2:  //02 - extended segment address record
					{
						__asm("nop"); //not useful in this code
						break;
					}
					case 4: //04 - extended linear address record
					{
						if (intel_hex_numofdatabytes != 2) //intel_hex_numofdatabytes must be 2;
						{
							error_intel_hex = true;
						}
						if (intel_hex_localreg_data[0] != 0x08) //First data must be 0x08
						{
							error_intel_hex = true;
						}
						break;
					} //case 4: //04 - extended linear address record
					case 5:  //05 - start linear address record (MDK-ARM only)
					{
						__asm("nop"); //not useful in this code
						break;
					}
				}
			}
			else	//if (validate_intel_hex_record(ser_inp_line, intel_hex_numofdatabytes, intel_hex_type, intel_hex_address, intel_hex_localreg_data))
			{
				//Invalid record received
				con_send_string((uint8_t*)" Error: Invalid");
				error_intel_hex = true;
			}
		} //if (validate_intel_hex_record(ser_inp_line, &intel_hex_numofdatabytes, &intel_hex_type, &intel_hex_address, &intel_hex_localreg_data[0]))
	} //while (intel_hex_type != 0) //Run until case 1: means end-of-file record
}


bool validate_intel_hex_record(uint8_t *ser_inp_line, uint8_t *intel_hex_numofdatabytes,
		 uint8_t *intel_hex_type, uint16_t *intel_hex_address, uint8_t *intel_hex_localreg_data)
{
	uint8_t checksum_read, checksum_calc;
	int16_t i;

	//Record Intel Hex ":llaaaatt[dd...]cc"

	//":llaaaatt[dd...]cc" : is the colon that starts every Intel HEX record
	if (ser_inp_line[0] != ':')
	{
		//Error: Not started with ":"
		error_intel_hex = true;
		return false;
	}
	
	//":llaaaatt[dd...]cc" ll is the record-length field that represents the number of data bytes (dd) in the record
	*intel_hex_numofdatabytes = conv_2a_hex_to_uint8(ser_inp_line, 1);

	//":llaaaatt[dd...]cc" aaaa is the address field that represents the starting address for subsequent data in the record
	*intel_hex_address = (conv_2a_hex_to_uint8(ser_inp_line, 3) << 8) + (conv_2a_hex_to_uint8(ser_inp_line, 5));
	
	//":llaaaatt[dd...]cc"	tt is the field that represents the HEX record type, which may be one of the following:
	//											00 - data record
	//											01 - end-of-file record
	//											02 - extended segment address record
	//											04 - extended linear address record
	//											05 - start linear address record (MDK-ARM only)
	*intel_hex_type = conv_2a_hex_to_uint8(ser_inp_line, 7);
	
	//":llaaaatt[dd...]cc"	dd is a data field that represents one byte of data. A record may have multiple data bytes. The number of data bytes in the record must match the number specified by the ll field
	if (*intel_hex_numofdatabytes > 0)
	{
		for(i = 0; i < *intel_hex_numofdatabytes; i ++) //2 ASCII bytes per each byte of data in the record
		{
			intel_hex_localreg_data[i] = conv_2a_hex_to_uint8(ser_inp_line, 2 * i + 9);
		}
	}

	//":llaaaatt[dd...]cc"	cc is the checksum field that represents the checksum of the record. The checksum is calculated by summing the values of all hexadecimal digit pairs in the record modulo 256 and taking the two's complement
	checksum_read = conv_2a_hex_to_uint8(ser_inp_line, 2 * (*intel_hex_numofdatabytes) + 9);
	//Now compute Checksum
	checksum_calc = *intel_hex_numofdatabytes; // Checksum init
	for(i = 0; i < *intel_hex_numofdatabytes; i++)
		checksum_calc += intel_hex_localreg_data[i];
	checksum_calc += *intel_hex_type;
	checksum_calc += (uint8_t)((*intel_hex_address & 0xFF00) >> 8) + (uint8_t)(*intel_hex_address & 0x00FF);
	checksum_calc += checksum_read;
	//And check the computed Checksum
	if (checksum_calc != 0)
		error_intel_hex = true;
	return !error_intel_hex;
}
