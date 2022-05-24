/*
 * This file is part of the PS/2 Keyboard Adapter for MSX, using libopencm3 project.
 *
 * Copyright (C) 2021 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * This original SW is compiled to a Sharp/Epcom MSX HB-8000 and a brazilian ABNT2 PS/2 keyboard (ID=275)
 * But it is possible to update the table sending a Intel Hex File through serial or USB
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

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>

#include "system.h"
#if MCU == STM32F103C8		//Yes: All code!

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "dbasemgtF1.h"
#include "database.c"
#include "serial.h"


//Processor related sizes and adress:
#define USART_ECHO_EN 						1	//All chars received in Intel Hex are echoed in serial routines
#define STRING_MOUNT_BUFFER_SIZE	128
#define MAX_ERASE_TRIES						3
#define FLASH_WRONG_DATA_WRITTEN 	0x80
#define RESULT_OK 								0


//Prototype area:
//Usart operations
void usart_get_string_line(uint8_t*, uint16_t);
//Intel Hex operations
void get_valid_intelhex_file(uint8_t*, uint16_t, uint8_t*);
void usart_get_intel_hex(uint8_t*, uint16_t, uint8_t*);
bool validate_intel_hex_record(uint8_t*, uint8_t*, uint8_t*, uint16_t*, uint8_t*);
//flash operations
uint32_t flash_program_data(void);
void check_flash_error(void);
void check_flash_locked(void);
bool check_page_erased(uint32_t, bool*, uint16_t*);

//Global var area:
bool error_intel_hex;
bool abort_intelhex_reception;
bool compatible_database;
extern uint32_t *base_of_database;						//Declared on msxmap.cpp
extern uint8_t UNUSED_DATABASE[(uint8_t)10];	//Declared on msxmap.cpp
extern volatile bool update_ps2_leds;					//Declared on msxmap.cpp
extern uint8_t y_dummy;												//Declared on msxmap.cpp
extern bool ps2numlockstate;									//Declared on ps2handl.c
extern bool enable_xon_xoff;									//Declared on serial.c
uint8_t flash_buffer_ram[DATABASE_SIZE]; 			//Local variable, in aim to not consume resources in the main functional machine
bool flash_locked;


void database_setup(void)
{
	uint8_t ch, str_mount[STRING_MOUNT_BUFFER_SIZE];
	uint32_t iter;
	volatile uint32_t*base_of_database32;
	volatile uint8_t*base_of_database8;
	void *void_ptr;
	bool sector_erased = true;
	uint16_t attempts_erasing_page, qty_in_buffer;

	compatible_database = true;
	flash_locked = true;
	
	//Any character received via serial during the time of waiting PS/2 BAT enables Cleanup Database Flash
	if (console_available_get_char())
	{
		//Clear Serial RX Buffer
		while(console_available_get_char())
			ch = console_get_char(&qty_in_buffer);
		console_send_string((uint8_t*)"\r\nCleanup Database Flash. Send ""&"" to proceed or any other key to abort ");
		//Read a key
		while (!console_available_get_char())
		{
			if( ((systicks - lastsysticks) % FREQ_INT_SYSTICK) == 0 )
			{
				ch = MAX_TIMEOUT2AMPERSAND - (systicks - lastsysticks) / FREQ_INT_SYSTICK;
				if(print_message && ch < MAX_TIMEOUT2AMPERSAND)
				{
					console_send_string((uint8_t*)"Timeout to answer: ");
					conv_uint32_to_dec((uint32_t)ch, str_mount);
					console_send_string(str_mount);
					console_send_string((uint8_t*)"s \r");
					print_message = false;
				}
			}
			else
			{
				print_message = true;
			}
			//Check timeout
			if( (systicks - lastsysticks) > (MAX_TIMEOUT2AMPERSAND * FREQ_INT_SYSTICK) )
			{
				//User messages
				console_send_string((uint8_t*)"\r\n\nTimeout to answer: Proceeding without Reset the Database.\r\n");
				ring_put_ch(&console_rx_ring, ' ');
			}
		}
		ch = console_get_char(&qty_in_buffer);
		console_put_char(ch);
		if(ch == '&')
		{
			//Information to user
			console_send_string((uint8_t*)"\r\nErasing flash memory...\r\nBase Address  Size  Status\r\n");
			for(uint16_t sect_num = DATABASE_BASE_PAGE; sect_num < (DATABASE_TOP_PAGE+1); sect_num++)
			{
				//0x8005800 (Erase pages from 22 to 31)
				attempts_erasing_page = 0;
				while (attempts_erasing_page < MAX_ERASE_TRIES) //3 tries
				{
					//Information to user
					void_ptr = &str_mount;
					conv_uint32_to_8a_hex(((uint32_t)(sect_num*FLASH_PAGE_SIZE)), void_ptr);
					console_send_string((uint8_t*)" 0x");
					console_send_string((uint8_t*)str_mount);
					console_send_string((uint8_t*)"   ");
					conv_uint32_to_dec((uint32_t)FLASH_PAGE_SIZE, void_ptr);
					console_send_string((uint8_t*)str_mount);
					console_send_string((uint8_t*)"  ");
					//Cleaning is needed only if it is not erased. First check
					sector_erased = true;
					uint8_t *page_fl = (uint8_t *)(sect_num * FLASH_PAGE_SIZE);
					for (iter = 0; iter < (FLASH_PAGE_SIZE); iter++)	//All page bytes must be checked
					{
						if(	(*(volatile uint8_t *)(page_fl + iter)) != 0xFF )
						{
							sector_erased = false;
							break;	//quit "for (uint16_t iter = 0; iter < (FLASH_PAGE_SIZE / sizeof(uint32_t)); iter++)"
						}	//if(	(*(uint32_t *)(DATABASE_BASE_ADD + iter)) != 0xFFFFFFFF )
					} //for (iter = 0; iter < (DATABASE_TOP_ADDR + 1 - DATABASE_BASE_ADD); iter+=4)
					if (!sector_erased)	//Cleaning is needed only if it is not erased. Now do the erase!
					{
						serial_wait_tx_ends();
						//Erasing page
						check_flash_locked();
						check_flash_error();
						flash_erase_page((uint32_t)(sect_num * FLASH_PAGE_SIZE));
						flash_wait_for_last_operation();
						check_flash_error();
					}
					//Now confirm cleaning
					if(check_page_erased((uint32_t)(sect_num*FLASH_PAGE_SIZE), &sector_erased, &attempts_erasing_page))
					{
						base_of_database = (uint32_t*)((uint32_t)(sect_num * FLASH_PAGE_SIZE));
						break;	// this break quits "while (attempts_erasing_page < 3) //MAX_ERASE_TRIES tries"
					}
					console_send_string((uint8_t*)"\r\n");
				}	//0x800C0000 (Sector 3) while (attempts_erasing_page < MAX_ERASE_TRIES) //3 tries
			} //for(uint16_t sect_num = 22; sect_num < 32; sect_num++)
		}	//if(ch == '&')
	}	//if (!console_available_get_char())

	// if INITIAL_DATABASE is unprogrammed
	bool empty_database = true;
	base_of_database32 = (uint32_t *)INITIAL_DATABASE;
	for(iter = 0; iter < (DATABASE_SIZE / 4); iter++)
	{
		if(*(base_of_database32 + iter) != 0xFFFFFFFF)
		{
			empty_database = false;
			break;	//quit: for(iter = 0; iter < DATABASE_SIZE; iter += 4)
		}
	}

	//If it is unprogrammed, fill that area with the contents of DEFAULT_MSX_KEYB_DATABASE_CONVERSION
	if (empty_database)
	{
		//Database is empty. First copy DEFAULT_MSX_KEYB_DATABASE_CONVERSION to RAM
		base_of_database8 = (uint8_t*)&DEFAULT_MSX_KEYB_DATABASE_CONVERSION[0][0];
		for(iter = 0; iter < DATABASE_SIZE; iter ++)
		{
				ch = *(base_of_database8 + iter);
				flash_buffer_ram[iter] = ch;
		}

		//Then put it on INITIAL_DATABASE place => programming flash memory
		base_of_database = (uint32_t*)INITIAL_DATABASE;
		//Information to user
		console_send_string((uint8_t*)"\r\n\nInitializing Default Database on flash memory...");
		console_send_string((uint8_t*)"\r\n\nBase Address  Size\r\n");
		uintptr_t base_of_database_num = (uintptr_t)base_of_database;	//Convert from pointer to integer
		void_ptr = &str_mount;
		conv_uint32_to_8a_hex(((uint32_t)base_of_database_num), void_ptr);
		console_send_string((uint8_t*)" 0x");
		console_send_string((uint8_t*)str_mount);
		console_send_string((uint8_t*)"    ");
		conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
		console_send_string((uint8_t*)str_mount);
		serial_wait_tx_ends();
		check_flash_locked();
		check_flash_error();

		//programming flash memory
		//flash_program((uint32_t)base_of_database_num, flash_buffer_ram, DATABASE_SIZE);
		for(iter = 0; iter < DATABASE_SIZE; iter += 4)
		{
			//programming word data
			flash_program_word(base_of_database_num+iter, *((uint32_t*)(flash_buffer_ram + iter)));
			flash_wait_for_last_operation();
			uint32_t flash_status = flash_get_status_flags();
			if(flash_status != FLASH_SR_EOP)
				return;
		}
		check_flash_error();

		console_send_string((uint8_t*)"\r\n\nVerification of written data:\r\n");
		//verify if correct data was written
		for (iter = 0; iter < DATABASE_SIZE; iter+=4)	//DATABASE_SIZE must be checked
		{
			uint32_t iter_div4 = iter / sizeof(uint32_t);// Pointer of uint32_t has a step of 4 bytes
			if(	*(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
			{
				console_send_string((uint8_t*)"Wrong data written into flash memory\r\n");
				console_send_string((uint8_t*)"Dest add => 0x");
				void_ptr = &str_mount;
				conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)", found = 0x");
				conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)". Source add => 0x");
				conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)", was = 0x");
				conv_uint32_to_8a_hex(*((uint32_t*)(flash_buffer_ram + iter)), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)"\r\nLocked!");
				return;
			}
		} //for (iter = 0; iter < DATABASE_SIZE; iter+=4)	//3K must be checked
	}	//if (empty_database)

	//Searching a valid (useful) Database. Unused databases are marked as UNUSED_DATABASE
	volatile uint32_t displacement = 0;
	base_of_database8 = (uint8_t*)((uint32_t)INITIAL_DATABASE - displacement);

	while(((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)DATABASE_BASE_ADD)
	{
		//Searching a valid (useful) Database. Unused databases are marked as UNUSED_DATABASE
		if(
		*(base_of_database8+0) == UNUSED_DATABASE[0] &&
		*(base_of_database8+1) == UNUSED_DATABASE[1] &&
		*(base_of_database8+2) == UNUSED_DATABASE[2] &&
		*(base_of_database8+3) == UNUSED_DATABASE[3] &&
		*(base_of_database8+4) == UNUSED_DATABASE[4] &&
		*(base_of_database8+5) == UNUSED_DATABASE[5] &&
		*(base_of_database8+6) == UNUSED_DATABASE[6] &&
		*(base_of_database8+7) == UNUSED_DATABASE[7] &&
		*(base_of_database8+8) == UNUSED_DATABASE[8] &&
		*(base_of_database8+9) == UNUSED_DATABASE[9] )
		{
			displacement += DATABASE_SIZE;
			base_of_database8 = (uint8_t*)((uint32_t)INITIAL_DATABASE - displacement);
		}
		else
			break;	//if it is here, it found a valid Database
	}

	//Check Database consistensy (CheckSum & BCC of the first 319 blocks of 10 bytes each)
	uint8_t CheckSum = 0, bcc = 0;	//bcc is a vertical parity
	for (iter = 0; iter < (DATABASE_SIZE - 10); iter ++)
	{
		CheckSum += *(base_of_database8 + iter);
		bcc ^= *(base_of_database8 + iter);
	}	//for (iter = 0; iter < (DATABASE_SIZE - 10); iter ++)
	if( ((*(base_of_database8 + DATABASE_SIZE) + CheckSum) != 0)	||
			((*(base_of_database8 + (DATABASE_SIZE - 1))) != bcc)			||
			 (*(base_of_database8 + 0) != 1) 													||
			 (*(base_of_database8 + 1) != 0) )
	{
		compatible_database = false;
		console_send_string((uint8_t*)"\r\n\n!!!Attention!!! => No valid Database found. Please update it!\r\n\n");
	}
	else
	{
		y_dummy 				=  *(base_of_database8 + 3) & 0x0F; //Low nibble (no keys at this column)
		ps2numlockstate = (*(base_of_database8 + 3) & 0x10) != 0; //Bit 4
		enable_xon_xoff = (*(base_of_database8 + 3) & 0x20) != 0; //Bit 5
		update_ps2_leds = true;
		base_of_database = (uint32_t*)base_of_database8;
	}
	flash_lock();
	flash_locked = true;
}


int flashF1_rw(void)	//was main. It is int to allow simulate as a single module
{
	uint32_t result = 0;
	uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];
	//uint8_t flash_buffer_ram[DATABASE_SIZE]; //Local variable, in aim to not consume resources in the main functional machine
	//void *ptr_flash_buffer_ram;

	console_send_string((uint8_t*)"To update the Database, please send the new file in Intel Hex format!");
	console_send_string((uint8_t*)"\r\nOr turn off now.");
	get_valid_intelhex_file(&str_mount[0], STRING_MOUNT_BUFFER_SIZE, &flash_buffer_ram[0]);
	
	//ptr_flash_buffer_ram = flash_buffer_ram;
	serial_wait_tx_ends();
	//result = flash_program_data(ptr_flash_buffer_ram);
	result = flash_program_data();

	switch(result)
	{
	case RESULT_OK: //everything ok
		break;
	case FLASH_WRONG_DATA_WRITTEN: //data read from Flash is different than written data
		break;

	default: //wrong flags' values in Flash Status Register (FLASH_SR)
		console_send_string((uint8_t*)"\r\nWrong value of FLASH_SR: ");
		conv_uint32_to_8a_hex(result, str_mount);
		console_send_string(&str_mount[0]);
		break;
	}
	//send end_of_line
	console_send_string((uint8_t*)"\r\n");
	return result;
}	//int flashF4_rw(void)	//was main. It is int to allow simulate as a single module



bool check_page_erased(uint32_t page_add, bool *sector_erased, uint16_t *attempts_erasing_page)
{
	//Now confirm cleaning
	*sector_erased = true;
	for (uint32_t iter = 0; iter < (FLASH_PAGE_SIZE / sizeof(uint32_t)); iter++)	//All page bytes must be checked
	{
		if(	(*(volatile uint32_t *)(page_add + iter)) != 0xFFFFFFFF )
		{
			(*attempts_erasing_page)++;
			*sector_erased = false;
			//Information to user - continued
			if(*attempts_erasing_page == 1)
				console_send_string((uint8_t*)"Not OK at first attempt");
			if(*attempts_erasing_page == 2)
				console_send_string((uint8_t*)"Not OK at second attempt");
			if(*attempts_erasing_page == MAX_ERASE_TRIES)
				console_send_string((uint8_t*)"Not OK at third attempt\r\n");
			break;	//quit "for (iter = 0; iter < (DATABASE_TOP_ADDR + 1 - DATABASE_BASE_ADD); iter += 4)"
		}	//if(	(*(uint32_t *)(DATABASE_BASE_ADD + iter)) != 0xFFFFFFFF )
	} //for (iter = 0; iter < (DATABASE_TOP_ADDR + 1 - DATABASE_BASE_ADD); iter+=4)
	if(*sector_erased)
	{
		//Information to user - continued
		console_send_string((uint8_t*)"   Done\r\n");
		//Flash Database zone cleared. Points to default (Initial) Database address
		//base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE);
	}	//if(!attempts_erasing_page)
return *sector_erased;
}


void check_flash_locked(void)
{
	//Unlock FLASH_if it is locked
	if(flash_locked)
	{
		flash_unlock();
		flash_locked = false;
		flash_wait_for_last_operation();
	}
}


void check_flash_error(void)
{
	//Read FLASH_SR (Flash status register), searching for errors
	if(FLASH_SR & (1 << 14))	//RDERR (1 << 14): Read Protection Error (pcrop)
	{
		console_send_string((uint8_t*)"RDERR: Read Protection Error (pcrop)\r\n");
		FLASH_SR |= (1 << 14);	//Cleared by writing 1.
	}
	if(FLASH_SR & FLASH_SR_PGERR)	//PGSERR: Programming sequence error
	{
		console_send_string((uint8_t*)"PGSERR: Programming sequence error\r\n");
		FLASH_SR |= FLASH_SR_PGERR;	//Cleared by writing 1.
	}
	/*if(FLASH_SR & FLASH_SR_PGAERR)	//PGAERR: Programming alignment error
	{
		console_send_string((uint8_t*)"PGAERR: Programming alignment error\r\n");
		FLASH_SR |= FLASH_SR_PGAERR;	//Cleared by writing 1.
	}*/
	if(FLASH_SR & FLASH_SR_WRPRTERR)//WRPERR: Write protection error
	{
		console_send_string((uint8_t*)"FLASH_SR_WRPRTERR: Write protection error\r\n");
		FLASH_SR |= FLASH_SR_WRPRTERR;	//Cleared by writing 1.
	}
	/*if(FLASH_SR & FLASH_SR_OPERR)	//OPERR: Operation error. This bit is set only if error interrupts are enabled (ERRIE = 1).
	{
		console_send_string((uint8_t*)"OPERR: Operation error\r\n");
		FLASH_SR |= FLASH_SR_OPERR;	//Cleared by writing 1.
	}*/
	serial_wait_tx_ends();
}


void usart_get_string_line(uint8_t *ser_inp_line, uint16_t str_max_size)
{ //Reads until CR and returns an ASCIIZ on *ser_inp_line
	uint8_t u8char = 0;
	uint16_t qty_in_buffer, iter = 0;

redohere:
	while(iter < str_max_size)
	{
		//wait until next char is available
		while (!console_available_get_char())
		{
			if( ((systicks - lastsysticks) % FREQ_INT_SYSTICK) == 0 )
			{
				u8char = MAX_TIMEOUT2RX_INTEL_HEX - (systicks - lastsysticks) / FREQ_INT_SYSTICK;
				if(print_message && u8char < MAX_TIMEOUT2RX_INTEL_HEX)
				{
					console_send_string((uint8_t*)"Timeout to start to receive Intel Hex in: ");
					conv_uint32_to_dec((uint32_t)u8char, str_mount);
					console_send_string(str_mount);
					console_send_string((uint8_t*)"s \r");
					print_message = false;
				}
			}
			else
			{
				print_message = true;
			}
			//Check timeout
			if( (systicks - lastsysticks) > (MAX_TIMEOUT2RX_INTEL_HEX * FREQ_INT_SYSTICK) )
			{
				//User messages
				console_send_string((uint8_t*)"\r\n\nTimeout to start to receive Intel Hex is reached: Reset requested by the system.\r\n");
				reset_requested();
			}
		}
		u8char = console_get_char(&qty_in_buffer);

#if USART_ECHO_EN == 1
		if (u8char == 3)
		{
			while (!console_put_char('^'))
				__asm("nop");
			while (!console_put_char('C'))
				__asm("nop");
		}
		else if (u8char != '\r')  //if u8char == '\r' do nothing
		{
			if (u8char == '\n')
			{
				while (!console_put_char('\r'))
					__asm("nop");
				while (!console_put_char('\n'))
					__asm("nop");
			}
			else
				while (!console_put_char(u8char))
					__asm("nop");
		}
#endif

		if(u8char == 3)
			abort_intelhex_reception = true;
		else
		{
			//only accepts char between '0' ~ ':', 'A' ~ 'F', 'a' ~ 'f'
			if( ( ((u8char > ('/')) && (u8char < (':'+1))) ||
					(((u8char&0x5F) > ('A'-1)) && ((u8char&0x5F) < ('G'))))	)
				//assembling the record
				ser_inp_line[iter++] = u8char;
			else if( (iter > 10) && ((u8char == '\n') || (u8char == '\r')) )	//	'\r' or '\n'
			{	//End of record. Close the ASCIIZ.
				ser_inp_line[iter++] = 0;
				break;
			}
			else
			{	
				//The minimum valid record is 11 characters, so redo.
				iter = 0;
				goto redohere;
			}
		}
	} //while(iter < str_max_size)
	// Indicates that we received new line data, and it is going to be shiftingalidated.
	gpio_toggle(GPIOC, GPIO13);	//Toggle LED each received line
}


void get_valid_intelhex_file(uint8_t *ser_inp_line, uint16_t str_max_size, uint8_t *ram_buffer)
{
	error_intel_hex = true;
	abort_intelhex_reception = false;
	while (error_intel_hex == true)
	{
		usart_get_intel_hex(ser_inp_line, str_max_size, ram_buffer);
		if (error_intel_hex == true)
		{
			console_send_string((uint8_t*)"\r\n\n\n\nERROR in Intel Hex. ERROR\r\n\nPlease resend the Intel Hex...");
		}
		else if (abort_intelhex_reception)
		{
			console_send_string((uint8_t*)"\r\n\n\n\n!!!!!INTERRUPTED!!!!!\r\n\nPlease resend the Intel Hex...");
		}
		else
			return;
	}
}


void usart_get_intel_hex(uint8_t *ser_inp_line, uint16_t str_max_size, uint8_t *ram_buffer)
{
	uint8_t intel_hex_localreg_data[STRING_MOUNT_BUFFER_SIZE / 2], intel_hex_numofdatabytes, intel_hex_type;
	uint16_t i, count_rx_intelhex_bytes, intel_hex_address, first_data_address_intel_hex;
	uint16_t shifting;
	bool seek_first_intel_hex_address_for_data = true;
	
	intel_hex_numofdatabytes = 0; //Force init of uint8_t here
	intel_hex_type = 0; //Force init of uint8_t here
	count_rx_intelhex_bytes = 0; //Force init of uint16_t here
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
				console_send_string((uint8_t*)" Error: Invalid");
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


uint32_t flash_program_data(void)
{
	uint32_t iter;
	uint32_t displacement;
	bool DBaseSizeErasedPlaceFound = false, sector_erased = true;
	uint8_t str_mount[20];

	//verify:
	//1) If there is DATABASE_SIZE room in sector DATABASE_BASE_PAGEBER to acomodate a new Database image
	displacement = 0;
	base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE);
	//Information to user
	console_send_string((uint8_t*)"\r\n\nSearching for an empty 3200 bytes in sector 3 of flash memory...\r\nBase Address  Size  Status\r\n");
	while( (((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)DATABASE_BASE_ADD) && !DBaseSizeErasedPlaceFound)
	{
		//Information to user
		void *void_ptr = &str_mount;
		conv_uint32_to_8a_hex(((uint32_t)INITIAL_DATABASE - displacement), void_ptr);
		console_send_string((uint8_t*)" 0x");
		console_send_string((uint8_t*)str_mount);
		console_send_string((uint8_t*)"    ");
		conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
		console_send_string((uint8_t*)str_mount);
		for (iter = 0; iter < DATABASE_SIZE; iter+=4)
		{
			//Searching a DATABASE_SIZE room in the address range of flash sector DATABASE_BASE_PAGEBER to acomodate a new Database image
			if(	*(base_of_database + iter) != 0xFFFFFFFF )
			{
				console_send_string((uint8_t*)"  Not available\r\n");
				DBaseSizeErasedPlaceFound = false;
				displacement += DATABASE_SIZE;
				base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE - displacement);
				break;	// this break quits 'for (iter; iter < DATABASE_SIZE; iter+=4)'
			}
			else //if(	*(base_of_database8 + iter) != 0xFFFFFFFF )
			{
				//DATABASE_SIZE page is free on "base_of_database"
				DBaseSizeErasedPlaceFound = true;
				console_send_string((uint8_t*)"  Ok!\r\n");
				break;	// this break quits 'for (iter; iter < DATABASE_SIZE; iter+=4)'
			} //if(	*(base_of_database8 + iter) != 0xFFFFFFFF )
		} //for (iter; iter < DATABASE_SIZE; iter+=4)
	}	//while( (((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)DATABASE_BASE_ADD) && !DBaseSizeErasedPlaceFound)

	//2) If there is no room in pages 22 to 31, then erase them
	if (!DBaseSizeErasedPlaceFound)
	{
		//DATABASE_SIZE bytes free room was not found: Perform erase of page 22 to 31
		uint16_t attempts_erasing_page = 0;
		//Information to user
		console_send_string((uint8_t*)"\r\nErasing flash memory...\r\nBase Address  Size  Status\r\n");
		for(uint16_t sect_num = DATABASE_BASE_PAGE; sect_num < (DATABASE_TOP_PAGE+1); sect_num++)
		{
			//0x8005800 (Erase pages from 22 to 31)
			while (attempts_erasing_page < MAX_ERASE_TRIES) //3 tries
			{
				//Information to user
				void *void_ptr = &str_mount;
				conv_uint32_to_8a_hex(((uint32_t)(sect_num*FLASH_PAGE_SIZE)), void_ptr);
				console_send_string((uint8_t*)" 0x");
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)"   ");
				conv_uint32_to_dec((uint32_t)FLASH_PAGE_SIZE, void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)"  ");
				//Cleaning is needed only if it is not erased. First check
				sector_erased = true;
				uint8_t *page_fl = (uint8_t *)(sect_num * FLASH_PAGE_SIZE);
				for (iter = 0; iter < (FLASH_PAGE_SIZE); iter++)	//All page bytes must be checked
				{
					if(	(*(volatile uint8_t *)(page_fl + iter)) != 0xFF )
					{
						sector_erased = false;
						break;	//quit "for (uint16_t iter = 0; iter < (FLASH_PAGE_SIZE / sizeof(uint32_t)); iter++)"
					}	//if(	(*(uint32_t *)(DATABASE_BASE_ADD + iter)) != 0xFFFFFFFF )
				} //for (iter = 0; iter < (DATABASE_TOP_ADDR + 1 - DATABASE_BASE_ADD); iter+=4)
				if (!sector_erased)	//Cleaning is needed only if it is not erased. Now do
				{
					serial_wait_tx_ends();
					//Erasing page
					check_flash_locked();
					check_flash_error();
					flash_erase_page((uint32_t)(sect_num * FLASH_PAGE_SIZE));
					flash_wait_for_last_operation();
					check_flash_error();
				}
				//Now confirm cleaning
				if(check_page_erased((uint32_t)(sect_num*FLASH_PAGE_SIZE), &sector_erased, &attempts_erasing_page))
				{
					base_of_database = (uint32_t*)((uint32_t)(sect_num * FLASH_PAGE_SIZE));
					break;	// this break quits "while (attempts_erasing_page < 3) //MAX_ERASE_TRIES tries"
				}
				console_send_string((uint8_t*)"\r\n");
			}	//0x800C0000 (Sector 3) while (attempts_erasing_page < MAX_ERASE_TRIES) //3 tries
		} //for(uint16_t sect_num = 22; sect_num < 32; sect_num++)
	}	//if (!DBaseSizeErasedPlaceFound)
	
	//Programming Flash Memory
	//Information to user
	console_send_string((uint8_t*)"\r\nProgramming flash memory...\r\nBase Address  Size  RAM Address\r\n");
	uintptr_t base_of_database_num = (uintptr_t)base_of_database;	//Convert from pointer to integer
	void *void_ptr = &str_mount;
	conv_uint32_to_8a_hex(((uint32_t)base_of_database_num), void_ptr);
	console_send_string((uint8_t*)" 0x");
	console_send_string((uint8_t*)str_mount);
	console_send_string((uint8_t*)"  ");
	conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
	console_send_string((uint8_t*)str_mount);
	console_send_string((uint8_t*)"  0x");
	conv_uint32_to_8a_hex(((uintptr_t)flash_buffer_ram), void_ptr);
	console_send_string((uint8_t*)str_mount);
	serial_wait_tx_ends();
	check_flash_locked();
	check_flash_error();
	//programming flash memory
	//flash_program((uint32_t)base_of_database_num, flash_buffer_ram, DATABASE_SIZE);
	for(iter = 0; iter < DATABASE_SIZE; iter += 4)
	{
		//programming word data
		flash_program_word(base_of_database_num+iter, *((uint32_t*)(flash_buffer_ram + iter)));
		flash_wait_for_last_operation();
		uint32_t flash_status = flash_get_status_flags();
		if(flash_status != FLASH_SR_EOP)
			return flash_status;
	}
	check_flash_error();

	console_send_string((uint8_t*)"\r\n\nVerification of written data:\r\n");
	//verify if correct data was written
	for (iter = 0; iter < (DATABASE_SIZE); iter += 4)	//All DATABASE_SIZE must be checked
	{
		uint32_t iter_div4 = iter / sizeof(uint32_t);// Pointer of uint32_t has a step of 4 bytes
		if(	*(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
		{
			if(*(base_of_database + iter) == 0xFFFFFFFF)
			{
				console_send_string((uint8_t*)"(RAM buffer address = 0x");
				conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)". Was = 0x");
				conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)", while flash address 0x");
				conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)" remains erased.\r\nLocked!");
				serial_wait_tx_ends();
			}
			else
			{
				console_send_string((uint8_t*)"Wrong data written into flash memory:\r\nDest add => 0x");
				void_ptr = &str_mount;
				conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)", found = 0x");
				conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)". Source add => 0x");
				conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)", was = 0x");
				conv_uint32_to_8a_hex(*((uint32_t*)(flash_buffer_ram + iter)), void_ptr);
				console_send_string((uint8_t*)str_mount);
				console_send_string((uint8_t*)"\r\nLocked!");
			}
			return FLASH_WRONG_DATA_WRITTEN;
		}	//if(	*(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
	} //for (iter = 0; iter < DATABASE_SIZE; iter+=4)	//3K must be checked

	//if flash sector DATABASE_BASE_PAGEBER was NOT just erased, there is no need to "invalidate" former Database,
	//as there is no former Database
	base_of_database_num = (uintptr_t)base_of_database;	//Convert from pointer to integer
	void_ptr = &str_mount;

	if (base_of_database_num != INITIAL_DATABASE)
	{
		console_send_string((uint8_t*)"\r\n\nNulling former Database:\r\n");
		base_of_database += DATABASE_SIZE;	//points to former Database
		base_of_database_num = (uintptr_t)base_of_database;	//Convert from pointer to integer
		console_send_string((uint8_t*)str_mount);
		//Make former Databse not useable anymore
		//First copy UNUSED_DATABASE to RAM
		uint16_t *base_of_database16 = (uint16_t*)&UNUSED_DATABASE;
		for(iter = 0; iter < (DB_NUM_COLS / sizeof(uint16_t)); iter ++)
		{
			uint16_t ch16 = *(base_of_database16 + iter);
			conv_uint32_to_8a_hex((uintptr_t)(base_of_database_num + iter), void_ptr);
			console_send_string((uint8_t*)str_mount);
			serial_wait_tx_ends();
			flash_program_half_word(base_of_database_num+iter, ch16);
			flash_wait_for_last_operation();
			uint32_t flash_status = flash_get_status_flags();
			if(flash_status != FLASH_SR_EOP)
				return flash_status;
			check_flash_error();
			//Now check written data
			if((*(uint16_t*)(base_of_database+iter)) == ch16)
			{
				console_send_string((uint8_t*)" <= Ok\r\n");
			}
			else	//if (*(uint16_t*)(base_of_database+iter) = ch16)
			{
				console_send_string((uint8_t*)" <= Wrong\r\n");
				flash_lock();
				flash_locked = true;
				return FLASH_WRONG_DATA_WRITTEN;
			}	//if (*(uint16_t*)(base_of_database+iter) = ch16)
		}	//for(iter = 0; iter < (DB_NUM_COLS / sizeof(uint16_t)); iter ++)
	}	//if (base_of_database_num != INITIAL_DATABASE)
	console_send_string((uint8_t*)"\r\nSuccessfully written database at 0x");
	console_send_string((uint8_t*)".\r\n\nNow, please TURN OFF to plug the PS/2 keyboard!");
	flash_lock();
	flash_locked = true;
	return RESULT_OK;
}	//uint32_t flash_program_data(void)

#endif	//#if MCU == STM32F103C8