/*
 * This file is part of the MSX Keyboard Subsystem Emulator project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
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

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "system.h"
#include "sys_timer.h"
#include "serial.h"
#include "hr_timer.h"
#include "msxmap.h"
#include "serial_no.h"
#if USE_USB == true
#include "cdcacm.h"
#include "version.h"
#endif	//#if USE_USB == true

//Global variables 
uint32_t scan_pointer;
uint32_t delay_to_read_x_scan;
uint16_t init_inactivity_cycles[SCAN_POINTER_SIZE];
extern uint32_t systicks;													//Declared on sys_timer.cpp
extern bool single_step, wait_flag, single_sweep;	//Declared on sys_timer.cpp
extern bool ticks_keys;														//Declared on sys_timer.cpp
extern uint8_t init_scancount, end_scancount;			//Declared on sys_timer.cpp
extern uint8_t y_scan;														//Declared on sys_timer.cpp
extern volatile uint8_t caps_line, kana_line;			//Declared on sys_timer.cpp
extern uint8_t serial_no[LEN_SERIAL_No + 1];			//Declared on serial_no.c
extern bool ok_to_rx;															//Declared on serial_no.c
extern int usb_configured;												//Declared on cdcacm.c
extern uint8_t inactivity_cycles[SCAN_POINTER_SIZE];//Declared on sys_timer.cpp
extern uint8_t mountISRstr[MNTSTR_SIZE];					//Declared on msxmap.cpp
extern struct s_pascal_string pascal_string;						//Declared on msxmap.cpp


//Scan speed selection
const uint8_t SPEED_SELEC[SCAN_POINTER_SIZE][8]= {"1.00000", "2.00000", "4.00000", "8.00000", "16.0000", "32.0000",
																									"64.0000", "128.000", "256.000", "512.000", "1023.95", "2048.13",
																									"4095.36", "8194.32", "16431.9", "32073.3", "60215.1", "119658."};


const uint8_t MAX_INACT_READ_CYLES[SCAN_POINTER_SIZE]= {0, 0, 0, 0, 0, 0, 0, 0, 0,
																												0, 0, 0, 2, 4, 8, 16, 30, 60};

const uint8_t TIME_TO_READ_X[DELAY_TO_READ_SIZE][5]= {"2.25", "2.40", "2.65", "2.90", "3.15", "3.40", "3.65",
																											"3.90", "4.15", "4.40", "4.65", "4.90", "5.15", "5.40"};


int main(void){
	//Query the reset cause
	uint32_t reset_org;
	reset_org = RCC_CSR;
	RCC_CSR   = RCC_CSR_RMVF;
	systicks 	= RCC_CSR;					 //Only to avoid warnings of unused RCC_CSR variable

#if MCU == STM32F103
	//Blue Pill
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	//WeAct miniF4 STM32F401CCU6 and up, V2.0 and up (Black Pill)
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
#endif	//#if MCU == STM32F401

#if MCU == STM32F103
	rcc_periph_clock_enable(RCC_AFIO); //Have to clock AFIO to use PA15 and PB4 freed by gpio remap below

	// Bits 9:8 TIM2_REMAP[1:0]: TIM2 remapping - 01: Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3)
	gpio_primary_remap(AFIO_MAPR_TIM2_REMAP_PARTIAL_REMAP1, 0);

	// Full Serial Wire JTAG capability without JNTRST
	//gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST, 0);

	// Disable JTAG, enable SWD. This frees up GPIO PA15 (JTDI), PB3 (JTDO / TRACESWO) and PB4 (NJTRST)
	// GPIO PA13 (SWDIO) and PA14 (SWCLK) are still used by STlink SWD.
	//I didn't be successful to get PB3 freeed, and I had to reengineered this new solution, like
	//do not use interrupts to CAPS and KANA LEDs, and, for example, I put them inside systicks task,
	//that I suppose it is a better solution.
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);
#endif	//#if MCU == STM32F103

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	//Debug & performance measurement
	msxmap object;
	object.general_debug_setup();

	//Get Serial number based on unique microcontroller factory masked number
	serialno_read(serial_no);

	serial_setup();

#if USE_USB == true
	cdcacm_init();
	//Time to USB be enumerated and recognized by the OS and terminal app
#if MCU == STM32F103
	for (uint32_t i = 0; i < 0x2000000; i++) __asm__("nop");
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	for (uint32_t i = 0; i < 0x4000000; i++) __asm__("nop");
#endif	//#if MCU == STM32F401
#endif	//#if USE_USB == true
	con_send_string((uint8_t*)"\r\n\n\r\nMSX keyboard subsystem emulator " FIRMWARE_VERSION);
	con_send_string((uint8_t*)"\r\nBased on hardware " HARDWARE_BASE "\r\nSerial number is ");
	con_send_string((uint8_t*)serial_no);
	con_send_string((uint8_t*)"\r\nFirmware built on ");
	con_send_string((uint8_t*)__DATE__);
	con_send_string((uint8_t*)" ");
	con_send_string((uint8_t*)__TIME__);
	con_send_string((uint8_t*)"\r\n\nThis boot was requested from ");

	// Initialize ring buffer for readings of DUT inside isr.
	pascal_string_init(&pascal_string, mountISRstr, MNTSTR_SIZE);

	if(reset_org & RCC_CSR_PINRSTF)
		string_append((uint8_t*)"NRST_pin", &pascal_string);
	if(reset_org & RCC_CSR_PORRSTF){
		if(pascal_string.str_len == 8)
			string_append((uint8_t*)" and PowerOn", &pascal_string);
		else
			string_append((uint8_t*)"Software", &pascal_string);
	}
	if(reset_org & RCC_CSR_SFTRSTF){
		if(pascal_string.str_len >= 8)
			string_append((uint8_t*)" and Software", &pascal_string);
		else
			string_append((uint8_t*)"Software", &pascal_string);
	}
	if(reset_org & RCC_CSR_IWDGRSTF){
		if(pascal_string.str_len >= 8)
			string_append((uint8_t*)" and IWDG", &pascal_string);
		else
			string_append((uint8_t*)"IWDG", &pascal_string);
	}
	if(reset_org & RCC_CSR_WWDGRSTF){
		if(pascal_string.str_len >= 8)
			string_append((uint8_t*)" and WWDG", &pascal_string);
		else
			string_append((uint8_t*)"WWDG", &pascal_string);
	}
	if(reset_org & RCC_CSR_LPWRRSTF){
		if(pascal_string.str_len >= 8)
			string_append((uint8_t*)" and Low-power", &pascal_string);
		else
			string_append((uint8_t*)"Low-power", &pascal_string);
	}

	con_send_string(pascal_string.data);
	//After send pascal_string.data to print, clear it.
	pascal_string.str_len = 0;
	pascal_string.data[0] = 0;

	con_send_string((uint8_t*)". Booting...");

	con_send_string((uint8_t*)"\r\n\r\nConfiguring:\r\n");

#if USE_USB == true
	if(usb_configured)
		con_send_string((uint8_t*)". USB has been enumerated => Console and UART are over USB.\r\n");
	else
		{
			con_send_string((uint8_t*)". USB host not found => Using Console over UART. Now USB is disabled\r\n");
			disable_usb();
		}
#else	//#if USE_USB == true
	con_send_string((uint8_t*)". Non USB version. Console is over UART.\r\n");
#endif	//#if USE_USB == true

	//User messages
	con_send_string((uint8_t*)"- 5V compatible pin ports and interrupts to interface like a real MSX;\n\r");

	object.msx_interface_setup();

	//User messages
	con_send_string((uint8_t*)"- High resolution timer2;\r\n");

	// Now configure High Resolution Timer for quarter micro second resolution Delay
	tim_hr_setup(TIM_HR);

	con_send_string((uint8_t*)"- SysTick;\r\n");

	systick_setup();
	
	con_send_string((uint8_t*)"- Ports config locking.\r\n");

	con_send_string((uint8_t*)"\r\nBoot complete! Press ? to show available options.\r\n");
	
	//These variables are updated inside the loop, from menu (through console).
	scan_pointer = INIT_SCAN_POINTER;									//Starts with 32KHz
	delay_to_read_x_scan = INIT_DELAY_TO_READ_X_SCAN;	//Starts with 3.65μs
	init_scancount = 0;
	end_scancount  = 8;			//8 for HB8000, 9 for Expert and 10 for full MSX keyboards
	caps_line = 0x0B;				//Starts with caps led blinking
	kana_line = 0x0B;				//Starts with Scroll led blinking
	wait_flag = false;			//Starts with running scan

	serial_rx_start();
	ok_to_rx = true;

	/*********************************************************************************************/
	/************************************** Main Loop ********************************************/
	/*********************************************************************************************/
	for(;;)
	{
		//The functionality running in main loop is to control serial communication and interact with user
		uint8_t mountstring[60];

		con_send_string((uint8_t*)"\r\n> ");	//put prompt
		while (!con_available_get_char())
		{
			//wait here until new char is available at serial port, but print the changes info of received keystroke
			if(pascal_string.str_len)
			{
				con_send_string(pascal_string.data);
				//After send string to console, clear it.
				pascal_string.str_len = 0;
				pascal_string.data[0] = 0;
			}
		}
		uint8_t ch = con_get_char();
		switch (ch)
		{
			case '?':
				con_send_string((uint8_t*)"(?) Available options\r\n1) General:\r\n");
				con_send_string((uint8_t*)"   r (Show Running config);\r\n");
				con_send_string((uint8_t*)"   c (Caps Lock line <- On/Off/Blink);\r\n");
				con_send_string((uint8_t*)"   k (Katakana line  <- On/Off/Blink);\r\n");
				con_send_string((uint8_t*)"2) Scan related:\r\n");
				con_send_string((uint8_t*)"   s (Scan submenu - Set first [Y Begin] and last [Y End] colunms to scan);\r\n");
				con_send_string((uint8_t*)"   + (Increase scan rate);\r\n");
				con_send_string((uint8_t*)"   - (Decrease scan rate);\r\n");
				con_send_string((uint8_t*)"   p (Toggle pause scan);\r\n");
				con_send_string((uint8_t*)"   n (Next step colunm scan)                        <= when scan is paused;\r\n");
				con_send_string((uint8_t*)"   Space (One shot scan, from [Y Begin] to [Y End]) <= when scan is paused;\r\n");
				con_send_string((uint8_t*)"3) Times / Delays / Duties:\r\n");
				con_send_string((uint8_t*)"   a) Time to read X_Scan (after Y_Scan) update:\r\n");
				con_send_string((uint8_t*)"   < (decrease by 0.25μs);\r\n");
				con_send_string((uint8_t*)"   > (increase by 0.25μs);\r\n");
				con_send_string((uint8_t*)"   b) Read duty cycle: 1 work N idle. N may be 0 to maximum for speed:\r\n");
				con_send_string((uint8_t*)"   i (After one sweep active read cycle, configure number of idle cycles).\r\n");
			break;	//case '?':

			case 'r': //else if (ch == 'r')
				//Show Running config. Only print.
				con_send_string((uint8_t*)"(r) Running config:\r\nScan rate: ");
				con_send_string((uint8_t*)&SPEED_SELEC[scan_pointer][0]);
				if(wait_flag)
				{
					con_send_string((uint8_t*)"Hz;\r\nScan is PAUSED;");
				}
				else
				{
					con_send_string((uint8_t*)"Hz;\r\nScan is RUNNING;");
				}
				con_send_string((uint8_t*)"\r\nScan begins [Y Begin] at 0x");
				conv_uint8_to_2a_hex(init_scancount, &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[1]);
				con_send_string((uint8_t*)" and ends [Y End] at 0x");
				conv_uint8_to_2a_hex(end_scancount, &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[1]);
				con_send_string((uint8_t*)";\r\nDelay to read X_Scan (after Y_Scan update): ");
				con_send_string((uint8_t*)&TIME_TO_READ_X[delay_to_read_x_scan][0]);
				con_send_string((uint8_t*)"μs\r\nCaps Line: Current value = ");
				if (caps_line == 0)
					con_send_string((uint8_t*)"0 (active)");
				else if (caps_line == 1)
					con_send_string((uint8_t*)"1 (off)");
				else if (caps_line == 0x0B)
					con_send_string((uint8_t*)"Blink");
				con_send_string((uint8_t*)";\r\nKatakana Line: Current value = ");
				if (kana_line == 0)
					con_send_string((uint8_t*)"0 (active);\r\n");
				else if (kana_line == 1)
					con_send_string((uint8_t*)"1 (off);\r\n");
				else if (kana_line == 0x0B)
					con_send_string((uint8_t*)"Blink;\r\n");
				con_send_string((uint8_t*)"Duty read cycle is 1 active to ");
				conv_uint32_to_dec((uint32_t)init_inactivity_cycles[scan_pointer], &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[0]);
				con_send_string((uint8_t*)" inactive.\r\n");
			break;	//case 'r':

			case 'c':
				//Caps Lock. Print current one.
				con_send_string((uint8_t*)"(c) Caps Line - Set to Active (0), Off (1) or Blink (B).\r\nCurrent value = ");
				conv_uint8_to_2a_hex(caps_line, &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[1]);
				con_send_string((uint8_t*)";\r\n");
				con_send_string((uint8_t*)"Enter 0, 1 or B to update: ");
				ch = 0xFF;
				while ( (ch != '0') && (ch != '1') && (ch != 'B') )
				{
					while (!con_available_get_char()) __asm("nop");	//wait here until new char is available at serial port
					ch = con_get_char();
					if (ch >= 'a')
						ch &= 0x5F; //To capital
					if (ch == 'B')
					{
						if(gpio_get(KANA_port, KANA_pin_id))
							gpio_clear(CAPS_port, CAPS_pin_id);
						else
							gpio_set(CAPS_port, CAPS_pin_id);
					}
				}
				mountstring[0] = '0';
				mountstring[1] = ch;
				mountstring[2] = '\0';
				con_send_string((uint8_t*)&mountstring[1]);
				con_send_string((uint8_t*)".\r\n");
				caps_line = conv_2a_hex_to_uint8(&mountstring[0], 0);
			break;	//case 'c':

			case 'k':	//else if (ch == 'k')
				//Kana Line (It controls PS/2 Scroll Lock).
				con_send_string((uint8_t*)"(k) Katakana Line - Set to Active (0), Off (1) or Blink (B)\r\nCurrent value = ");
				conv_uint8_to_2a_hex(kana_line, &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[1]);
				con_send_string((uint8_t*)";\r\n");
				con_send_string((uint8_t*)"Enter 0, 1 or B to update: ");
				ch = 0xFF;
				while (  (ch != '0') && (ch != '1') && (ch != 'B') )
				{
					while (!con_available_get_char()) __asm("nop");	//wait here until new char is available at serial port
					ch = con_get_char();
					if(ch > 'a')
						ch &= 0x5F; //To capital
					if (ch == 'B')
					{
						if(gpio_get(CAPS_port, CAPS_pin_id))
							gpio_clear(KANA_port, KANA_pin_id);
						else
							gpio_set(KANA_port, KANA_pin_id);
					}
				}
				mountstring[0] = '0';
				mountstring[1] = ch;
				mountstring[2] = '\0';
				con_send_string((uint8_t*)&mountstring[1]);
				con_send_string((uint8_t*)".\r\n");
				kana_line = conv_2a_hex_to_uint8(&mountstring[0], 0);
			break;	//case 'k':

			case 's':	//if (ch == 's')
				//Update Y Begin and End
				con_send_string((uint8_t*)"(s) Scan Sub menu:");
				con_send_string((uint8_t*)"\r\n         ^C or Enter now aborts;");
				con_send_string((uint8_t*)"\r\n         b ([Y Begin] - Update the value) Current one = 0x");
				conv_uint8_to_2a_hex(init_scancount, &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[1]);
				con_send_string((uint8_t*)";\r\n         e ([Y End] - Update the value) Current one = 0x");
				conv_uint8_to_2a_hex(end_scancount, &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[1]);
				con_send_string((uint8_t*)".\r\n>> ");
				while (!con_available_get_char()) __asm("nop");	//wait here until new char is available at serial port
				ch = con_get_char();
				if (ch != ('\r' || '\03'))
				{
					//Operation not aborted
					if (ch == 'b')
					{
						//Y Begin. Print current one: init_scancount
						con_send_string((uint8_t*)"Scan begins at colunm 0x");
						conv_uint8_to_2a_hex(init_scancount, &mountstring[0]);
						con_send_string((uint8_t*)&mountstring[1]);
						con_send_string((uint8_t*)". Enter 0-");
						conv_uint8_to_2a_hex(end_scancount, &mountstring[0]);
						con_send_string((uint8_t*)&mountstring[1]);
						con_send_string((uint8_t*)" to update: ");
						ch = 0xFF;
						mountstring[59] = 0x0F;
						while( (ch < '0') || ((ch > '9') && (ch < 'A')) || (ch > 'F') || (mountstring[59] > end_scancount) )
						{
							while (!con_available_get_char()) __asm("nop");	//wait here until new char is available at serial port
							ch = con_get_char();
							if(ch > 'a')
								ch &= 0x5F; //To capital
							mountstring[0] = '0';
							mountstring[1] = ch;
							mountstring[2] = '\0';
							mountstring[59] = conv_2a_hex_to_uint8(&mountstring[0], 0);
						}
						init_scancount = conv_2a_hex_to_uint8(&mountstring[0], 0);
						con_send_string(&mountstring[1]);
						con_send_string((uint8_t*)".\r\n");
					}
					else if (ch == 'e')
					{
						//Y End. Print current one: end_scancount
						con_send_string((uint8_t*)"Scan ends at colunm 0x");
						conv_uint8_to_2a_hex(end_scancount, &mountstring[0]);
						con_send_string((uint8_t*)&mountstring[1]);
						con_send_string((uint8_t*)". Enter ");
						conv_uint8_to_2a_hex(init_scancount, &mountstring[0]);
						con_send_string((uint8_t*)&mountstring[1]);
						con_send_string((uint8_t*)"-F to update: ");
						ch = 0xFF;
						mountstring[59] = 0;
						while( (ch < '0') || ((ch > '9') && (ch < 'A')) || (ch > 'F') || (mountstring[59] < init_scancount) )
						{
							while (!con_available_get_char()) __asm("nop");	//wait here until new char is available at serial port
							ch = con_get_char();
							if(ch > 'a')
								ch &= 0x5F; //To capital
							mountstring[0] = '0';
							mountstring[1] = ch;
							mountstring[2] = '\0';
							mountstring[59] = conv_2a_hex_to_uint8(mountstring, 0);
						}
						end_scancount = conv_2a_hex_to_uint8(&mountstring[0], 0);
						con_send_string(&mountstring[1]);
						con_send_string((uint8_t*)".\r\n");
					}	//else if (ch == 'e')
				}	//if (ch != ('\r' && '\03'))
				else
				{
					//Aborted
					con_send_string((uint8_t*)"\r\nOperation aborted!\r\n");
				}
			break;	//case 's':

			case '+':
				if(scan_pointer <= SCAN_POINTER_SIZE-2)
				{
					scan_pointer ++;
					systick_update(scan_pointer);	//t_sys_timer.cpp has a table with systick reload
					con_send_string((uint8_t*)"(+) New scan frequency applied: ");
					con_send_string((uint8_t*)&SPEED_SELEC[scan_pointer][0]);
					con_send_string((uint8_t*)"Hz.\r\n");
				}
				else
				{
					con_send_string((uint8_t*)"Maximum scan frequency unchanged: Already working: ");
					con_send_string((uint8_t*)&SPEED_SELEC[scan_pointer][0]);
					con_send_string((uint8_t*)"Hz.\r\n");
				}
			break;	//case '+':

			case '-':
				if(scan_pointer)
				{
					scan_pointer --;
					systick_update(scan_pointer);	//t_sys_timer.cpp has a table with systick reload
					con_send_string((uint8_t*)"(-) New scan frequency applied: ");
					con_send_string((uint8_t*)&SPEED_SELEC[scan_pointer][0]);
					con_send_string((uint8_t*)"Hz.\r\n");
				}
				else
				{
					con_send_string((uint8_t*)"Minimum scan frequency unchanged: Already workimg at ");
					con_send_string((uint8_t*)&SPEED_SELEC[scan_pointer][0]);
					con_send_string((uint8_t*)"Hz.\r\n");
				}
			break;	//case '-':

			case 'p':	//else if (ch == 'p')
				if(wait_flag ^= true)
				{
					con_send_string((uint8_t*)"(p) (Toggle pause scan): Scan is paused\r\n");
				}
				else
				{
					con_send_string((uint8_t*)"(p) (Toggle pause scan): Scan is running. Config:");
					con_send_string((uint8_t*)"\r\nScan rate: ");
					con_send_string((uint8_t*)&SPEED_SELEC[scan_pointer][0]);
					con_send_string((uint8_t*)"Hz;\r\nScan is beginning at 0x");
					conv_uint8_to_2a_hex(init_scancount, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[1]);
					con_send_string((uint8_t*)" [Y Begin] and ending at 0x");
					conv_uint8_to_2a_hex(end_scancount, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[1]);
					con_send_string((uint8_t*)" [Y End].\r\n");
				}
			break;	//case 'p':

			case ' ':
				if(wait_flag)
				{
					con_send_string((uint8_t*)"(" ") One shot scan with the configuration:");
					con_send_string((uint8_t*)"\r\nScan rate: ");
					con_send_string((uint8_t*)&SPEED_SELEC[scan_pointer][0]);
					con_send_string((uint8_t*)"Hz;\r\nScan will begin at 0x");
					conv_uint8_to_2a_hex(init_scancount, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[1]);
					con_send_string((uint8_t*)" [Y Begin] and will end at 0x");
					conv_uint8_to_2a_hex(end_scancount, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[1]);
					con_send_string((uint8_t*)" [Y End].\r\n");
					single_sweep = true;
					wait_flag = false;
					y_scan = init_scancount;
				}
			break;	//case ' '

			case 'n':
				if(wait_flag)
				{
					con_send_string((uint8_t*)"(n) Next step colunm scan. This one is 0x");
					conv_uint8_to_2a_hex(y_scan, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[1]);
					con_send_string((uint8_t*)"; then next will be 0x");
					if ((y_scan + 1) <= end_scancount)
						conv_uint8_to_2a_hex((y_scan + 1), &mountstring[0]);
					else 
						conv_uint8_to_2a_hex(init_scancount, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[1]);
					con_send_string((uint8_t*)".\r\n");
					single_step = true;
					wait_flag = false;
				}	//if(wait_flag)
			break;	//case 'n'

			case '>':
				if(delay_to_read_x_scan <= DELAY_TO_READ_SIZE-2)
				{
					delay_to_read_x_scan ++;
					con_send_string((uint8_t*)"(>) New delay to read X_Scan (after Y_Scan update) applied: ");
					con_send_string((uint8_t*)&TIME_TO_READ_X[delay_to_read_x_scan][0]);
					con_send_string((uint8_t*)"μs\r\n");
				}
				else
				{
					con_send_string((uint8_t*)"(>) Maximum delay to read X_Scan unchanged: Already workimg at ");
					con_send_string((uint8_t*)&TIME_TO_READ_X[delay_to_read_x_scan][0]);
					con_send_string((uint8_t*)"μs\r\n");
				}
			break;	//case '>':

			case '<':
				if(delay_to_read_x_scan)
				{
					delay_to_read_x_scan --;
					con_send_string((uint8_t*)"(<) New delay to read X_Scan (after Y_Scan update) applied: ");
					con_send_string((uint8_t*)&TIME_TO_READ_X[delay_to_read_x_scan][0]);
					con_send_string((uint8_t*)"μs\r\n");
				}
				else
				{
					con_send_string((uint8_t*)"(<) Minimum delay to read X_Scan unchanged: Already workimg at ");
					con_send_string((uint8_t*)&TIME_TO_READ_X[delay_to_read_x_scan][0]);
					con_send_string((uint8_t*)"μs\r\n");
				}
			break;	//case '<':

			case 'i':
				uint8_t input_buffer[3];
				con_send_string((uint8_t*)"(i) Inactive scan read times, after one active. For this freq (");
				con_send_string((uint8_t*)&SPEED_SELEC[scan_pointer][0]);
				con_send_string((uint8_t*)"Hz),\r\nthe actual value is ");
				if(init_inactivity_cycles[scan_pointer] > MAX_INACT_READ_CYLES[scan_pointer])
					init_inactivity_cycles[scan_pointer] = MAX_INACT_READ_CYLES[scan_pointer];
				conv_uint32_to_dec((uint32_t)init_inactivity_cycles[scan_pointer], &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[0]);
				con_send_string((uint8_t*)" with a maximum value of ");
				conv_uint32_to_dec((uint32_t)MAX_INACT_READ_CYLES[scan_pointer], &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[0]);
				con_send_string((uint8_t*)".");
				if (MAX_INACT_READ_CYLES[scan_pointer] > 0)
				{
					do
					{
						con_send_string((uint8_t*)"\r\nPlease enter a new decimal value: ");
						switch (console_get_line(&input_buffer[0], 2))
						{
							case 2:
								init_inactivity_cycles[scan_pointer] = (input_buffer[0] - '0') * 10 + (input_buffer[1] - '0');
							break;	//case 2:
							case 1:
								init_inactivity_cycles[scan_pointer] = (input_buffer[0] - '0');
							break;	//case 1:
							case 0:
								init_inactivity_cycles[scan_pointer] = 0;
							break;	//case 0:
							default:
							break;	//default:
						}	//switch (console_get_line(&input_buffer[0], 2))
					}while( init_inactivity_cycles[scan_pointer] > MAX_INACT_READ_CYLES[scan_pointer] );
				}	//if (MAX_INACT_READ_CYLES[scan_pointer] > 0)
				con_send_string((uint8_t*)".\r\n");
			break;	//case 'i':
		}	//switch (ch)
	}	//for(;;)
	return 0; //Suppose never reach here
} //int main(void)
