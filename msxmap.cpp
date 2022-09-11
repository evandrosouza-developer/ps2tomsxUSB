/*
 * This file is part of the PS/2 keyboard Interface for MSX project.
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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
//#include <libopencm3/stm32/dbgmcu.h>

#include "system.h"
#include "msxmap.h"
#include "ps2handl.h"
#include "serial.h"
#include "dbasemgt.h"

#define MAX_TIME_OF_IDLE_KEYSCAN_SYSTICKS	4		//30 / 4 = 7.5 times per second is the maximum sweep speed
#define	NIBBLE														4
#define	CASE_MASK													0x03
#define	CASEx_TYPE												3		//Relative position within a line of Case type
#define	CASE0_KEY0												4		//Relative position within a line of first key of case 0
#define	CASE0_KEY1												5		//   "         "       "   "   "  " second  "  "  case 0
#define	CASE1_KEY0												6		//   "         "       "   "   "  "  first  "  "  case 1
#define	CASE1_KEY1												7		//   "         "       "   "   "  " second  "  "  case 1
#define	CASE2_KEY0												6		//Cases 1 and 2 share the same database positions as they don't overlap
#define	CASE2_KEY1												7		//Cases 1 and 2 share the same database positions as they don't overlap
#define X_POLARITY_BIT_POSITION						3
#define X_POLARITY_BIT_MASK								(1 << X_POLARITY_BIT_POSITION) //8
#define	Y_LOCAL_MASK											0xF0//Mask of Y on press/release command byte
#define	X_LOCAL_MASK											7		//Mask of X on press/release command byte
#define	Y_SHIFT														6		//Shift colunm
#define	X_SHIFT														0		//Shift line
#define	MSX_SHIFT_PRESS										(Y_SHIFT<<NIBBLE)|X_SHIFT//Shift press will be resolved as 0x60
#define	Y_GRAPH														6		//Graph colunm
#define	X_GRAPH														2		//Graph line
#define	Y_CTRL														6		//CTRL colunm
#define	X_CTRL														1		//CTRL line
#define	Y_STOP														7		//Stop key colunm
#define	X_STOP														4		//Stop key line

//Variáveis globais: Visíveis por todo o contexto do programa
uint8_t* base_of_database;
extern uint32_t systicks;											//Declared on sys_timer.cpp
extern bool ps2numlockstate;									//Declared on ps2handl.c
bool do_next_keep_alive;
volatile bool shiftstate;
volatile uint16_t linhavarrida;
extern volatile bool update_ps2_leds;					//Declared on ps2handl.c
//Place to store previous time for each Y last scan
volatile uint32_t previous_y_systick[ 16 ] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//Variable used to store the values of the X for each Y scan - Each Y has its own image of BSSR register
uint32_t x_bits[ 16 ]; //All pins that interface with PORT B of 8255 must have high level as default

uint8_t y_dummy;															//Read from MSX Database to sinalize "No keys mapping"
volatile uint32_t formerscancode;
volatile uint8_t scancode[4];									//scancode[0] stores the quantity of bytes;

uint8_t CtrlAltDel;
//First record of unused V.1.0 Database
uint8_t UNUSED_DATABASE[(uint8_t)DB_NUM_COLS] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08};

extern bool enable_xon_xoff;									//Declared on serial.c

#define DISPATCH_QUEUE_SIZE								16
uint8_t dispatch_keys_queue_buffer[DISPATCH_QUEUE_SIZE];

struct sring dispatch_keys_queue;

// Table to translate the Y order: From port read to the expected one:
const uint8_t Y_XLAT_TABLE[uint8_t(16)] = { 0b0000, 0b1000, 0b0100, 0b1100,
																						0b0010, 0b1010, 0b0110, 0b1110,
																			 			0b0001, 0b1001, 0b0101, 0b1101,
																						0b0011, 0b1011, 0b0111, 0b1111};


void msxmap::msx_interface_setup(void)
{
	//Set Alternate function
#if MCU == STM32F401
	gpio_set_af(Y0_PORT, GPIO_AF3, Y0_PIN | Y1_PIN | Y2_PIN | Y3_PIN);
	gpio_set_af( X_PORT, GPIO_AF3, X0_PIN | X1_PIN | X2_PIN | X3_PIN | X4_PIN | X5_PIN | X6_PIN | X7_PIN);
#endif

	//Not the STM32 default, but it is the default MSX state: Release MSX keys;
	gpio_set(X_PORT,
	X7_PIN | X6_PIN | X5_PIN | X4_PIN | X3_PIN | X2_PIN | X1_PIN | X0_PIN);

	//Init output port B
	//gpio_mode_setup(X7_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, 
	//X7_PIN | X6_PIN | X5_PIN | X4_PIN | X3_PIN | X2_PIN | X1_PIN | X0_PIN);
#if MCU == STM32F103
	gpio_set_mode(X7_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, 
	X7_PIN | X6_PIN | X5_PIN | X4_PIN | X3_PIN | X2_PIN | X1_PIN | X0_PIN);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	gpio_mode_setup(X_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
	X7_PIN | X6_PIN | X5_PIN | X4_PIN | X3_PIN | X2_PIN | X1_PIN | X0_PIN);
	gpio_set_output_options(X_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
	X7_PIN | X6_PIN | X5_PIN | X4_PIN | X3_PIN | X2_PIN | X1_PIN | X0_PIN);
#endif

	//Init startup state of BSSR image to each Y scan
	for(uint8_t i = 0; i < 16; i++)
		x_bits[ i ] = X7_SET_OR | X6_SET_OR | X5_SET_OR | X4_SET_OR | X3_SET_OR | X2_SET_OR | X1_SET_OR | X0_SET_OR;
	
	// Initialize dispatch_keys_queue ringbuffer
	ring_init(&dispatch_keys_queue, dispatch_keys_queue_buffer, DISPATCH_QUEUE_SIZE);
	for(uint16_t i=0; i<DISPATCH_QUEUE_SIZE; ++i)
		dispatch_keys_queue.data[i]=0;

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC3 MSX 8255 Pin 17)
	//gpio_set(Y3_PORT, Y3_PIN); //pull up resistor
#if MCU == STM32F103
	gpio_set(Y3_PORT, Y3_PIN); //pull up resistor
	gpio_set_mode(Y3_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, Y3_PIN); // PC3 (MSX 8255 Pin 17)
	exti_select_source(Y3_exti, Y3_PORT);
	exti_set_trigger(Y3_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y3_exti);
	exti_enable_request(Y3_exti);
	gpio_port_config_lock(Y3_PORT, Y3_PIN);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	gpio_mode_setup(Y3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, Y3_PIN); // PC3 (MSX 8255 Pin 17)
	exti_select_source(Y3_exti, Y3_PORT);
	exti_set_trigger(Y3_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y3_exti);
	exti_enable_request(Y3_exti);
	gpio_port_config_lock(Y3_PORT, Y3_PIN);
#endif

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC2 MSX 8255 Pin 16)
	//gpio_set(Y2_PORT, Y2_PIN); //pull up resistor
#if MCU == STM32F103
	gpio_set(Y2_PORT, Y2_PIN); //pull up resistor
	gpio_set_mode(Y2_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, Y2_PIN); // PC2 (MSX 8255 Pin 16)
	exti_select_source(Y2_exti, Y2_PORT);
	exti_set_trigger(Y2_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y2_exti);
	exti_enable_request(Y2_exti);
	gpio_port_config_lock(Y2_PORT, Y2_PIN);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	gpio_mode_setup(Y2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, Y2_PIN); // PC2 (MSX 8255 Pin 16)
	exti_select_source(Y2_exti, Y2_PORT);
	exti_set_trigger(Y2_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y2_exti);
	exti_enable_request(Y2_exti);
	gpio_port_config_lock(Y2_PORT, Y2_PIN);
#endif

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC1 MSX 8255 Pin 15)
	//gpio_set(Y1_PORT, Y1_PIN); //pull up resistor
#if MCU == STM32F103
	gpio_set(Y1_PORT, Y1_PIN); //pull up resistor
	gpio_set_mode(Y1_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, Y1_PIN); // PC1 (MSX 8255 Pin 15)
	exti_select_source(Y1_exti, Y1_PORT);
	exti_set_trigger(Y1_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y1_exti);
	exti_enable_request(Y1_exti);
	gpio_port_config_lock(Y1_PORT, Y1_PIN);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	gpio_mode_setup(Y1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, Y1_PIN); // PC1 (MSX 8255 Pin 15)
	exti_select_source(Y1_exti, Y1_PORT);
	exti_set_trigger(Y1_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y1_exti);
	exti_enable_request(Y1_exti);
	gpio_port_config_lock(Y1_PORT, Y1_PIN);
#endif

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC0 MSX 8255 Pin 14)
	//gpio_set(Y0_PORT, Y0_PIN); //pull up resistor
#if MCU == STM32F103
	gpio_set(Y0_PORT, Y0_PIN); //pull up resistor
	gpio_set_mode(Y0_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, Y0_PIN); // PC0 (MSX 8255 Pin 14)
	exti_select_source(Y0_exti, Y0_PORT);
	exti_set_trigger(Y0_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y0_exti);
	exti_enable_request(Y0_exti);
	gpio_port_config_lock(Y0_PORT, Y0_PIN);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	gpio_mode_setup(Y0_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, Y0_PIN); // PC0 (MSX 8255 Pin 14)
	exti_select_source(Y0_exti, Y0_PORT);
	exti_set_trigger(Y0_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y0_exti);
	exti_enable_request(Y0_exti);
	gpio_port_config_lock(Y0_PORT, Y0_PIN);
#endif

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC0 MSX 8255 Pin 14),
	// CAPS_LED and KANA/Cyrillic_LED (mapped to scroll lock) to replicate in PS/2 keyboard
	// Set both to input and enable internal pullup

	// CAPS_LED
#if MCU == STM32F103
	gpio_set_mode(CAPSLOCK_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, CAPSLOCK_PIN); // CAP_LED (MSX 8255 Pin 11)
	gpio_set(CAPSLOCK_PORT, CAPSLOCK_PIN); //pull up resistor
	gpio_port_config_lock(CAPSLOCK_PORT, CAPSLOCK_PIN);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	gpio_mode_setup(CAPSLOCK_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, CAPSLOCK_PIN); // CAP_LED (MSX 8255 Pin 11)
	gpio_set(CAPSLOCK_PORT, CAPSLOCK_PIN); //pull up resistor
	gpio_port_config_lock(CAPSLOCK_PORT, CAPSLOCK_PIN);
#endif

	// Kana LED
#if MCU == STM32F103
	gpio_set_mode(KANA_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, KANA_PIN); // KANA_LED - Mapeado para Scroll Lock
	gpio_set(KANA_PORT, KANA_PIN); //pull up resistor
	gpio_port_config_lock(KANA_PORT, KANA_PIN);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	gpio_mode_setup(KANA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, KANA_PIN); // KANA_LED - Mapeado para Scroll Lock
	gpio_set(KANA_PORT, KANA_PIN); //pull up resistor
	gpio_port_config_lock(KANA_PORT, KANA_PIN);
#endif

	// Enable EXTI9_5 interrupt. (for Y - bits 3 to 0)
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);

	//Highest priority to avoid interrupt Y scan loss
	nvic_set_priority(NVIC_EXTI9_5_IRQ, IRQ_PRI_Y_SCAN); 		//Y3 to Y0

#if MCU == STM32F401
	//Workaround here to stop iwdg, wwdg and TIMx counters during halt
#if TIM_HR == TIM2
	DBGMCU_APB1_FZ = DBG_IWDG_STOP | DBG_WWDG_STOP | DBG_TIM2_STOP;
#endif	//#if TIM_HR == TIM2
#if TIM_HR == TIM3
	DBGMCU_APB1_FZ = DBG_IWDG_STOP | DBG_WWDG_STOP | DBG_TIM3_STOP;
#endif	//#if TIM_HR == TIM3
#if TIM_HR == TIM4
	DBGMCU_APB1_FZ = DBG_IWDG_STOP | DBG_WWDG_STOP | DBG_TIM4_STOP;
#endif	//#if TIM_HR == TIM4
#if TIM_HR == TIM5
	DBGMCU_APB1_FZ = DBG_IWDG_STOP | DBG_WWDG_STOP | DBG_TIM5_STOP;
#endif	//#if TIM_HR == TIM5
#endif	//#if MCU == STM32F401
}


// Verify if there is an available ps2_byte_received on the receive ring buffer, but does not fetch this one
//Input: void
//Output: True if there is char available in input buffer or False if none
bool msxmap::available_msx_disp_keys_queue_buffer(void)
{
	uint8_t i = dispatch_keys_queue.get_ptr;
	if(i == dispatch_keys_queue.put_ptr)	//msx_dispatch_kq_put)
		return false;	//No char in buffer
	else
		return true;
}


// Fetches the next ps2_byte_received from the receive ring buffer
//Input: void
//Outut: Available byte read
uint8_t msxmap::get_msx_disp_keys_queue_buffer(void)
{
	uint8_t i, result;

	i = dispatch_keys_queue.get_ptr;
	if(i == dispatch_keys_queue.put_ptr)
		//No char in buffer
		return 0;
	result = dispatch_keys_queue.data[i];
	i++;
	dispatch_keys_queue.get_ptr = i & (uint8_t)(DISPATCH_QUEUE_SIZE- 1); //if(i) >= (uint16_t)DISPATCH_QUEUE_SIZE)	i = 0;
	return result;
}


/*Put a MSX Key (byte) into a buffer to get it mounted to x_bits
* Input: uint8_t as parameter
* Output: Total number of bytes in buffer, or ZERO if buffer was already full.*/
uint8_t msxmap::put_msx_disp_keys_queue_buffer(uint8_t data_word)
{
	uint8_t i, i_next;
	
	i = dispatch_keys_queue.put_ptr;
	i_next = i + 1;
	i_next &= (uint8_t)(DISPATCH_QUEUE_SIZE - 1);
	if (i_next != dispatch_keys_queue.get_ptr)
	{
		dispatch_keys_queue_buffer[i] = data_word;
		dispatch_keys_queue.put_ptr = i_next;
		return (uint8_t)(DISPATCH_QUEUE_SIZE - dispatch_keys_queue.get_ptr + dispatch_keys_queue.put_ptr) & (uint8_t)(DISPATCH_QUEUE_SIZE - 1);
	}
	else 
		return 0;
}


//The objective of this routine is implement a smooth typing.
//The usage is to put a byte key (bit 7:4 represents Y, bit 3 is the release=1/press=0, bits 2:0 are the X )
//F = 15Hz. This routine is called from Ticks interrupt routine
void msxmap::msxqueuekeys(void)
{
	uint8_t x_local, y_local, readkey;
	bool x_local_setb;
	if (available_msx_disp_keys_queue_buffer())
	{
		//This routine is allways called AFTER the mapped condition has been tested 
		readkey = get_msx_disp_keys_queue_buffer();
		y_local = (readkey & Y_LOCAL_MASK) >> NIBBLE;
		x_local = readkey & X_LOCAL_MASK;
		x_local_setb = ((readkey & X_POLARITY_BIT_MASK) >> X_POLARITY_BIT_POSITION) == (uint8_t)1;
		// Compute x_bits of ch and verifies when Y was last updated,
		// with aim to update MSX keys, no matters if the MSX has the interrupts stucked. 
		compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
	}
}


void msxmap::convert2msx()
{
	switch(CtrlAltDel)
	{
		case 0:
		{
			//First state: No reset keys
			if (
			(scancode[0] == (uint8_t)1) 		&&
			(scancode[1] == (uint8_t)0x14) ) //Left Control
				CtrlAltDel = 1;
			break;
		}
		case 1:
		{
			//Second state: Left Control
			if (
			(scancode[0] == (uint8_t)1) 		&&
			(scancode[1] == (uint8_t)0x11) ) //Left Alt
				//Left Control + Left Alt are pressed together
				CtrlAltDel = 2;
			else
				CtrlAltDel = 0;
			break;
		}
		case 2:
		{
			//Third state: Num Pad Del
			if (
			(scancode[0] == (uint8_t)1) 		&&
			(scancode[1] == (uint8_t)0x71) ) //Num pad Del
			{
				//Left Control + Left Alt + Num Pad Del are pressed together
				//User messages
				con_send_string((uint8_t*)"Reset requested by user\r\n");
				reset_requested();
			}
			else
				CtrlAltDel = 0;
		}
	}

	if (
	(scancode[0] == (uint8_t)1) 		&&
	(scancode[1] == (uint8_t)0x77) )
	{
		//NumLock Pressed. Toggle NumLock status
		ps2numlockstate = !ps2numlockstate;
		update_ps2_leds = true;  //this will force update_leds at main loop
		return;
	}

	if (
	(scancode[0] == (uint8_t)2) 		&&
	(scancode[1] == (uint8_t)0xF0)	&&
	(scancode[2] == (uint8_t)0x77) )
	{
		//NumLock Released. Return with no action
		return;
	}

	if (
	( scancode[0] == (uint8_t)1)	  &&
	((scancode[1] == (uint8_t)0x12) || (scancode[1] == (uint8_t)0x59)) )
	{
		//Shift Pressed.
		shiftstate = true;
	}

	if (
	( scancode[0] == (uint8_t)2)		&&
	( scancode[1] == (uint8_t)0xF0) &&
	((scancode[2] == (uint8_t)0x12) || (scancode[2]== (uint8_t)0x59)) )
	{
		//Shift Released (Break code).
		shiftstate = false;
	}

	if (	//on release of Shift, Ctrl, Graph or Code, release all presses
	//Check PS/2 Shift released
	((scancode[0] == (uint8_t)2)		&&
	( scancode[1] == (uint8_t)0xF0) &&
	((scancode[2] == (uint8_t)0x12) || (scancode[2]== (uint8_t)0x59)))	||
	//Check PS/2 Windows key (GRAPH) released
	((scancode[0] == (uint8_t)3)		&&
	( scancode[1] == (uint8_t)0xE0) &&
	( scancode[2] == (uint8_t)0xF0) &&
	((scancode[3] == (uint8_t)0x1F) || (scancode[3]== (uint8_t)0x27)))	||
	//Check PS/2 left Ctrl key released
	((scancode[0] == (uint8_t)2)		&&
	( scancode[1] == (uint8_t)0xF0) &&
	((scancode[2] == (uint8_t)0x14)))																		||
	//Check PS/2 right Ctrl key released
	((scancode[0] == (uint8_t)3)		&&
	( scancode[1] == (uint8_t)0xE0) &&
	( scancode[2] == (uint8_t)0xF0) &&
	( scancode[3] == (uint8_t)0x14))																		||
	//Check PS/2 left Alt key (CODE) released
	((scancode[0] == (uint8_t)2)		&&
	( scancode[1] == (uint8_t)0xF0) &&
	(	scancode[2] == (uint8_t)0x11))																		||
	//Check PS/2 right Alt key (CODE) released
	((scancode[0] == (uint8_t)3)		&&
	( scancode[1] == (uint8_t)0xE0) &&
	( scancode[2] == (uint8_t)0xF0) &&
	(	scancode[3] == (uint8_t)0x11))
		 )
	{
		//Shift and/or Graph and/or Control and/or Code were/was released, then force release of all other keys
		for(uint8_t i = 0; i < 16; i++)
			x_bits[ i ] = X7_SET_OR | X6_SET_OR | X5_SET_OR | X4_SET_OR | X3_SET_OR | X2_SET_OR | X1_SET_OR | X0_SET_OR;
		return;
	}

	if (
	((scancode[0] == (uint8_t)3)		&&
	( scancode[1] == (uint8_t)0xE1) &&
	( scancode[2] == (uint8_t)0x14) &&
	( scancode[3] == (uint8_t)0x77))	||
	((scancode[0] == (uint8_t)2)		&&
	( scancode[1] == (uint8_t)0xE0)	&&
	( scancode[2] == (uint8_t)0x7E))	)
	{
		//Pause Break Key & Control + Break (Control was already sent at this time).
		//Create an effect of press and release of MSX STOP, through:
		//dispatch MSX STOP pressed 3 times and 1 MSX STOP release
		for(uint16_t i = 0; i < 3; i++)	//Keep the MSX STOP key pressed for a while
			put_msx_disp_keys_queue_buffer(Y_STOP<<NIBBLE | X_STOP);
		put_msx_disp_keys_queue_buffer(Y_STOP<<NIBBLE | X_POLARITY_BIT_MASK | X_STOP);	//Then release MSX STOP key
		return;
	}

	//Now searches for PS/2 scan code in Database to match. First search first colunm
	linhavarrida = 1;
	while ( 
	//(MSX_KEYB_DATABASE_CONVERSION[linhavarrida][0] != scancode[1]) &&
	(*(base_of_database+linhavarrida*DB_NUM_COLS+0) != scancode[1]) &&
	(linhavarrida < N_DATABASE_REGISTERS) )
	{
		linhavarrida++;
	}
	if (
	(scancode[0] == (uint8_t)1) && 
	(scancode[1]  == *(base_of_database+linhavarrida*DB_NUM_COLS+0)) && 
	(linhavarrida < N_DATABASE_REGISTERS) )
	{
		//1 byte key
		//msx_dispatch(linhavarrida);
		msx_dispatch();
		return;
	}
	if ((scancode[0] >= (uint8_t)1) &&
	(linhavarrida < N_DATABASE_REGISTERS) )
	{
		//2 bytes key, then now search match on second byte of scancode
		while ( 
		(*(base_of_database+linhavarrida*DB_NUM_COLS+1) != scancode[2]) &&
		(linhavarrida < N_DATABASE_REGISTERS))
		{
			linhavarrida++;
		}
		if(
		(scancode[0] == (uint8_t)2) && 
		(scancode[1] == *(base_of_database+linhavarrida*DB_NUM_COLS+0)) && 
		(scancode[2] == *(base_of_database+linhavarrida*DB_NUM_COLS+1)) )
		{
			//Ok: Matched code
			//msx_dispatch(linhavarrida);
			msx_dispatch();
			return;
		}
		//3 bytes key, then now search match on third byte of scancode
		while (
		(*(base_of_database+linhavarrida*DB_NUM_COLS+2) != scancode[3]) &&
		(linhavarrida < N_DATABASE_REGISTERS) )
		{
			linhavarrida++;
		}
		if( 
		(scancode[0] == 3) && 
		(scancode[1] == *(base_of_database+linhavarrida*DB_NUM_COLS+0)) && 
		(scancode[2] == *(base_of_database+linhavarrida*DB_NUM_COLS+1)) &&
		(scancode[3] == *(base_of_database+linhavarrida*DB_NUM_COLS+2)) )
		{
			//Ok: Matched code
			//msx_dispatch(linhavarrida);
			msx_dispatch();
			return;
		}
	}	//if ((scancode[0] >= (uint8_t)1) && (linhavarrida < N_DATABASE_REGISTERS) )
}	//void msxmap::convert2msx()

	/*
	The structure of the Database is:
		The  three first columns of each line are the mapped scan codes;
		The 4th column is The Control Byte, detailed bellow:
		CONTROL BYTE:
			High nibble is Reserved;
			(bit 3) Combined Shift;
			(bit 2) Reserved-Not used;
			(bits 1-0) Modifyer Type:
			.0 - Default mapping
			.1 - NumLock Status+Shift changes
			.2 - PS/2 Shift
			.3 - Reserved-Not used
		
		This table has 3 modifyers: Up two MSX keys are considered to each mapping behavior modifying:
		
		5th and 6th columns have the mapping ".0 - Default mapping";
		7th e 8th columns have the mapping ".1 - NumLock Status+Shift changes";
		9th and 10th columns have the mapping ".2 - PS/2 Shift", where I need to
		release the sinalized Shift in PS/2 to the MSX and put the coded key, and so,
		release them, reapplying the Shift key, deppending on the initial state;
		
		
		Each column has a MSX coded key with the following structure:
		(Bit 7:4) MSX Y PPI 8255 PC3:0 is send to an OC outputs BCD decoder, for example:
							In the case of Hotbit HB8000, the keyboard scan is done as a 9 columns scan, CI-16 7445N 08 to 00;
							If equals to 1111 (Y=15), there is no MSX key mapped.
		(Bit 3)		0: keypress
							1: key release;
		(Bit 2:0) MSX X, ie, which bit will carry the key, to be read by PPI 8255 PB7:0.
	
	
	Now in my native language: Português
	 Primeiramente verifico se este scan, que veio em scancode[n], está referenciado na tabela MSX_KEYB_DATABASE_CONVERSION.
	 Esta tabela, montada em excel, está pronta para ser colada
	 A tabela já está com sort, para tornar possível executar uma pesquisa otimizada.
	
	 As três primeiras posições (colunas) de cada linha são os scan codes mapeados;
	 A 4ª coluna é o controle, e tem a seguinte estrutura:
		CONTROL BYTE:
		High nibble is Reserved; 
		(bit 3) Combined Shift;
		(bit 2) Reserved-Not used;
		(bits 1-0) Modifyer Type:
		.0 - Default mapping
		.1 - NumLock Status+Shift changes
		.2 - PS/2 Shift
		.3 - Reserved-Not used
	
	Esta tabela contém 3 modificadores: São codificadas até 2 teclas do MSX para cada modificador de mapeamento:
	
	 5ª e 6ª colunas contém o mapeamento ".0 - Default mapping";
	 7ª e 8ª colunas contém o mapeamento ".1 - NumLock Status+Shift changes";
	 9ª e 10ª colunas contém o mapeamento ".2 - PS/2 Shift", onde necessito
	 liberar o Shift sinalizado no PS/2 para o MSX e colocar as teclas codificadas,
	 e liberá-las, reinserindo o Shift, se aplicável;
	
	
	 Cada coluna contém uma tecla codificada para o MSX, com a seguinte estrutura:
	 (Bit 7:4) MSX Y PPI 8255 PC3:0 é enviada à um decoder BCD com OC outputs.
						 MSX Y PPI 8255 PC3:0. No caso do Hotbit, o scan é feito em 9 colunas, CI-16 7445N 08 a 00;
						 Se 1111 (Y=15), não há tecla MSX mapeada.
	 (Bit 3)	 0: keypress
						 1: key release;
	 (Bit 2:0) MSX X, ou seja, qual bit levará a informação da tecla, a ser lida pela PPI 8255 PB7:0.
	
	*/

//void msxmap::msx_dispatch(volatile uint16_t linhavarrida)
void msxmap::msx_dispatch(void)
{
	volatile uint8_t y_local, x_local;
	volatile bool x_local_setb;

 	switch(*(base_of_database+linhavarrida*DB_NUM_COLS+CASEx_TYPE) & CASE_MASK)
	{
		case 0:
		{	// .0 - Default mapping (Columns 4 & 5)
			// Key 0:
			y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)Y_LOCAL_MASK) >> NIBBLE;
			// Verify if key is mapped
			if (y_local != y_dummy)
			{
				x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)X_LOCAL_MASK;
				x_local_setb = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)X_POLARITY_BIT_MASK) >> X_POLARITY_BIT_POSITION;
				// Calcula x_bits da Key 0 e verifica se o tempo em que foi atualizado o dado de Linha X da Coluna Y,
				// com a finalidade de atualizar teclas mesmo sem o PPI ser atualizado. 
				compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
			}
			// Key 1:
			y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & (uint8_t)Y_LOCAL_MASK) >> NIBBLE;
			// Verify if key is mapped
			if (y_local != y_dummy)
			{
				x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & (uint8_t)X_LOCAL_MASK;
				if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == (MSX_SHIFT_PRESS))
				{
					//Shift key
					if(shiftstate)
						put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS);	//return MSX Shift key as pressed state
					else
						put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS | X_POLARITY_BIT_MASK);	//return MSX Shift key as released state
				}	//	if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
				else // if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
				{
					//Other key: different from Shift key
					put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1));
				}	//else // if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
			}	//if (y_local != y_dummy)
			break;
		} // .0 - Mapeamento default (Colunas 4 e 5)

		case 1:
		{	// .1 - Mapeamento NumLock (Colunas 6 e 7)
			if (ps2numlockstate ^ shiftstate)
			{
				//numlock and Shift have different status
				// Key 0 (PS/2 NumLock ON (Default)):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)Y_LOCAL_MASK)  >> NIBBLE;
				// Verify if key is mapped
				if (y_local != y_dummy)
				{
					x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)X_LOCAL_MASK;
					x_local_setb = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)X_POLARITY_BIT_MASK) >> X_POLARITY_BIT_POSITION;
					compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
				}
				// Key 1 (PS/2 NumLock ON (Default)):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & (uint8_t)Y_LOCAL_MASK) >> NIBBLE;
				// Verify if key is mapped
				if (y_local != y_dummy)
				{
					x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & (uint8_t)X_LOCAL_MASK;
					if( (Y_SHIFT == y_local) && (X_SHIFT == x_local) )	//if SHIFT, return to shiftstate
					{
						//Shift key
						if(shiftstate)
							put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS);	//return MSX Shift key as pressed state
						else
							put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS | X_POLARITY_BIT_MASK);	//return MSX Shift key as released state
					}	//if( (Y_SHIFT == y_local) && (X_SHIFT == x_local) )
					else // if( (Y_SHIFT == y_local) && (X_SHIFT == x_local) )
					{
						//Other key: different from Shift key
						put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1));
					}	//else // if( (Y_SHIFT == y_local) && (X_SHIFT == x_local) )
				}	//if (y_local != y_dummy)
			}
			else //if (ps2numlockstate ^ shiftstate)
			{
				//numlock and Shift share the same status
				// Verify if there are keys mapped
				// Key 0 (PS/2 NumLock OFF):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY0) & (uint8_t)Y_LOCAL_MASK)  >> NIBBLE;
				// Verify if key is mapped
				if (y_local != y_dummy)
				{
					x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY0) & (uint8_t)X_LOCAL_MASK;
					x_local_setb = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY0) & (uint8_t)X_POLARITY_BIT_MASK) >> X_POLARITY_BIT_POSITION;
					// Calcula x_bits da Key 0 e verifica se o tempo em que foi atualizado o dado de Linha X da Coluna Y,
					// com a finalidade de atualizar teclas mesmo sem o PPI ser atualizado. 
					compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
				}	//if (y_local != y_dummy)
				// Key 1 (PS/2 NumLock OFF):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY1) & (uint8_t)Y_LOCAL_MASK) >> NIBBLE;
				// Verify if key is mapped
				if (y_local != y_dummy)
				{
					if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == (MSX_SHIFT_PRESS))
					{
						//Shift key
						if(shiftstate)
							put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS);	//return MSX Shift key as pressed state
						else
							put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS | X_POLARITY_BIT_MASK);	//return MSX Shift key as released state
					}	//if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
					else // if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
					{
						//Other key: different from Shift key
						put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY1));
					}	//else // if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE1_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
				}	//if (y_local != y_dummy)
			} //if (ps2numlockstate ^ shiftstate)
			break;
		}  // .1 - Mapeamento NumLock (Colunas 6 e 7)

		case 2:
		{	// .2 - Mapeamento alternativo (PS/2 Left and Right Shift)  (Colunas 8 e 9)
			if (!shiftstate)
			{
				//Shift state is OFF (not pressed), so performs as CASE 0 (see the displacement CASE0_KEY...)
				// Key 0 (PS/2 NumLock ON (Default)):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)Y_LOCAL_MASK)  >> NIBBLE;
				if (y_local != y_dummy)	// Verify if key is mapped
				{
					x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)X_LOCAL_MASK;
					x_local_setb = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY0) & (uint8_t)X_POLARITY_BIT_MASK) >> X_POLARITY_BIT_POSITION;
					// Calcula x_bits da Key 0 e verifica se o tempo em que foi atualizado o dado de Linha X da Coluna Y,
					// com a finalidade de atualizar teclas mesmo sem o PPI ser atualizado. 
					compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
				}
				// Key 1 (PS/2 NumLock ON (Default)):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & (uint8_t)Y_LOCAL_MASK) >> NIBBLE;
				if (y_local != y_dummy)	// Verify if key is mapped
				{
					x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & (uint8_t)X_LOCAL_MASK;
					if( ((*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1)) & ~(uint8_t)X_POLARITY_BIT_MASK) == (MSX_SHIFT_PRESS))
					{
						//Shift key
						if(shiftstate)
							put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS);	//return MSX Shift key as pressed state
						else
							put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS | X_POLARITY_BIT_MASK);	//return MSX Shift key as released state
					}	//if( (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
					else // if( (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
					{
						//Other key: different from Shift key
						put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1));
					}	//else // if( (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE0_KEY1) & ~(uint8_t)X_POLARITY_BIT_MASK) == MSX_SHIFT_PRESS)
				}	//if (y_local != y_dummy)
			}
			else //case 2: shiftstate is ON now!, so it is a true case 2 (see the displacement CASE2_KEY...)
			{
				// Verifica se ha teclas mapeadas
				// Key 0 (PS/2 Left and Right Shift):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY0) & (uint8_t)Y_LOCAL_MASK) >> NIBBLE;
				if (y_local != y_dummy)	// Verify if key is mapped
				{
					//if CODE or GRAPH keys to be pressed, then first release MSX Shift key, and so, press CODE (or GRAPH)
					if(	(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY0) == (uint8_t)0x64) ||	//CODE key pressed
							(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY0) == (uint8_t)0x62)	)	//GRAPH key pressed
					{
						//then first release MSX Shift key
						compute_x_bits_and_check_interrupt_stuck(Y_SHIFT, X_SHIFT, true);
						//and so, press CODE (or GRAPH)
						put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY0));
					}  //if (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY0) == 0x64) //if CODE key pressed
					else	//put default key
					{
						x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY0) & (uint8_t)X_LOCAL_MASK;
						x_local_setb = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY0) & (uint8_t)X_POLARITY_BIT_MASK) >> X_POLARITY_BIT_POSITION;
						compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
					}
				}	//if (y_local != y_dummy)
				// Key 1 (PS/2 Left and Right Shift):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY1) & (uint8_t)Y_LOCAL_MASK) >> NIBBLE;
				// Verify if key is mapped
				if (y_local != y_dummy)
				{
					//So, first send CODE (or GRAPH) released, or another one, BUT if is MSX Shift at this position, it is release
					if( (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY1) & ~(uint8_t)X_POLARITY_BIT_MASK)
																																									== (MSX_SHIFT_PRESS) )
					{
						//Shift key
						if(shiftstate)
							put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS);	//return MSX Shift key as pressed state
						else
							put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS | X_POLARITY_BIT_MASK);	//return MSX Shift key as released state
					}	//if( (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY1) & ~(uint8_t)X_POLARITY_BIT_MASK) == (MSX_SHIFT_PRESS))
					else	//if( (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY1) & ~(uint8_t)X_POLARITY_BIT_MASK) == (MSX_SHIFT_PRESS))
					{
						put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY1));
						//and then, if it is CODE (or GRAPH) Release, reinsert the dropped MSX Shift key
						if(	(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY1) == (uint8_t)0x6C) ||	//if CODE key released
								(*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY1) == (uint8_t)0x6A)	) //if GRAPH key released
						{
							//and then, as it is CODE (or GRAPH) Release, reinsert the dropped MSX Shift key
							if(shiftstate)
								put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS);	//return MSX Shift key as pressed state
							else
								put_msx_disp_keys_queue_buffer(MSX_SHIFT_PRESS | X_POLARITY_BIT_MASK);	//return MSX Shift key as released state
						}	//if CODE key released or GRAPH key released
					}	//if( (*(base_of_database+linhavarrida*DB_NUM_COLS+CASE2_KEY1) & ~(uint8_t)X_POLARITY_BIT_MASK) == (MSX_SHIFT_PRESS))
				}	//if (y_local != y_dummy)
			}	//case 2: if (!shiftstate)
		}  // .2 - Alternate Mapping (PS/2 Left and Right Shift)  (Colunas 8 e 9)
	}	//switch(*(base_of_database+linhavarrida*DB_NUM_COLS+3) & CASE_MASK)
}	//void msxmap::msx_dispatch(void)


void msxmap::compute_x_bits_and_check_interrupt_stuck (
	volatile uint8_t y_local, uint8_t x_local, bool x_local_setb)
{
	uint16_t msx_Y_scan;
	switch (x_local)
	{
		case 0:
		{
			if (x_local_setb)	//key release
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X0_SET_OR) & X0_SET_AND;
 				break;
			}
			else 							//keypress
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X0_CLEAR_AND) | X0_CLEAR_OR;
				break;
			}
		}
		case 1:
		{
			if (x_local_setb)	//key release
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X1_SET_OR) & X1_SET_AND;
				break;
			}
			else							//keypress
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X1_CLEAR_AND) | X1_CLEAR_OR;
				break;
			}
		}
		case 2:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X2_SET_OR) & X2_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X2_CLEAR_AND) | X2_CLEAR_OR;
				break;
			}
		}
		case 3:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X3_SET_OR) & X3_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X3_CLEAR_AND) | X3_CLEAR_OR;
				break;
			}
		}
		case 4:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X4_SET_OR) & X4_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X4_CLEAR_AND) | X4_CLEAR_OR;
				break;
			}
		}
		case 5:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X5_SET_OR) & X5_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X5_CLEAR_AND) | X5_CLEAR_OR;
				break;
			}
		}
		case 6:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X6_SET_OR) & X6_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X6_CLEAR_AND) | X6_CLEAR_OR;
				break;
			}
		}
		case 7:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X7_SET_OR) & X7_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X7_CLEAR_AND) | X7_CLEAR_OR;
				break;
			}
		}
	}
	//See when the Y colunm's XLine was updated, in order to update keys even without the PPI being updated.
	if (systicks - previous_y_systick[Y_XLAT_TABLE[y_local]] > MAX_TIME_OF_IDLE_KEYSCAN_SYSTICKS)
	{
		//MSX is not updating Y, so updating keystrokes by interrupts is not working
		//Verify the actual hardware Y_SCAN
		// First I have to disable Y_SCAN interrupts, to avoid misspelling due to updates
		exti_disable_request(Y3_exti | Y2_exti | Y1_exti | Y0_exti);
#if MCU == STM32F401
		// Read the MSX keyboard Y scan through G5PIO pins A5:A8, mask to 0 other bits and rotate right 5
		msx_Y_scan = (gpio_port_read(Y0_PORT) & Y_MASK) >> 5;
		
		if(Y_XLAT_TABLE[y_local] == msx_Y_scan)
			GPIO_BSRR(X_PORT) = x_bits[msx_Y_scan]; //Atomic GPIOB update => Release and press MSX keys for this column
#endif	//#if MCU == STM32F401

#if MCU == STM32F103
		// Read the MSX keyboard Y scan through GPIO pins A12:A8, mask to 0 other bits and rotate right 8 and 9
		msx_Y_scan = (gpio_port_read(GPIOA)) & Y_MASK;
		msx_Y_scan = ((msx_Y_scan >> 8) & 0x3) | ((msx_Y_scan >> 9) & 0xC);

		GPIOB_BSRR = x_bits[msx_Y_scan]; //Atomic GPIOB update => Release and press MSX keys for this column
#endif	//#if MCU == STM32F103

		//Than reenable Y_SCAN interrupts
		exti_enable_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
	}
}


/*************************************************************************************************/
/*************************************************************************************************/
/******************************************* ISR's ***********************************************/
/*************************************************************************************************/
/*************************************************************************************************/
#if MCU == STM32F103
void exti15_10_isr(void) // PC2 and PC3 - It works like interrupt on change of each one of Y connected pins
{
	volatile uint16_t msx_Y_scan;

	//Debug & performance measurement
	gpio_clear(Dbg_Yint_PORT, Dbg_Yint2e3_PIN); //Signs start of interruption

	// Read the MSX keyboard Y scan through GPIO pins A12:A8, mask to 0 other bits and rotate right 8
	msx_Y_scan = (gpio_port_read(GPIOA)>>8) & 0x0F;
	
	GPIOB_BSRR = x_bits[msx_Y_scan]; //Atomic GPIOB update => Release and press MSX keys for this column

	//Debug & performance measurement
	gpio_set(Dbg_Yint_PORT, Dbg_Yint2e3_PIN); //Signs end of interruption. Default condition is "1"
    
	// Clear interrupt Y Scan flags, including those not used on this ISR
	// if(exti_get_flag_status(EXTI7), (EXTI6), (EXTI4) and (EXTI3))
	exti_reset_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
    
	//Update systicks (time stamp) for this Y
	previous_y_systick[msx_Y_scan]  = systicks;
}

void exti9_5_isr(void) // PC0 and PC1 - It works like interrupt on change of each one of Y connected pins
{
	volatile uint16_t msx_Y_scan;

	//Debug & performance measurement
	gpio_clear(Dbg_Yint_PORT, Dbg_Yint0e1_PIN); //Signs start of interruption

	// Read the MSX keyboard Y scan through GPIO pins A12:A8, mask to 0 other bits and rotate right 8 and 9
	msx_Y_scan = (gpio_port_read(GPIOA)) & Y_MASK;
	msx_Y_scan = ((msx_Y_scan >> 8) & 0x3) | ((msx_Y_scan >> 9) & 0xC);

	/*//Port A10 broken => Using B4 instead
	if (gpio_get(Y2_port, Y2_PIN))
		msx_Y_scan |= 0b0100;
	else
		msx_Y_scan &= ~0b0100;*/
	
	GPIOB_BSRR = x_bits[msx_Y_scan]; //Atomic GPIOB update => Release and press MSX keys for this column

	//Debug & performance measurement
	gpio_set(Dbg_Yint_PORT, Dbg_Yint0e1_PIN); //Signs end of interruption. Default condition is "1"

	// Clear interrupt Y Scan flags, including those not used on this ISR
	// if(exti_get_flag_status(EXTI7), (EXTI6), (EXTI4) and (EXTI3))
	exti_reset_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
    
	//Update systicks (time stamp) for this Y
	previous_y_systick[msx_Y_scan]  = systicks;
}


#if MCU == STM32F401
void exti9_5_isr(void) // PC3, PC2, PC1 and PC0 - This ISR works like interrupt on change of each one of Y connected pins
{
	volatile uint16_t msx_Y_scan;

	//Debug & performance measurement
	//gpio_clear(Dbg_Yint_PORT, Dbg_Yint0e1_PIN); //Signs start of interruption. This line is useful only to measure performance, ie, only in development phase
	GPIO_BSRR(Dbg_Yint_PORT) = Dbg_Yint_PIN << 16;

	// Read the MSX keyboard Y scan through GPIO pins A12:A8, mask to 0 other bits and rotate right 8 and 9
	msx_Y_scan = (gpio_port_read(Y0_PORT) & Y_MASK) >> 5;
	
	if(Y_XLAT_TABLE[y_local] == msx_Y_scan)
		GPIO_BSRR(X_PORT) = x_bits[msx_Y_scan]; //Atomic GPIOB update => Release and press MSX keys for this column

	//Debug & performance measurement
	//gpio_set(Dbg_Yint_PORT, Dbg_Yint_PIN); //Signs end of interruption. Default condition is "1". This line is useful only to measure performance, ie, only in development phase
	GPIO_BSRR(Dbg_Yint_PORT) = Dbg_Yint_PIN;

	// Clear interrupt Y Scan flags
	exti_reset_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
    
	//Update systicks (time stamp) for this Y
	previous_y_systick[msx_Y_scan] = systicks;
}
#endif	//#if MCU == STM32F401


/*void exti4_isr(void)	//Port A10 broken => Using B4 instead
{
	//Debug & performance measurement
	gpio_clear(Dbg_Yint_port, Dbg_Yint2e3_PIN); //Signs start of interruption

	// Read the MSX keyboard Y scan through GPIO pins A11:A8, mask to 0 other bits and rotate right 8
	msx_Y_scan = (gpio_port_read(GPIOA)>>8) & 0x0F;
	
	//Port A10 broken => Using B4 instead
	if (gpio_get(Y2_port, Y2_PIN))
		msx_Y_scan |= 0b0100;
	else
		msx_Y_scan &= ~0b0100;
	
	GPIOB_BSRR = x_bits[msx_Y_scan]; //Atomic GPIOB update => Release and press MSX keys for this column

	//Debug & performance measurement
	gpio_set(Dbg_Yint_port, Dbg_Yint2e3_PIN); //Signs end of interruption. Default condition is "1"
    
	// Clear interrupt Y Scan flags, including those not used on this ISR
	// if(exti_get_flag_status(EXTI7), (EXTI6), (EXTI4) and (EXTI3))
	exti_reset_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
    
	//Update systicks (time stamp) for this Y
	previous_y_systick[msx_Y_scan]  = systicks;

}*/
#endif	//#if MCU == STM32F103
