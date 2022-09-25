/** @addtogroup 04 ps2handl PS/2 Keyboard Interface Handler
 *
 * @file ps2handl.c Power Control of a PS/2 key and general interface to and from PS/2 keyboard, inlcuding interrupt routines.
 *
 * @brief <b>Power Control of a PS/2 key and general interface to and from PS/2 keyboard, inlcuding interrupt routines.</b>
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
 * This file is part of the PS/2 to MSX keyboard Converter and 
 * MSX Keyboard Subsystem Emulator projects, using libopencm3 project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
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


#include "ps2handl.h"


//PS/2 keyboard iteration constants
#define COMM_TYPE3_NO_REPEAT							0xF8	//248 Type 3 command
#define COMM_READ_ID											0xF2	//242
#define COMM_SET_TYPEMATIC_RATEDELAY			0xF3	//243 Type 2 command
#define COMM_ENABLE												0xf4	//244
#define COMM_SET_RESET_LEDS								0xED	//237
#define COMM_ECHO													0xEE	//238
#define KBCOMM_RESEND											0xFE	//254
#define ARG_NO_ARG												0xFF	//255
#define ARG_LOWRATE_LOWDELAY							0x7F	//Type 2 (Delay 1 second to repeat. 2cps repeat rate)
#define KB_ACKNOWLEDGE										0xFA
#define KB_FIRST_ID												0xAB
#define KB_SECOND_ID											0x83
#define KB_SUCCESSFULL_BAT								0xAA
#define KB_ERROR_BAT											0xFC


//Global Vars
volatile uint16_t ps2int_state;
volatile uint8_t ps2int_TX_bit_idx;
volatile uint8_t ps2int_RX_bit_idx;
extern uint16_t state_overflow_tim2;							//Declared on hr_timer_delay.c
extern uint64_t TIM2_Update_Cnt;									//Declared on hr_timer_delay.c Overflow of time_between_ps2clk

volatile uint8_t command, argument;

volatile uint32_t prev_systicks;
volatile uint32_t ps2int_prev_systicks;
extern uint32_t systicks;													//Declared on msxhid.cpp
extern uint64_t acctimeps2data0;									//Declared on hr_timer_delay.c
extern  uint32_t formerscancode;									//declared on msxmap.cpp
extern uint8_t scancode[4];												//declared on msxmap.cpp
extern uint64_t time_between_ps2clk;							//Declared on hr_timer_delay.c
extern uint16_t fail_count;												//declared on msxhid.cpp
volatile bool formerps2datapin, update_ps2_leds;
volatile bool ps2_keyb_detected, ps2numlockstate;
volatile bool command_ok, echo_received;
bool caps_state, kana_state;
bool caps_former, kana_former;

volatile bool mount_scancode_OK;									//used on mount_scancode()
volatile bool ps2_keystr_e0 = false;
volatile bool ps2_keystr_e1 = false;
volatile bool ps2_keystr_f0 = false;
volatile uint8_t ps2_byte_received;
volatile uint8_t mount_scancode_count_status = 0;

//Need to stay as global to avoid creating different instancies
volatile uint8_t ps2_recv_buffer[PS2_RECV_BUFFER_SIZE];
volatile uint8_t ps2_recv_put_ptr;
volatile uint8_t ps2_recv_get_ptr;

//Local prototypes (not declared in ps2handl.h)
void init_ps2_recv_buffer(void);
bool available_ps2_byte(void);
uint8_t get_ps2_byte(volatile uint8_t*);
void send_start_bit_next(uint16_t);
void ps2_clock_send(bool);
void ps2_clock_receive(bool);
void ps2_send_command(uint8_t, uint8_t);
void reset_mount_scancode_machine(void);
void send_start_bit_now(void);
void send_start_bit2(void);
void send_start_bit3(void);

extern char _ebss[];


//Power on PS/2 Keyboard and related pins setup
void power_on_ps2_keyboard()
{
	//PS/2 power control pin
#if MCU == STM32F103
	gpio_set_mode(PS2_POWER_CTR_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, PS2_POWER_CTR_PIN);
#endif
#if MCU == STM32F401
	gpio_set_af(PS2_POWER_CTR_PORT, GPIO_AF1, PS2_POWER_CTR_PIN);	//Set Alternate function
	gpio_mode_setup(PS2_POWER_CTR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PS2_POWER_CTR_PIN);
	gpio_set_output_options(PS2_POWER_CTR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PS2_POWER_CTR_PIN);
#endif

	gpio_set(PS2_POWER_CTR_PORT, PS2_POWER_CTR_PIN);	//Turn on PS/2 Keyboard

	// PS/2 keyboard Clock and Data pins
#if MCU == STM32F103
	gpio_set(PS2_CLOCK_PORT, PS2_CLOCK_PIN); //Hi-Z
	gpio_set_mode(PS2_CLOCK_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, PS2_CLOCK_PIN);
	gpio_port_config_lock(PS2_CLOCK_PORT, PS2_CLOCK_PIN);
#endif
#if MCU == STM32F401
	//gpio_set_af(PS2_CLOCK_PORT, GPIO_AF1, PS2_CLOCK_PIN);	//Set Alternate function
	gpio_set(PS2_CLOCK_PORT, PS2_CLOCK_PIN); //Hi-Z
	gpio_set_output_options(PS2_CLOCK_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, PS2_CLOCK_PIN);
	gpio_mode_setup(PS2_CLOCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PS2_CLOCK_PIN);
#if PS2_CLK_INTERRUPT == GPIO_INT
	exti_select_source(PS2_CLOCK_EXTI, PS2_CLOCK_PORT);
	exti_set_trigger(PS2_CLOCK_EXTI, EXTI_TRIGGER_FALLING);
	exti_reset_request(PS2_CLOCK_EXTI);
	exti_enable_request(PS2_CLOCK_EXTI);
#endif //#if PS2_CLK_INTERRUPT == GPIO_INT
	gpio_port_config_lock(PS2_CLOCK_PORT, PS2_CLOCK_PIN);
#endif

	// PS/2 keyboard Data pin
#if MCU == STM32F103
	gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);											//Hi-Z
	gpio_set_mode(PS2_DATA_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, PS2_DATA_PIN);
	gpio_port_config_lock(PS2_DATA_PORT, PS2_DATA_PIN);
#endif
#if MCU == STM32F401
	gpio_set_af(PS2_DATA_PORT, GPIO_AF1, PS2_DATA_PIN);	//Set Alternate function
	gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);		//Hi-Z
	gpio_mode_setup(PS2_DATA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PS2_DATA_PIN);
	gpio_set_output_options(PS2_DATA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, PS2_DATA_PIN);
	gpio_port_config_lock(PS2_DATA_PORT, PS2_DATA_PIN);
#endif

#if PS2_CLK_INTERRUPT == GPIO_INT
	// Enable EXTI15_10 interrupt.
	nvic_enable_irq(IRQ_PRI_TIM_HR);

	//High priority to avoid PS/2 interrupt loss
	nvic_set_priority(NVIC_EXTI15_10_IRQ, IRQ_PRI_TIM_HR);
#endif //#if PS2_CLK_INTERRUPT == GPIO_INT

	//Starts with RX state: PS2INT_RECEIVE
	ps2int_state = PS2INT_RECEIVE;
	ps2int_RX_bit_idx = 0;
	command_ok = true;
	
	init_ps2_recv_buffer();
}


bool keyboard_check_alive(void)
{
	uint32_t systicks_start_command;	//Initial time mark

	while(ps2int_RX_bit_idx != 0) __asm("nop");

	systicks_start_command = systicks;
	ps2_send_command(COMM_ENABLE, ARG_NO_ARG); //Enable command.
	while (!command_ok && (systicks - systicks_start_command) < 2) __asm("nop"); //Must be excecuted in less than 67ms
	if(!command_ok)
	{
		return false;
	}

	systicks_start_command = systicks;
	echo_received = false;
	ps2_send_command(COMM_ECHO, ARG_NO_ARG); //Echo command.
	while (!command_ok && (systicks - systicks_start_command) < 3) __asm("nop"); //Must be excecuted in less than 67ms
	if(command_ok)
	{
		return echo_received;
	}
	return false;
}


void power_off_ps2_keyboard()
{
	gpio_clear(PS2_POWER_CTR_PORT, PS2_POWER_CTR_PIN);
	con_send_string((uint8_t*)"\r\nPS/2 interface powered down.\r\n\n");
}


// Initialize receive ringbuffer
void init_ps2_recv_buffer()
{
	uint8_t i;

	formerscancode = 0;
	ps2_recv_put_ptr=0;
	ps2_recv_get_ptr=0;
	for(i = 0; i < PS2_RECV_BUFFER_SIZE; ++i)
	{
		ps2_recv_buffer[i]=0;
	}
}

// Verify if there is an available ps2_byte_received on the receive ring buffer, but does not fetch this one
bool available_ps2_byte()
{
	uint8_t i;

	i = ps2_recv_get_ptr;
	if(i == ps2_recv_put_ptr)
		//No char in buffer
		return false;
	else
		return true;
}

// Fetches the next ps2_byte_received from the receive ring buffer
uint8_t get_ps2_byte(volatile uint8_t *buff)
{
	uint8_t i, result;

	i = ps2_recv_get_ptr;
	if(i == ps2_recv_put_ptr)
		//No char in buffer
		return 0;
	result = buff[i];
	i++;
	ps2_recv_get_ptr = i & (uint16_t)(PS2_RECV_BUFFER_SIZE - 1); //if(i >= (uint16_t)SERIAL_RING_BUFFER_SIZE)	i = 0;
	return result;
}


bool ps2_keyb_detect(void)
{
	uint32_t systicks_start_command;	//Initial time mark
	uint8_t mountstring[16];					//Used in con_send_string()
	uint8_t ch;
	//uint16_t qty_in_buffer;
	/*serial_wait_tx_ends();*/

	//Clean RX serial buffer to not false glitch database_setup
	while (con_available_get_char())
		ch = con_get_char();
	ch&= 0xFF;	//only to avoid unused variable warning.

	//Wait for 2.5s to keyboard execute its own power on and BAT (Basic Assurance Test) procedure
	systicks_start_command = systicks;
	prev_systicks = systicks;
	ps2_keyb_detected = false;
	uint16_t localcount = 0;
	while ( ((systicks-systicks_start_command) < (25*3)) && (!available_ps2_byte()) )	//Wait 2500ms for keyboard power on
	{
		if(prev_systicks != systicks)
		{//systicks updated. Each 15 ticks is a "#"  |.....|
			if((systicks - systicks_start_command - localcount) > 14)
			{
				localcount = systicks - systicks_start_command;
				con_send_string((uint8_t*)"\r  to proceed BAT: |");
				for(uint16_t i = 0; i < (uint8_t)(localcount / 15); i++)
					con_send_string((uint8_t*)"#");
			}
		}
		prev_systicks = systicks;	//To avoid errors on keyboard power up BEFORE the first access
	}
	//Fill bar graph
	localcount = systicks - systicks_start_command;
	con_send_string((uint8_t*)"\r  to proceed BAT: |");
	for(uint16_t i = 0; i < (uint8_t)(localcount / 15); i++)
		con_send_string((uint8_t*)"#");

	if ((systicks-systicks_start_command) >= (25*3))
	{
		//User messages
 		con_send_string((uint8_t*)"\r\n..  Timeout on BAT: No keyboard!\r\n");
		return ps2_keyb_detected;
	}
	//PS/2 keyboard might already sent its BAT result. Check it:
	ps2_byte_received = get_ps2_byte(&ps2_recv_buffer[0]);
	if(ps2_byte_received != 0)
	{
		if(ps2_byte_received == KB_SUCCESSFULL_BAT)
		{
			//User messages
			con_send_string((uint8_t*)"\r\n..  BAT (Basic Assurance Test) OK in ");
			conv_uint32_to_dec((prev_systicks - systicks_start_command), &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)" ticks;\r\n");
		}
		else
		{
			//User messages
			con_send_string((uint8_t*)"..  BAT not OK: Received 0x");
			conv_uint8_to_2a_hex(ps2_byte_received, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)" instead of 0xAA\r\n");
		}
	}
	
	//Send command Read ID. It musts responds with 0xFA (implicit), 0xAB, 0x83
	// Wait clock line to be unactive for 100 ms (3 systicks)
	//con_send_string((uint8_t*)"Sending Read ID comm\r\n");
	systicks_start_command = systicks;
	ps2_send_command(COMM_READ_ID, ARG_NO_ARG); //Read ID command.
	while (!command_ok && (systicks - systicks_start_command)<(3*3)) //Must be excecuted in less than 100ms
	{
		prev_systicks = systicks;	//To avoid errors on keyboard power up BEFORE the first access
		if(systicks != systicks_start_command)
			prev_systicks = systicks;	//To avoid errors on keyboard power up BEFORE the first access
	}
	if (command_ok)
	{
		//con_send_string((uint8_t*)"Waiting 0xAB\r\n");
		systicks_start_command = systicks;
		while(!available_ps2_byte()&& (systicks - systicks_start_command)<(1*3))
    __asm("nop");
		ps2_byte_received = get_ps2_byte(&ps2_recv_buffer[0]);
		if(ps2_byte_received == KB_FIRST_ID)
		{
			//con_send_string((uint8_t*)"Waiting 0x83\r\n");
			systicks_start_command = systicks;
			while(!available_ps2_byte() && (systicks - systicks_start_command)<(1*3))
			__asm("nop");
			ps2_byte_received = get_ps2_byte(&ps2_recv_buffer[0]);
			if(ps2_byte_received == KB_SECOND_ID)
			{
				ps2_keyb_detected = true;
				//User messages
				con_send_string((uint8_t*)"..  PS/2 Keyboard detected;\r\n");
			}
			else
			{
				//con_send_string((uint8_t*)"Did not receive 0x83");
				con_send_string((uint8_t*)"..  PS/2 Keyboard not detected!\r\n");
				return ps2_keyb_detected;
			}
		}
		else
		{
			//con_send_string((uint8_t*)"Did not receive 0xAB");
			return ps2_keyb_detected;
		}
	}
	else
	{
		//User messages
		/*con_send_string((uint8_t*)"PS/2 ReadID command not OK. Elapsed time was ");
		conv_uint32_to_8a_hex((systicks - systicks_start_command), &mountstring[0]);
		con_send_string((uint8_t*)&mountstring[0]);
		con_send_string((uint8_t*)"\r\n"); */
		return ps2_keyb_detected;
	}

	//The objective of this block is to minimize the keyboard interruptions, to keep time to high priority MSX interrupts.
	//Send type 3 command 0xFA (Set Key Type Make/Break - This one only disables typematic repeat):
	//  If it does not receive "ack" (0xFA), then send type 2 command 0xF3 + 0x7F (2cps repeat rate + 1 second delay)
	//  It musts respond with an "ack" after the first byte, than with a second "ack" after the second byte.
	//User messages
	//con_send_string((uint8_t*)"Type 2 sets typematic repeat 0xF3 0x7F requested\r\n");
	
	//Type 2 command: Set typematic rate to 2 cps and delay to 1 second.
	systicks_start_command = systicks;
	ps2_send_command(COMM_SET_TYPEMATIC_RATEDELAY, ARG_LOWRATE_LOWDELAY);
	while (!command_ok && (systicks - systicks_start_command)<(2*3)) //Must be excecuted in less than 200ms
		__asm("nop");
	if (command_ok)
		//User messages
		con_send_string((uint8_t*)"..  Delay 1 second to repeat, 2cps repeat rate (Type 2 command) OK;\r\n");
	else
	{
		//User messages
		con_send_string((uint8_t*)"..  Type 3 Disables typematic repeat 0xFA requested\r\n");

		//.1 second delay (to display serial contents) ONLY TO DEBUG
		systicks_start_command = systicks;
		while ((systicks - systicks_start_command)<(1*3))
			__asm("nop");

		systicks_start_command = systicks;
		//Type 3 command: Set All Keys Make/Break: This one only disables typematic repeat and applies to all keys
		ps2_send_command(COMM_TYPE3_NO_REPEAT, ARG_NO_ARG);
		while (!command_ok && (systicks - systicks_start_command)<(1*3)) //Must be excecuted in less than 100ms
			__asm("nop");
		if (command_ok)
			//User messages
			con_send_string((uint8_t*)"..  Type 3 Disables typematic 0xFA repeat OK\r\n");
	}
	return ps2_keyb_detected;
}


void reset_requested(void)
{
	power_off_ps2_keyboard();
	//Wait here 1/3 second to consolidate this power off
	uint32_t readsysticks = systicks;
	//wait 1/3 second
	while (systicks <= (readsysticks + 10)) __asm("nop");
	systick_interrupt_disable();
#if MCU == STM32F401
	uint32_t *magic = (uint32_t *)&_ebss;
	magic[0] = BOOTMAGIC0;
	magic[1] = BOOTMAGIC1;
#endif	//#if MCU ==STM32F401
	scb_reset_system();
	for(;;) {};//Wait here until reset
}

/*************************************************************************************************/
/******************************  Support to other ISR's ******************************************/
/*************************************************************************************************/

void ps2_send_command(uint8_t cmd, uint8_t argm)
{
	uint8_t mountstring[16];					//Used in con_send_string()
	/*uint32_t systicks_start_command;	//Initial time mark

	systicks_start_command = systicks;
	while(!command_ok && ((systicks_start_command - systicks) < 2) )
		__asm("nop");	*/
	if(ps2int_state != PS2INT_RECEIVE)
	{
		//User messages
		con_send_string((uint8_t*)"0x");
		conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		con_send_string((uint8_t*)&mountstring[0]);
		con_send_string((uint8_t*)", instead of 0x0400\r\n");
	}
	command =  cmd;
	argument = argm;
	ps2int_state = PS2INT_SEND_COMMAND;
	send_start_bit_now();
}


void ps2_update_leds(bool num, bool caps, bool scroll)
{
	uint8_t mountstring[16];					//Used in con_send_string()

	if(ps2int_state != PS2INT_RECEIVE)
	{
		//User messages
		con_send_string((uint8_t*)"0x");
		conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		con_send_string((uint8_t*)&mountstring[0]);
		con_send_string((uint8_t*)", instead of 0x0400\r\n");
	}
	command = COMM_SET_RESET_LEDS;
	argument = (scroll<<0)|(num<<1)|(caps<<2);
	ps2int_state = PS2INT_SEND_COMMAND;
	send_start_bit_now();
}


//Insert a delay before run send_start_bit_now()
void send_start_bit_next(uint16_t x_usec)
{
	delay_usec(TIM_HR, x_usec, send_start_bit_now); //wait x_usec and go to send_start_bit on TIM_HR_TIMER overflow interrupt
}


//This three functions are the split of Transmit Initiator, to avoid stuck inside an interrupt due to 120u and 20usec
void send_start_bit_now(void)
{
	timer_disable_irq(TIM_HR, TIM_DIER_CC1IE);	// Disable interrupt on Capture/Compare1, but keeps on overflow
#if PS2_CLK_INTERRUPT == GPIO_INT
	exti_disable_request(PS2_CLOCK_EXTI);
#endif	
	command_ok = false; //Here command_OK is initialized
	gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);
	gpio_clear(PS2_CLOCK_PORT, PS2_CLOCK_PIN);
	//if keyboard interrupt was not disabled, it would be interrupted here, pointing to something not coded
	//Something was wrong with original delay, so I decided to use TIM_HR_TIMER Capture/Compare interrupt
	// See hr_timer_delay.c file
	//now insert a 120us delay and run step 2 of send_start_bit function
	delay_usec(TIM_HR, 120, send_start_bit2); //wait 120usec and go to send_start_bit2 on TIM_HR_TIMER overflow interrupt
}


void send_start_bit2(void) //Second part of send_start_bit
{
	gpio_clear(PS2_DATA_PORT, PS2_DATA_PIN); //this is the start bit
	//now insert a 10us delay and run step 3 of send_start_bit function
	delay_usec(TIM_HR, 10, send_start_bit3); /*wait 10usec and go to send_start_bit3 on TIM_HR_TIMER overflow interrupt*/
}


void send_start_bit3(void) //Third part of send_start_bit
{
	ps2int_prev_systicks = systicks;
	//Rise clock edge starts the PS/2 device to receive command/argument
	gpio_set(PS2_CLOCK_PORT, PS2_CLOCK_PIN);

	ps2int_TX_bit_idx = 0;	// In TX Int, as we don't manage start bit inside int, idx can start with 0.

	prepares_capture(TIM_HR);
#if PS2_CLK_INTERRUPT == GPIO_INT
	exti_reset_request(PS2_CLOCK_EXTI);
	exti_enable_request(PS2_CLOCK_EXTI);
#endif	
}


/*****************  Excecution of bitbang Support to other ISR's **********************/

/*  Enter point of PS/2 clock line, called from interrupt handled by msxhid  */
/*  Here is decided if the int is going to be treated as send or receive*/
void ps2_clock_update(bool ps2datapin_logicstate)
{
	if (!ps2datapin_logicstate && (ps2datapin_logicstate == formerps2datapin))
	{
		//State low is repeated
		acctimeps2data0 += time_between_ps2clk;
		if (acctimeps2data0 >= 200000) //.2s
		{
			gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);
			ps2int_state = PS2INT_RECEIVE;
			ps2int_RX_bit_idx = 0;
		}
	}
	else
		acctimeps2data0 = 0;		//Reset acc time counter
	formerps2datapin = ps2datapin_logicstate;	//To compare at next bit
	uint8_t mountstring[16]; //Used in con_send_string()
	/*Any keyboard interrupt that comes after 900 micro seconds means an error condition,
	but I`m considering it as an error for about 100 ms, to acommodate this to power on, to answer 
	to Read ID command. I observed this behavior on my own PS/2 keyboards. It is huge!*/
	if( ((ps2int_state == PS2INT_SEND_COMMAND) || (ps2int_state == PS2INT_SEND_ARGUMENT))
				&& ((ps2int_TX_bit_idx != 0) && (systicks - ps2int_prev_systicks) > 1) )
	{	//reset to PS/2 receive condition
		gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);
		//User messages (debug)
		con_send_string((uint8_t*)"ps2_clock_sent reseted - Timeout = ");
		conv_uint32_to_dec((systicks - ps2int_prev_systicks), &mountstring[0]);
		con_send_string((uint8_t*)&mountstring[0]);
		con_send_string((uint8_t*)", ps2int_TX_bit_idx = ");
		conv_uint32_to_dec((uint32_t)ps2int_TX_bit_idx, &mountstring[0]);
		con_send_string((uint8_t*)&mountstring[0]);
		con_send_string((uint8_t*)", ");
		conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		con_send_string((uint8_t*)&mountstring[0]);
		con_send_string((uint8_t*)";\r\n");
		ps2int_state = PS2INT_RECEIVE;
		ps2int_RX_bit_idx = 0;
	}
	
	if( (ps2int_state == PS2INT_SEND_COMMAND) || (ps2int_state == PS2INT_SEND_ARGUMENT) )
	{
		ps2_clock_send(ps2datapin_logicstate);
	}
	else
	{
		ps2_clock_receive(ps2datapin_logicstate);
	}
}


void ps2_clock_send(bool ps2datapin_logicstate)
{
	uint8_t mountstring[16]; //Used in con_send_string()
	ps2int_prev_systicks = systicks;
	//Time check - The same for all bits
	if (time_between_ps2clk > 10000) // time >10ms
	{
		con_send_string((uint8_t*)"Time > 10ms on TX");
	}
	//|variável| = `if`(condição) ? <valor1 se true> : <valor2 se false>;:
	//Only two TX states of send: ps2_send_command & send_argument
	uint8_t data_byte = (ps2int_state == PS2INT_SEND_COMMAND) ? command : argument;
	//if( (ps2int_TX_bit_idx >= 0) && (ps2int_TX_bit_idx < 8) )	//The first test will be always true
	if(ps2int_TX_bit_idx < 8)
	{
		bool bit = data_byte & (1 << (ps2int_TX_bit_idx));
		ps2int_TX_bit_idx++;
			//User messages (debug)
			/*con_send_string((uint8_t*)"sent bit #");
			conv_uint32_to_dec((uint32_t)ps2int_TX_bit_idx-1, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)": ");*/
		if(bit)
		{
			gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);
			//User messages (debug)
			/*con_send_string((uint8_t*)"1\r\n");*/
		}
		else
		{
			gpio_clear(PS2_DATA_PORT, PS2_DATA_PIN);
			//User messages (debug)
			/*con_send_string((uint8_t*)"0\r\n");*/
		}
	}
	else if(ps2int_TX_bit_idx == 8)
	{//parity
		bool parity =! __builtin_parity(data_byte);
		//User messages (debug)
		//con_send_string((uint8_t*)"sent p: "); //This print continues below
		if(parity)
		{
			gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);
			//User messages (debug)
			//con_send_string((uint8_t*)"1\r\n");
		}
		else
		{
			gpio_clear(PS2_DATA_PORT, PS2_DATA_PIN);
			//User messages (debug)
			//con_send_string((uint8_t*)"0\r\n");
		}
		ps2int_TX_bit_idx = 9;
	}
	else if(ps2int_TX_bit_idx == 9)
	{//stop bit
		gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);
		ps2int_TX_bit_idx = 10;
		//User messages (debug)
		//con_send_string((uint8_t*)"sent stop\r\n");
	}
	else if(ps2int_TX_bit_idx >= 10)
	{
		if(ps2datapin_logicstate == false)
		{
			//  ACK bit ok
			//User messages (debug)
			/*con_send_string((uint8_t*)"TX Data sent OK: 0x");
			conv_uint8_to_2a_hex(data_byte, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)", 0x");
			conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)"\r\n");*/
			
		}
		else
		{
			// Ack bit NOT ok
			gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);	//To warranty that is not caused by let this pin LOW
			//User messages (debug)
			con_send_string((uint8_t*)"Trying to send 0x");
			conv_uint8_to_2a_hex(data_byte, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)". ACK bit not received from keyboard\r\n");
			ps2int_state = PS2INT_RECEIVE; //force to receive_status in absence of something better
			ps2int_RX_bit_idx = 0;
		}

		if(ps2int_state == PS2INT_SEND_COMMAND)
		{
			//Although the original logic pointed state=PS2INT_RECEIVE,
			//after send the commmand, you have to wait the Acknowlodge of the PS/2 command.
			//It is fixed here.
			if(command != COMM_ECHO)
			{
				ps2int_state = PS2INT_WAIT_FOR_COMMAND_ACK;
				ps2int_RX_bit_idx =  0;
				//User messages (debug)
				/*con_send_string((uint8_t*)"TX: new 0x");
				conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
				con_send_string((uint8_t*)&mountstring[0]);
				con_send_string((uint8_t*)"\r\n");*/
			}
			else
			{	//New state created to acomodate waiting for echo
				ps2int_state = PS2INT_WAIT_FOR_ECHO;
				ps2int_RX_bit_idx =  0;
			}
		}
		else if(ps2int_state == PS2INT_SEND_ARGUMENT)
		{
			ps2int_state = PS2INT_WAIT_FOR_ARGUMENT_ACK;
			ps2int_RX_bit_idx =  0;
			//User messages (debug)
			/*con_send_string((uint8_t*)"TX: new 0x");
			conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)"\r\n");*/
		}
		else
		{
			gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);
			ps2int_state = PS2INT_RECEIVE;
			ps2int_RX_bit_idx =  0;
			//User messages (debug)
			/*con_send_string((uint8_t*)"TX: new 0x");
			conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)"\r\n");*/
		}
	}
}


void ps2_clock_receive(bool ps2datapin_logicstate)
{
	static uint8_t data_word, stop_bit, parity_bit;
	uint8_t mountstring[16]; //Used in con_send_string()

	//Verify RX timeout, that is quite restricted, if compared to Send Command/Argument
	if ( (ps2int_RX_bit_idx != 0) && (time_between_ps2clk > 120) )  //because if RX_bit_idx == 0 will be the reset
	{	
		con_send_string((uint8_t*)"ps2_clock_receive - Timeout\r\n");
		ps2int_RX_bit_idx = 0;
	}
	ps2int_prev_systicks = systicks;

	if(ps2int_RX_bit_idx == 0)
	{
		//Force this interface to put data line in Hi-Z to avoid unspected behavior in case of errors
		gpio_set(PS2_DATA_PORT, PS2_DATA_PIN);
		//User messages (debug)
		//con_send_string((uint8_t*)"RX: ps2int_state = 0x");
		//conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		//con_send_string((uint8_t*)&mountstring[0]);
		//con_send_string((uint8_t*)"\r\n");
		data_word = 0;
		stop_bit = 0xff;
		parity_bit = 0xff;
		ps2int_RX_bit_idx = 1; //Points to the next bit
		bool start_bit = ps2datapin_logicstate;
		if(start_bit)
		{
			fail_count++;
  		ps2int_RX_bit_idx = 0; //reset
		}
	}
	else if( /*(ps2int_RX_bit_idx>0) && */(ps2int_RX_bit_idx<9) ) // collect bits 1 to 8 (D0 to D7)
	{
		data_word |= (ps2datapin_logicstate << (ps2int_RX_bit_idx - 1));
		ps2int_RX_bit_idx++;
	}
	else if(ps2int_RX_bit_idx == 9)
	{
		parity_bit = ps2datapin_logicstate;
		ps2int_RX_bit_idx++;
	}
	else if(ps2int_RX_bit_idx == 10)
	{	 // start + 8 + stop + parity (but started with 0)
		ps2int_RX_bit_idx = 0;	//next (reset) PS/2 receive condition

		stop_bit = ps2datapin_logicstate;
		bool parity_ok = __builtin_parity((data_word<<1)|parity_bit);
		//User messages (debug)
		/*con_send_string((uint8_t*)"RX Data: 0x");
		conv_uint8_to_2a_hex(data_word, &mountstring[0]);
		con_send_string((uint8_t*)&mountstring[0]);
		if(parity_ok)
			con_send_string((uint8_t*)", pbit OK,");
		else
			con_send_string((uint8_t*)", pbit issue,");
		if(stop_bit == 1)
			con_send_string((uint8_t*)" sbit OK,");
		else
			con_send_string((uint8_t*)" sbit issue,");
		con_send_string((uint8_t*)" 0x");
		conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		con_send_string((uint8_t*)&mountstring[0]);
		con_send_string((uint8_t*)"\r\n");*/

		if(parity_ok && (stop_bit == 1) ) //start bit condition was already tested above
		{	// ps2int_status receive procesing block (begin)
			if (ps2int_state == PS2INT_RECEIVE)
			{
				//this put routine is new
				uint8_t i = ps2_recv_put_ptr;
				uint8_t i_next = (i + 1) & (uint8_t)(PS2_RECV_BUFFER_SIZE - 1);
				if (i_next != ps2_recv_get_ptr)
				{
					ps2_recv_buffer[i] = data_word;
					ps2_recv_put_ptr = i_next;
				}
			}

			else if (ps2int_state == PS2INT_WAIT_FOR_COMMAND_ACK)
			{
				if(data_word == KB_ACKNOWLEDGE) //0xFA is Acknowledge from PS/2 keyboard
				{
					if(argument == ARG_NO_ARG) //0xFF is an empty argument
					{
						if(command == COMM_ECHO)
						{
							//Wait for ECHO
							ps2int_state = PS2INT_WAIT_FOR_ECHO;
							ps2int_RX_bit_idx = 0;//reset PS/2 receive condition
							command_ok = true;
						}
						else
						{
							//no argument: set to receive
							ps2int_state = PS2INT_RECEIVE;
							ps2int_RX_bit_idx = 0;//reset PS/2 receive condition
							command_ok = true;
						}
					}
					else
					{
						//argument is not empty (!=ARG_NO_ARG). Send it
						ps2int_state = PS2INT_SEND_ARGUMENT;
						send_start_bit_next(150);
					}
				}

				else if(data_word == KBCOMM_RESEND) //0xFE is Resend
				{
					ps2int_state = PS2INT_SEND_COMMAND;
					send_start_bit_next(150); //Send BOTH command and argument
				}	//if(data_word==KBCOMM_RESEND) //0xFE is Resend
				else
				{
					//User messages (debug)
					con_send_string((uint8_t*)"Got unexpected command response: 0x");
					conv_uint8_to_2a_hex(data_word, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[0]);
					con_send_string((uint8_t*)"\r\n");
					ps2int_state = PS2INT_RECEIVE;
					ps2int_RX_bit_idx = 0;//reset PS/2 receive condition
					fail_count++;
				}	//else if(data_word==KBCOMM_RESEND) //0xFE is Resend
			}

			else if (ps2int_state == PS2INT_WAIT_FOR_ARGUMENT_ACK)
			{
				if(data_word == KB_ACKNOWLEDGE) //Acknowledge from PS/2 keyboard
				{
					//Acknowledge received => set to receive
					ps2int_state = PS2INT_RECEIVE;
					ps2int_RX_bit_idx = 0;//Prepares for the next PS/2 receive condition
					command_ok = true;
				}
				else if(data_word == KBCOMM_RESEND) //0xFE is Resend
				{
					ps2int_state = PS2INT_SEND_COMMAND; //Resend BOTH command AND argument
					send_start_bit_next(150);
				}
				else
				{
					ps2int_state = PS2INT_RECEIVE;
					ps2int_RX_bit_idx = 0;//reset PS/2 receive condition
					//User messages (debug)
					con_send_string((uint8_t*)"Got unexpected command response: 0x");
					conv_uint8_to_2a_hex(data_word, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[0]);
					con_send_string((uint8_t*)"\r\n");
				}
			}		// ps2int_status receive procesing block (end)

			else if (ps2int_state == PS2INT_WAIT_FOR_ECHO)
			{
				if(data_word == COMM_ECHO) //Echo received from PS/2 keyboard
				{
					//Echo received => set to receive
					ps2int_state = PS2INT_RECEIVE;
					ps2int_RX_bit_idx = 0;//Prepares for the next PS/2 receive condition
					command_ok = true;
					echo_received = true;
				}
				else //if(data_word == COMM_ECHO)
				{	//data_word != COMM_ECHO (0xEE)
					echo_received = false;
					//User messages (debug)
					con_send_string((uint8_t*)"In 0x");
					conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[0]);
					con_send_string((uint8_t*)", received 0x");
					conv_uint8_to_2a_hex(data_word, &mountstring[0]);
					con_send_string((uint8_t*)&mountstring[0]);
					con_send_string((uint8_t*)" instead of COMM_ECHO (0xEE)\r\n");
				}	 //if(data_word == COMM_ECHO)
			}	//else if (ps2int_state == PS2INT_WAIT_FOR_ECHO)

		}	//if(start_bit==0 && stop_bit==1 && parity_ok)
		else
		{
			//User messages (debug)
			con_send_string((uint8_t*)"Framming Error. RX Data: 0x");
			conv_uint8_to_2a_hex(data_word, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)", parity ");
			mountstring[0] = parity_bit ? '1' : '0';
			mountstring[1] = 0;
			con_send_string((uint8_t*)mountstring);
			con_send_string((uint8_t*)", Stop ");
			mountstring[0] = stop_bit ? '1' : '0';
			mountstring[1] = 0;
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)"\r\n");
			fail_count++;
			ps2int_RX_bit_idx = 0;
		}
	}	//else if(ps2int_RX_bit_idx==10)
}


void reset_mount_scancode_machine()
{
	mount_scancode_count_status = 0;
	ps2_keystr_e0 = false;
	ps2_keystr_e1 = false;
	ps2_keystr_f0 = false;
}


bool mount_scancode()
{
	//Check MSX CAPS and Kana status update
	if( (caps_state = gpio_get(CAPSLOCK_PORT, CAPSLOCK_PIN)) != caps_former )
		update_ps2_leds = true;
	if( (kana_state = gpio_get(KANA_PORT, KANA_PIN)) != kana_former )
		update_ps2_leds = true;
	// static uint16_t prev_state_index=0;
	if (!mount_scancode_OK)
	{	
		while(available_ps2_byte() && !mount_scancode_OK)
		{
			ps2_byte_received = get_ps2_byte(&ps2_recv_buffer[0]);
			//User messages (debug)
			/*con_send_string((uint8_t*)"Mount_scancode RX Ch=");
			conv_uint8_to_2a_hex(ps2_byte_received, &mountstring[0]);
			con_send_string((uint8_t*)&mountstring[0]);
			con_send_string((uint8_t*)"\r\n"); */
			switch (mount_scancode_count_status)
			{
			case 0:	//Está lendo o primeiro byte do ps2_byte_received
			{
				if((ps2_byte_received > 0) && (ps2_byte_received < 0xE0)) //Se até 0xDF cai aqui
				{
					//Concluded. 1 byte only scan code.
					scancode[1] = ps2_byte_received;
					//Conclui scan
					scancode[0] = 1;
					mount_scancode_OK = true;
					reset_mount_scancode_machine();
					return true;
				}
				if(ps2_byte_received == 0xE0)
				{
					//0xE0 + (Any != 0xF0) <= 2 bytes
					//0xE0 + 0xF0 + (Any)  <= 3 bytes
					ps2_keystr_e0 = true;
					scancode[1] = ps2_byte_received;
					scancode[0] = 1;
					mount_scancode_count_status = 1; //points to next case
					break;
				}
				if(ps2_byte_received == 0xE1)
				{
					// Pause/Break key: 8 bytes (0xE1 + 7 bytes). Store only the 3 first bytes and discard the others
					ps2_keystr_e1 = true;
					scancode[1] = ps2_byte_received;
					scancode[0] = 1;
					mount_scancode_count_status = 1; //points to next case
					break;
				}
				if(ps2_byte_received == 0xF0)
				{
					//Always 2 bytes 0xF0 + (Any)
					ps2_keystr_f0 = true;
					scancode[1] = ps2_byte_received;
					scancode[0] = 1;
					mount_scancode_count_status = 1; //points to next case
					break;
				}
 				break;	//case syntax suggested
			}	//case 0:
			case 1:
			{
				if (ps2_keystr_e0)
				{
					if(ps2_byte_received != 0xF0)
					{
						if(ps2_byte_received != 0x12)
						{
							//2 bytes and this ps2_byte_received is != 0xF0 e != 0x12, so, concluded
							scancode[2] = ps2_byte_received;
							scancode[0] = 2;
							//task concluded
							mount_scancode_OK = true;
							reset_mount_scancode_machine();
							return true;
						}	//if(ps2_byte_received != 0x12)
						else
						{
							//Discard E0 12 here
							reset_mount_scancode_machine();
							break;
						}	//if(ps2_byte_received != 0x12)
					}	//if(ps2_byte_received != 0xF0)
					else //if(ps2_byte_received == 0xF0)
					{
							//3 bytes, but I'm reading the second one (E0 F0)
							scancode[2] = ps2_byte_received;
							scancode[0] = 2;
							mount_scancode_count_status = 2; //points to next case
							break;
					}	//if(ps2_byte_received == 0xF0)
				}
				if (ps2_keystr_e1)  //Break key (8 bytes)
				{
					//8 bytes: Two groups of 3 and a 2 bytes one: (E1, 14, 77; E1, F0, 14; F0, 77).
					//The group (F0, 77) is interpreted as a Num Lock break code: Harmless.
					//Ler apenas os 3 iniciais e desconsiderar os demais. Estou lendo o segundo byte
					scancode[2] = ps2_byte_received;
					scancode[0] = 2;
					mount_scancode_count_status = 2; //points to next case
					break;
				}
				if (ps2_keystr_f0 == true)
				{
					//São 2 bytes, logo, terminou
					// Exception are all the keys starting with E0 7C. Discard this,
					// If you want to map these keys, fix them in the excel origin file.
					scancode[2] = ps2_byte_received;
					scancode[0] = 2;
					//Conclui scan
					mount_scancode_OK = true;
					reset_mount_scancode_machine();
					return true;
				}
 				break;	//case syntax suggested
			}	//case 1:
			case 2:
			//Está lendo o terceiro byte do ps2_byte_received
			{
				if (ps2_keystr_e0 || ps2_keystr_e1)
				{
					//São 3 bytes, e estou lendo o terceiro byte, logo, terminou.
					// Exception is the PrintScreen break: It will be returned as one 3 bytes ps2_byte_received releases:
					// E0 F0 7C (plus both E0 F0 12 and E0 F0 7E are dicarded), but this key is not present on MSX.
					// If you want to map this key, fix it in excel file and click on the black keyboard to rerun macro
					if( (ps2_byte_received != 0x12) )
					{
						scancode[3] = ps2_byte_received;
						scancode[0] = 3;
						//Conclui scan
						mount_scancode_OK = true;
						reset_mount_scancode_machine();
						return true;
					}
					else
					{
						//Discard E0 F0 12 here
						reset_mount_scancode_machine();
						break;
					}
				}
 				break;	//case syntax suggested
			}	//case 2:
			default:
				break;
			}	//switch (mount_scancode_count_status)
		} //while((ps2_byte_received=get_ps2_byte(&ps2_recv_buffer[0]))!=0 && !mount_scancode_OK)
		return false;
	}//if (!mount_scancode_OK)
	return false;
}


void general_debug_setup(void)
{
#if MCU == STM32F103
	gpio_set_mode(SYSTICK_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SYSTICK_PIN);
	gpio_set(SYSTICK_PORT, SYSTICK_PIN); //Default condition is "1"

	gpio_set_mode(BIT0_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, BIT0_PIN);
	gpio_set(BIT0_PORT, BIT0_PIN); //Default condition is "1"

	gpio_set_mode(INT_TIM2_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TIM2CC1_PIN);
	gpio_set(INT_TIM2_PORT, TIM2CC1_PIN); //Default condition is "1"

	gpio_set_mode(INT_TIM2_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TIM2UIF_PIN); // PC3 (MSX 8255 Pin 17)
	gpio_set(INT_TIM2_PORT, TIM2UIF_PIN); //Default condition is "1"
	
	gpio_set_mode(Dbg_Yint_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, Dbg_Yint2e3_PIN); // PC2 e 3 (MSX 8255 Pin 17)
	gpio_set(Dbg_Yint_PORT, Dbg_Yint2e3_PIN); //Default condition is "1"
	
	gpio_set_mode(Dbg_Yint_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, Dbg_Yint0e1_PIN); // PC2 (MSX 8255 Pin 17)
	gpio_set(Dbg_Yint_PORT, Dbg_Yint0e1_PIN); //Default condition is "1"
#endif
#if MCU == STM32F401
	gpio_mode_setup(USER_KEY_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, USER_KEY_PIN);

	gpio_mode_setup(INT_PS2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, INT_PS2_PIN);
	gpio_set_output_options(INT_PS2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, INT_PS2_PIN);
	gpio_set(INT_PS2_PORT, INT_PS2_PIN); //Default condition is "1"

	gpio_mode_setup(TIM2CC1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TIM2CC1_PIN);
	gpio_set_output_options(TIM2CC1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, TIM2CC1_PIN);
	gpio_set(TIM2CC1_PORT, TIM2CC1_PIN); //Default condition is "1"

	gpio_mode_setup(TIM2UIF_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TIM2UIF_PIN);
	gpio_set_output_options(TIM2UIF_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, TIM2UIF_PIN);
	gpio_set(TIM2UIF_PORT, TIM2UIF_PIN); //Default condition is "1"
	
	gpio_mode_setup(Dbg_Yint_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, Dbg_Yint_PIN);
	gpio_set_output_options(Dbg_Yint_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, Dbg_Yint_PIN);
	gpio_set(Dbg_Yint_PORT, Dbg_Yint_PIN); //Default condition is "1"
#endif
}

void put_pullups_on_non_used_pins(void)
{
	//Left these pins as inputs, but floating state is avoided by activating internal pull-up resistor.
	//gpio_mode_setup(AVAILABLE_A2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, AVAILABLE_A2_PIN_ID);
#if MCU == STM32F103
	gpio_set(AVAILABLE_B3_PORT, AVAILABLE_B3_PIN);		//Hi-Z
	gpio_set_mode(AVAILABLE_B3_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, AVAILABLE_B3_PIN);
#endif
#if MCU == STM32F401
	gpio_mode_setup(AVAILABLE_A3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, AVAILABLE_A3_PIN);
	gpio_mode_setup(AVAILABLE_B2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, AVAILABLE_B2_PIN);	//This port has a R=10K to GND
#endif
}

/*************************************************************************************************/
/*************************************************************************************************/
/******************************************* ISR's ***********************************************/
/*************************************************************************************************/
/*************************************************************************************************/
#if MCU == STM32F401
void exti15_10_isr(void) // PS/2 Clock
{
#if PS2_CLK_INTERRUPT == GPIO_INT
	//Now starts PS2Clk interrupt handler
	//Debug & performance measurement
	gpio_clear(INT_PS2_PORT, INT_PS2_PIN); //Signs start of interruption
	//This was the ISR of PS/2 clock pin. It jumps to ps2_clock_update.
	//It is an important ISR, but it does not require critical timming resources as MSX Y scan does.
	if(exti_get_flag_status(PS2_CLOCK_EXTI))  // EXTI15
	{
		bool ps2datapin_logicstate=gpio_get(PS2_DATA_PORT, PS2_DATA_PIN);
		ps2_clock_update(ps2datapin_logicstate);
		exti_reset_request(PS2_CLOCK_EXTI);
	}
	//Debug & performance measurement
	gpio_set(INT_PS2_PORT, INT_PS2_PIN); //Signs end of interruption. Default condition is "1"
#endif	//#if PS2_CLK_INTERRUPT == GPIO_INT
}
#endif	//#if MCU == STM32F401
