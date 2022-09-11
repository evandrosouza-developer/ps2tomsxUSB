/*
 * This file is part of the PS/2 to MSX keyboard Converter and 
 * MSX Keyboard Subsystem Emulator projects, using libopencm3 project.
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

#include "system.h"
#include "serial.h"
#include "hr_timer.h"
#if (USE_USB == true)
#include "cdcacm.h"
#endif	//#if (USE_USB == true)

#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

// See the inspiring file:
// https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/usart_irq_printf/usart_irq_printf.c

volatile bool enable_xon_xoff = true, xon_condition = true, xoff_condition = false, xonoff_sendnow = false;

struct sring uart_tx_ring;
struct sring uart_rx_ring;
uint8_t uart_tx_ring_buffer[UART_TX_RING_BUFFER_SIZE];
uint8_t uart_rx_ring_buffer[UART_RX_RING_BUFFER_SIZE];
uint8_t buf_dma_tx[TX_DMA_SIZE];

#if !((MCU == STM32F401) && (USART_PORT == USART1))
	//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
	//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt
struct sring dma_rx_buffer;
uint8_t buf_dma_rx[RX_DMA_SIZE];
#endif	//#if !((MCU == STM32F401) && (USART_PORT == USART1))

#if USE_USB == true
struct sring con_tx_ring;
struct sring con_rx_ring;
uint8_t con_tx_ring_buffer[CON_TX_RING_BUFFER_SIZE];
uint8_t con_rx_ring_buffer[CON_RX_RING_BUFFER_SIZE];

extern	bool nak_cleared[6];										//Declared on cdcacm.c
extern	int usb_configured;											//Declared on cdcacm.c
#endif	//#if USE_USB == true


void ring_init(struct sring *ring, uint16_t buffer_size, uint8_t *buf)
{
	ring->data = buf;
	ring->buf_sm = buffer_size;
	ring->put_ptr = 0;
	ring->get_ptr = 0;
}


/*************************************************************************************************/
/******************************************* Setup ***********************************************/
/*************************************************************************************************/
//Ready to be used outside this module.
// Setup serial used in main.
void serial_setup(void)
{
	// Initialize input and output ring buffers.
	ring_init(&uart_tx_ring, UART_TX_RING_BUFFER_SIZE, uart_tx_ring_buffer);
	ring_init(&uart_rx_ring, UART_RX_RING_BUFFER_SIZE, uart_rx_ring_buffer);
#if USE_USB == true
	ring_init(&con_tx_ring, CON_TX_RING_BUFFER_SIZE, con_tx_ring_buffer);
	ring_init(&con_rx_ring, CON_RX_RING_BUFFER_SIZE, con_rx_ring_buffer);
#endif	//#if USE_USB == true

#if !((MCU == STM32F401) && (USART_PORT == USART1))
	//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
	//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt
	ring_init(&dma_rx_buffer, RX_DMA_SIZE, buf_dma_rx);
#endif	//#if !((MCU == STM32F401) && (USART_PORT == USART1))

	// Enable clocks for USART_PORT and DMA.
	rcc_periph_clock_enable(RCC_USART);
	//Enable clocks now for DMA
	rcc_periph_clock_enable(RCC_DMA);

	// Setup GPIO pin GPIO_USART_RX_TX on GPIO port A for transmit.
#if MCU == STM32F103
	gpio_set_mode(GPIO_BANK_USART_TX, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_PIN_USART_TX);
	gpio_set(GPIO_BANK_USART_RX, GPIO_PIN_USART_RX); //pull up resistor
	gpio_set_mode(GPIO_BANK_USART_RX, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO_PIN_USART_RX);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	// Setup GPIO pin for USART transmit
	gpio_mode_setup(GPIO_BANK_USART_TX, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_USART_TX);
	gpio_set_output_options(GPIO_BANK_USART_TX, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_USART_TX);
	gpio_set_af(GPIO_BANK_USART_TX, GPIO_AF7, GPIO_PIN_USART_TX);
	// Setup GPIO pin for USART receive
	gpio_mode_setup(GPIO_BANK_USART_RX, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_USART_RX);
	gpio_set_output_options(GPIO_BANK_USART_RX, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_PIN_USART_RX);
	gpio_set(GPIO_BANK_USART_RX, GPIO_PIN_USART_RX); //pull up resistor
	gpio_set_af(GPIO_BANK_USART_RX, GPIO_AF7, GPIO_PIN_USART_RX);
#endif	//#if MCU == STM32F401

	// Setup UART parameters.
	usart_set_baudrate(USART_PORT, 115200);
	usart_set_databits(USART_PORT, 8);
	usart_set_stopbits(USART_PORT, USART_STOPBITS_1);
	usart_set_parity(USART_PORT, USART_PARITY_NONE);
	usart_set_flow_control(USART_PORT, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART_PORT, USART_MODE_TX_RX);

	// Setup USART TX DMA
	dma_ch_reset(USART_DMA_BUS, USART_DMA_TX_CH);
	dma_set_peripheral_address(USART_DMA_BUS, USART_DMA_TX_CH, (uint32_t)&USART_DR(USART_PORT));
	dma_set_memory_address(USART_DMA_BUS, USART_DMA_TX_CH, (uint32_t)buf_dma_tx);
	dma_enable_memory_increment_mode(USART_DMA_BUS, USART_DMA_TX_CH);
	dma_set_peripheral_size(USART_DMA_BUS, USART_DMA_TX_CH, DMA_PSIZE_8BIT);
	dma_set_memory_size(USART_DMA_BUS, USART_DMA_TX_CH, DMA_MSIZE_8BIT);
	dma_set_priority(USART_DMA_BUS, USART_DMA_TX_CH, DMA_PL_HIGH);
	dma_enable_transfer_complete_interrupt(USART_DMA_BUS, USART_DMA_TX_CH);
#if MCU == STM32F103
	dma_set_read_from_memory(USART_DMA_BUS, USART_DMA_TX_CH);
#endif
#if MCU == STM32F401
	dma_set_transfer_mode(USART_DMA_BUS, USART_DMA_TX_CH, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	dma_channel_select(USART_DMA_BUS, USART_DMA_TX_CH, USART_DMA_TRG_CHSEL);
	dma_set_dma_flow_control(USART_DMA_BUS, USART_DMA_TX_CH);
	dma_enable_direct_mode(USART_DMA_BUS, USART_DMA_TX_CH);		//not using DMA FIFO
#endif

	/* Technical information brief:
	 * RX DMA is implemented using the concept of non stop running. It is programmed as circular mode,
	 * at address buf_dma_rx, which is part of the structure "dma_rx_buffer". 
	 * This structure is the same as implemented at the serial buffers, as they have the size of the
	 * buf_dma_rx and the get_ptr. This get_ptr is used to point where is the begining address to read 
	 * the dma buffer. Although the structure has a put_ptr, it has no usage, as the real put pointer
	 * is managed by DMA, and it is computed through a reading of DMA_SNDTR.
	 * The DMA buffer is read in three different occasions:
	 * 1) Half transfer
	 * 2) Transfer completed
	 * 3) Whenever there is a Idle time (no new serial ativity was observed after the last character was completed).
	*/
#if !((MCU == STM32F401) && (USART_PORT == USART1))
	//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
	//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt

	// Setup USART RX DMA
	dma_ch_reset(USART_DMA_BUS, USART_DMA_RX_CH);
	dma_set_peripheral_address(USART_DMA_BUS, USART_DMA_RX_CH, (uint32_t)&USART_DR(USART_PORT));
	dma_set_memory_address(USART_DMA_BUS, USART_DMA_RX_CH, (uint32_t)buf_dma_rx);
	dma_set_number_of_data(USART_DMA_BUS, USART_DMA_RX_CH, RX_DMA_SIZE);
	dma_enable_memory_increment_mode(USART_DMA_BUS, USART_DMA_RX_CH);
	dma_disable_peripheral_increment_mode(USART_DMA_BUS, USART_DMA_RX_CH);
	dma_set_peripheral_size(USART_DMA_BUS, USART_DMA_RX_CH, DMA_PSIZE_8BIT);
	dma_set_memory_size(USART_DMA_BUS, USART_DMA_RX_CH, DMA_MSIZE_8BIT);
	dma_enable_circular_mode(USART_DMA_BUS, USART_DMA_RX_CH);
	dma_set_priority(USART_DMA_BUS, USART_DMA_RX_CH, DMA_PL_HIGH);
	dma_enable_half_transfer_interrupt(USART_DMA_BUS, USART_DMA_RX_CH);
	dma_enable_transfer_complete_interrupt(USART_DMA_BUS, USART_DMA_RX_CH);
#if MCU == STM32F103
	dma_set_read_from_peripheral(USART_DMA_BUS, USART_DMA_RX_CH);
#endif	//#if MCU == STM32F103
#if MCU == STM32F401
	dma_set_transfer_mode(USART_DMA_BUS, USART_DMA_RX_CH, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	dma_channel_select(USART_DMA_BUS, USART_DMA_RX_CH, USART_DMA_TRG_CHSEL);
	dma_set_dma_flow_control(USART_DMA_BUS, USART_DMA_RX_CH);
	dma_enable_direct_mode(USART_DMA_BUS, USART_DMA_RX_CH);		//not using DMA FIFO
#endif	//#if MCU == STM32F401

	// Prepare RX DMA interrupts
	dma_clear_interrupt_flags(USART_DMA_BUS, USART_DMA_RX_CH, DMA_CGIF);
	nvic_set_priority(USART_DMA_RX_IRQ, IRQ_PRI_USART_DMA);
	nvic_enable_irq(USART_DMA_RX_IRQ);
#else		//#else #if !((MCU == STM32F401) && (USART_PORT == USART1))
	//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
	//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt
#endif	//#else #if !((MCU == STM32F401) && (USART_PORT == USART1))

	// Prepare TX DMA interrupts
	usart_disable_tx_complete_interrupt(USART_PORT);
	nvic_set_priority(USART_DMA_TX_IRQ, IRQ_PRI_USART_DMA);
	nvic_enable_irq(USART_DMA_TX_IRQ);

	// Enable the USART TX.
	usart_enable_tx_dma(USART_PORT);
	
#if !((MCU == STM32F401) && (USART_PORT == USART1))
	//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
	//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt

	// Enable the USART RX.
	usart_enable_idle_interrupt(USART_PORT); usart_disable_rx_interrupt(USART_PORT);
	usart_enable_rx_dma(USART_PORT);
	// Enable the DMA engine of USART RX. RX is never shutdown.
	dma_enable_ch(USART_DMA_BUS, USART_DMA_RX_CH);
#else		//#else #if !((MCU == STM32F401) && (USART_PORT == USART1))
	// Enable the USART RX.
	usart_enable_rx_interrupt(USART_PORT); usart_disable_idle_interrupt(USART_PORT);
#endif	//#else #if !((MCU == STM32F401) && (USART_PORT == USART1))
	nvic_set_priority(NVIC_USART_IRQ, IRQ_PRI_USART);
	nvic_enable_irq(NVIC_USART_IRQ);
	//Clear USART_SR_IDLE, to avoid IDLE new interrupts without new incoming chars.
	uint8_t bin = USART_SR(USART_PORT); bin = USART_DR(USART_PORT); bin &= 0xFF;
	usart_enable(USART_PORT);
	
	// Finally init X_ON/X_OFF flags.
	xon_condition = true;
	xoff_condition = false;
	xonoff_sendnow = false;
#if USE_USB == true
	nak_cleared[EP_CON_DATA_OUT] = true;
	nak_cleared[EP_UART_DATA_OUT] = true;
#endif
}


//Update usart communication parameters from USB host
#if USE_USB == true
void usart_update_comm_param(struct usb_cdc_line_coding *usart_comm_param)
{
	// Setup UART baud rate
	usart_set_baudrate(USART_PORT, usart_comm_param->dwDTERate);
	// Setup UART data bits length
	if (usart_comm_param->bParityType)
		usart_set_databits(USART_PORT, (usart_comm_param->bDataBits + 1 <= 8 ? 8 : 9));
	else
		usart_set_databits(USART_PORT, (usart_comm_param->bDataBits <= 8 ? 8 : 9));
	// Setup UART parity
	switch(usart_comm_param->bParityType)
	{
		case 0:
			usart_set_parity(USART_PORT, USART_PARITY_NONE);
		break;
		case 1:
			usart_set_parity(USART_PORT, USART_PARITY_ODD);
		break;
		case 2:
		break;
		default:
			usart_set_parity(USART_PORT, USART_PARITY_EVEN);
		break;
	}
	// Setup UART stop bits
	switch(usart_comm_param->bCharFormat)
	{
		case 0:
			usart_set_stopbits(USART_PORT, USART_STOPBITS_1);
		break;
		case 1:
			usart_set_stopbits(USART_PORT, USART_STOPBITS_1_5);
		break;
		case 2:
		break;
		default:
			usart_set_stopbits(USART_PORT, USART_STOPBITS_2);
		break;
	}
}
#endif	//#if USE_USB == true


void serial_rx_start(void)
{
#if (MCU == STM32F401)
#if (USART_PORT == USART2)
	dma_disable_ch(USART_DMA_BUS, USART_DMA_RX_CH);
	while(DMA_SCR(USART_DMA_BUS, USART_DMA_RX_CH) & DMA_SxCR_EN) __asm("nop");
#endif	//#if (USART_PORT == USART2)
#endif	//#if (MCU == STM32F401)
#if MCU == STM32F103
	while(DMA_CCR(USART_DMA_BUS, USART_DMA_RX_CH) & DMA_CCR_EN) __asm("nop");
#endif	//#if MCU == STM32F103
	//Clear USART_SR_IDLE, to avoid IDLE new interrupts without new incoming chars.
	uint8_t bin = USART_SR(USART_PORT); bin = USART_DR(USART_PORT); bin &= 0xFF;
#if !((MCU == STM32F401) && (USART_PORT == USART1))
	dma_set_number_of_data(USART_DMA_BUS, USART_DMA_RX_CH, RX_DMA_SIZE);
#endif	//#if !((MCU == STM32F401) && (USART_PORT == USART1))
	uart_rx_ring.put_ptr = 0;
	uart_rx_ring.get_ptr = 0;
#if USE_USB == true
	con_rx_ring.put_ptr = 0;
	con_rx_ring.get_ptr = 0;
#endif	//#if USE_USB == true
	dma_enable_ch(USART_DMA_BUS, USART_DMA_RX_CH);
}



//---------------------------------------------------------------------------------------
//----------------------------Communication output routines------------------------------
//---------------------------------------------------------------------------------------
//Used as an internal function.
//It is used to put a char in the ring buffer, both TX and RX.
//It returns number of chars are in the buffer of 0xFFFF when there was no room to add this char.
uint16_t ring_put_ch(struct sring *ring, uint8_t ch)
{
	uint16_t i, i_next;
	i = ring->put_ptr;											//i is the original position
	i_next = (i + 1) & (ring->buf_sm);	//i_next is the next position of i
	if(i_next != ring->get_ptr)
	{
		ring->data[i] = ch;			//saves in the put_ptr position 
		ring->put_ptr = i_next;	//now put_ptr points to the next position of i
		//Optimizing calculations inside the interrupt => The general formula is:
		//CharsInBuffer = (RING_BUFFER_SIZE - ring.get_ptr + ring.put_ptr) % RING_BUFFER_SIZE;
		//but BASE_RING_BUFFER_SIZE is a power of two, so the rest of the division is computed zeroing
		//the higher bits of the summed buffer lenght, so in the case of 256 (2**8), you have to keep only
		//the lowest 8 bits: (BASE_RING_BUFFER_SIZE - 1).
		return (uint16_t)(ring->buf_sm + 1 - ring->get_ptr + i_next) & (ring->buf_sm);
	}
	else
	{
		//No room for more (Buffer full)
		return 0xFFFF;
	}
}


//Used as an external function.
//It is used to put a char in the ring TX buffer, as it will initiate TX if the first one is put on buffer.
//It returns number of chars are in the buffer or 0xFFFF when there was no room to add this char.
uint16_t uart_tx_ring_dma_send_ch(uint8_t ch)
{
	uint16_t dma_remaining;
	bool uart_tx_ring_empty;

	dma_remaining = dma_get_number_of_data(USART_DMA_BUS, USART_DMA_TX_CH);
	uart_tx_ring_empty = (uart_tx_ring.get_ptr == uart_tx_ring.put_ptr);
	//1) If transmit is concluded, move first byte to serial. It will raise TXE instantly.
	/*if( (USART_SR(USART_PORT) & USART_SR_TC) && uart_tx_ring_empty && !dma_remaining )
	{
		usart_send(USART_PORT, (uint16_t)ch);
		return 0;
	}*/
	//2) If serial tx buffer is empty, but it's still transmitting
	if( (USART_SR(USART_PORT) & USART_SR_TXE) && uart_tx_ring_empty && !dma_remaining )
	{
		usart_send(USART_PORT, (uint16_t)ch);
		return 0;
	}
	//3) If no TX DMA is in charge, move this char into DMA buffer
	//	 and start TX DMA with this only one byte
	if(!dma_remaining)
	{
		//Put this one char in tx dma
		buf_dma_tx[0] = ch;
		//And start TX DMA with this one
		dma_set_number_of_data(USART_DMA_BUS, USART_DMA_TX_CH, sizeof(uint8_t));
		dma_enable_ch(USART_DMA_BUS, USART_DMA_TX_CH);
		usart_enable_tx_dma(USART_PORT);
		return 0;
	}
	//No one of the previous options. Put ch onto tx_buffer
	return ring_put_ch(&uart_tx_ring, ch);
}	//uint16_t uart_tx_ring_dma_send_ch(uint8_t ch)


//Ready to be used outside this module.
// Put a char (uint8_t) on console buffer (con_tx_ring or uart_tx_ring). Non blocking function.
// - Will put in con_tx_ring if usb is configured and in uart_tx_ring when NO USB or usb is NOT configured
// It returns number of chars are in the buffer or 0xFFFF when there was no room to add this char.
uint16_t con_put_char(uint8_t ch)
{
	CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
			#if USE_USB == true
			// Put char in console ring.
			if(usb_configured)
				ring_put_ch(&con_tx_ring, data);
			else	//if(usb_configured)
				uart_tx_ring_dma_send_ch(data);
		#else	//#if USE_USB == true
			// Put char in uart_tx_ring.
			uart_tx_ring_dma_send_ch(data);
		#endif	//	//#if USE_USB == true
	CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET	//CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET

#if USE_USB == true
	// Put char in console ring.
	if(usb_configured)
		return ring_put_ch(&con_tx_ring, ch);
	else	//if(usb_configured)
		return uart_tx_ring_dma_send_ch(ch);
#else	//#if USE_USB == true
	// Put char in uart_tx_ring.
	return uart_tx_ring_dma_send_ch(ch);
#endif	//	//#if USE_USB == true
}


//Ready to be used outside this module.
// Put an ASCIIZ (uint8_t) string on serial buffer.
// Wait until buffer is filled.
void con_send_string(uint8_t *string)
{
	uint16_t iter = 0;

#if USE_USB == true
	uint16_t qtty;
	qtty =  (con_tx_ring.buf_sm + 1 - con_tx_ring.get_ptr + con_tx_ring.put_ptr) & con_tx_ring.buf_sm;
#endif	//#if USE_USB == true
	while(string[iter])
	{
		while(con_put_char(string[iter]) == 0xFFFF)
			__asm("nop");
		iter++;
	}
#if USE_USB == true
	//If the quantity before filled was zero, means that first transmition is necessary. Afterwards, the CB will handle transmition.
	if(usb_configured && !qtty)
		first_put_ring_content_onto_ep(&con_tx_ring, EP_CON_DATA_IN);
#endif	//#if USE_USB == true
}


//Ready to be used outside this module.
//Force next console reading ch.
// It assumes that console buffer is empty, to do next console reading be what you put.
void insert_in_con_rx(uint8_t ch)
{
#if USE_USB == true
	if(usb_configured)
		ring_put_ch(&con_rx_ring, ch);
	else
		ring_put_ch(&uart_rx_ring, ch);
#else		//#if USE_USB == true
	ring_put_ch(&uart_rx_ring, ch);
#endif	//#if USE_USB == true
}


//---------------------------------------------------------------------------------------
//----------------------------Communication input routines-------------------------------
//---------------------------------------------------------------------------------------
//Used as an internal function.
//Returns true if there is a char available to read in the ring (both TX and RX) or false if not.
uint16_t ring_avail_get_ch(struct sring *ring)
{
	return (ring->buf_sm + 1 - ring->get_ptr + ring->put_ptr) & (ring->buf_sm);
}


//Used as an internal function.
//It returns char when it is available or 0xFF and qty_in_buffer=0 when no one is available
//Used on both TX and RX buffers.
uint8_t ring_get_ch(struct sring *ring, uint16_t *qty_in_buffer)
{
	if(ring->get_ptr == ring->put_ptr)
	{
		//No char in buffer
		*qty_in_buffer = 0;
		return 0xFF;
	}
	uint16_t local_get_ptr;
	local_get_ptr = ring->get_ptr;
	int8_t result = ring->data[local_get_ptr];
	local_get_ptr++;
	ring->get_ptr = local_get_ptr & (ring->buf_sm); //if(local_get_ptr >= (uint16_t)BASE_RING_BUFFER_SIZE)	i = 0;
	*qty_in_buffer = (ring->buf_sm + 1 - ring->get_ptr + ring->put_ptr) & (ring->buf_sm);
	return result;
}


//Ready to be used from outside of this module.
// If there is an available char in USART_PORT RX ring, it returns true.
uint16_t con_available_get_char(void)
{
#if USE_USB == true
	if(usb_configured)
		return (ring_avail_get_ch(&con_rx_ring));
	else
		return (ring_avail_get_ch(&uart_rx_ring));
#else	//#if USE_USB == true
	return (ring_avail_get_ch(&uart_rx_ring));
#endif	//	//#if USE_USB == true
}


//xon_xoff_control
static void xon_xoff_rx_control(struct sring *ring, uint16_t qty_in_buffer)
{
	if (enable_xon_xoff)
	{
		if (qty_in_buffer >= (3 * ring->buf_sm / 4))	//X_OFF_TRIGGER)
		{
			xon_condition = false;
			if (!xoff_condition)													//To send X_OFF only once
			{
				xoff_condition = true;
				xonoff_sendnow = true;
			}
		}
		else if (qty_in_buffer < (ring->buf_sm / 2))	//(uint16_t)X_ON_TRIGGER)
		{
			xoff_condition = false;
			if (!xon_condition)														//To send X_ON only once
			{
				xon_condition = true;
				xonoff_sendnow = true;
			}  //if (!xon_condition)
		} //else if (*qty_in_buffer <= (uint16_t)X_ON_TRIGGER)
	} //if (enable_xon_xoff)
}


//Used as an internal function.
//Implemented X_ON/X_OFF protocol, so it can be used only to console proposal. You have to use 
//It returns a int8_t char when it is available or 0xFF and qty_in_buffer=0 when no one is available
//Used only on RX buffer.
static int8_t ring_rx_get_ch(uint16_t *qty_in_buffer)
{
	int8_t ch;

#if USE_USB == true
	if(usb_configured)
	{
		ch = ring_get_ch(&con_rx_ring, qty_in_buffer);
		if ( (*qty_in_buffer >= (3 * con_rx_ring.buf_sm / 4)) && (nak_cleared[EP_CON_DATA_OUT]) )
			//con_rx_ring is running out of space. Set NAK on endpoint.
			set_nak_endpoint(EP_CON_DATA_OUT);
		else if ( (*qty_in_buffer < (con_rx_ring.buf_sm / 2)) && (!nak_cleared[EP_CON_DATA_OUT]) )
			//Now con_rx_ring has space. Clear NAK on endpoint.
			clear_nak_endpoint(EP_CON_DATA_OUT);
		return ch;
	}	//if(usb_configured)
	else
	{	//else if(usb_configured)
		ch = ring_get_ch(&uart_rx_ring, qty_in_buffer);
		xon_xoff_rx_control(&uart_rx_ring, *qty_in_buffer);
		return ch;
	}	//else if(usb_configured)
#else	//#else #if USE_USB == true
	ch = ring_get_ch(&uart_rx_ring, qty_in_buffer);
	xon_xoff_rx_control(&uart_rx_ring, *qty_in_buffer);
	return ch;
#endif	//#else	#if USE_USB == true
}


//Ready to be used from outside of this module.
// If there is an available char in serial, it returns with an uint8_t.
//You MUST use the above function "con_available_get_char" BEFORE this one,
//in order to verify a valid available reading by this function.
uint8_t con_get_char(void)
{
	//Yes, it's correct: The same call to both console reception rings!
	uint16_t bin; //information to be discarded
	return (uint8_t)ring_rx_get_ch(&bin);
}


static uint8_t getchar_locked(void)
{
#if USE_USB == true
	uint16_t bin; //information to be discarded
	if(usb_configured)
	{
		while(con_rx_ring.get_ptr == con_rx_ring.put_ptr) __asm("nop");
		return(uint8_t)ring_get_ch(&con_rx_ring, &bin);
	}
	else
	{
		while(uart_rx_ring.get_ptr == uart_rx_ring.put_ptr) __asm("nop");
		return(con_get_char());
	}
#else	//#if USE_USB == true
	while(uart_rx_ring.get_ptr == uart_rx_ring.put_ptr) __asm("nop");
	return(con_get_char());
#endif	//#if USE_USB == true
}


// Read a line from serial. You can limit how many chars will be available to enter.
// It returns how many chars were read. It is a blocking function
//It's correct here too: The same call to both reception rings!
uint8_t console_get_line(uint8_t *s, uint16_t len)
{
	uint8_t	*t = s;
	uint8_t	ch, mount_t[3];
	bool	valid_decimal_digit;

	*t = '\000';
	// read until a <CR> is received
	while ((ch = getchar_locked()) != '\r')
	{
		// First check valid characters in filename
		valid_decimal_digit = true;
		if( (ch < '0') || (ch > '9'))
			valid_decimal_digit = false;
		if (ch == 0x7F)  //Backspace
		{
			if (t > s)
			{
				// send ^H ^H to erase previous character
				con_send_string((uint8_t*)"\b \b");
				t--;
			}
		}   //if (ch == 0x8)
		else {
			if (valid_decimal_digit)
			{
				*t = ch;
				mount_t[0] = ch;
				mount_t[1] = 0;
				con_send_string(mount_t);
				if ((t - s) < len)
					t++;
			}
		}   //else if (c == 0x7F)
		// update end of string with NUL
		*t = '\000';
	}   //while (ch = console_getc()) != '\r')
	return t - s;
}


//=======================================================================================
//=============================Convertion routines Group=================================
//=======================================================================================
//Ready to be used outside this module.
// Convert a two byte string pointed by i into a binary byte. 
// It returns and no blocking function if there is enough space on USART_PORT TX buffer, otherwise,
uint8_t conv_2a_hex_to_uint8(uint8_t *instring, int16_t i)
{
	uint8_t binuint8, ch;

	ch = instring[i];// & 0x5F; //to capital letter
	binuint8 = (ch > ('A'-1) && ch < ('F'+1)) ?
						 (ch-('A'-10)) : (ch-'0'); //55 = "A"0x41-10; 48="0"0x30
	binuint8 <<= 4;
	ch = instring[i+1];// & 0x5F; //to capital letter
	binuint8 += (ch > ('A'-1) && ch < ('F'+1)) ?
							(ch-('A'-10)) : (ch-'0'); //55 = "A"0x41-10; 48="0"0x30
	return binuint8;
}


//Ready to be used outside this module.
// Convert a word (32 bit binary) to into a 8 char string. 
void conv_uint32_to_8a_hex(uint32_t value, uint8_t *outstring)
{
	uint8_t iter;

	/*end of string*/
	outstring += 8;
	*(outstring--) = 0;

	for(iter=0; iter<8; iter++)
	{
		*(outstring--) = (((value&0xf) > 0x9) ? (0x40 + ((value&0xf) - 0x9)) : (0x30 | (value&0xf)));
		value >>= 4;
	}
}


//Ready to be used outside this module.
// Convert a half-word (16 bit binary) to into a 4 char string. 
void conv_uint16_to_4a_hex(uint16_t value, uint8_t *outstring)
{
	uint8_t iter;

	/*end of string*/
	outstring += 4;
	*(outstring--) = 0;

	for(iter=0; iter<4; iter++)
	{
		*(outstring--) = (((value&0xf) > 0x9) ? (0x40 + ((value&0xf) - 0x9)) : (0x30 | (value&0xf)));
		value >>= 4;
	}
}


//Ready to be used outside this module.
// Convert a word (32 bit binary) to into a 10 char string. 
void conv_uint32_to_dec(uint32_t value, uint8_t *outstring)
{
	uint8_t iter, pos;
	const uint32_t power10[10] = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

	bool insertzero = false;
	for(iter=0; iter<=10; iter++)	//Fill outstring with \0
		(outstring[iter]) = 0;

	if(value == 0)
		outstring[0] = '0';
	else
	{
		pos = 0;
		for(iter=0; iter<10; iter++)
		{
			if ((value >= power10[iter]) || insertzero)
			{
				outstring[pos] = '0';
				while (value >= power10[iter])
				{
					value -= power10[iter];
					outstring[pos] += 1;
					insertzero = true;
				}
			}	//if (value > power10[iter])
			if (insertzero)
				pos++;
		}	//for(iter=0; iter<5; iter++)
	}
}


//Ready to be used outside this module.
// Convert a byte (8 bit binary) to into a 2 char string. 
void conv_uint8_to_2a_hex(uint8_t value, uint8_t *outstring)
{
	uint8_t iter;

	/*end of string*/
	outstring += 2;
	*(outstring--) = 0;

	for(iter=0; iter<2; iter++)
	{
		*(outstring--) = (((value&0xf) > 0x9) ? (0x40 + ((value&0xf) - 0x9)) : (0x30 | (value&0xf)));
		value >>= 4;
	}
}


//=======================================================================================
//============================= To be used with printf ==================================
//=======================================================================================
//Used as an internal function.
// Used to write more than one char on uart_tx_ring and initiate a transmission
int _write(int file, char *ptr, int len)
{
	// If the target file isn't stdout/stderr, then return an error
	// since we don't _actually_ support file handles
	if (file != STDOUT_FILENO && file != STDERR_FILENO) {
		// Set the errno code (requires errno.h)
		errno = EIO;
		return -1;
	}
	int i;
	uint8_t m_str[2];
	m_str[1] = 0;
	for (i = 0; i < len; i++)
	{
		// If we get a newline character, also be sure to send the carriage
		// return character first, otherwise the serial console may not
		// actually return to the left.
		if (ptr[i] == '\n') 
		{
			m_str[0] = '\r';
			con_send_string(m_str);
		}
		// Write the character to send to the USART transmit buffer, and block
		// until it has been sent.
		m_str[0] = ptr[i];
		con_send_string(m_str);
	}
	// Return the number of bytes we sent
	return i;
}


/*************************************************************************************************/
/*************************************************************************************************/
/******************************************* ISR's ***********************************************/
/*************************************************************************************************/
/*************************************************************************************************/
#if !(MCU == STM32F401) || !(USART_PORT == USART1)
	//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
	//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt
//same function used at:
//Idle time detected on USART RX ISR and for DMA USART RX ISR
static void usart_rx_read_dma(void)
{
	uint16_t i, qtty_dma_rx_in, dmarx_put_ptr, dma_get;

	//Put in uart_rx_ring what was received from USART via DMA
	dma_get =	dma_get_number_of_data(USART_DMA_BUS, USART_DMA_RX_CH);

	dmarx_put_ptr =	(dma_rx_buffer.buf_sm + 1 - dma_get) & (dma_rx_buffer.buf_sm);
	qtty_dma_rx_in = (dma_rx_buffer.buf_sm + 1 - dma_rx_buffer.get_ptr + dmarx_put_ptr) & 
									 (dma_rx_buffer.buf_sm);// % RX_DMA_SIZE

#if USE_USB == true
	uint16_t qtty_uart_rx_ring;
	qtty_uart_rx_ring = (uart_rx_ring.buf_sm + 1 - uart_rx_ring.get_ptr + uart_rx_ring.put_ptr) & (uart_rx_ring.buf_sm);
#endif	//#if USE_USB == true

	// Copy data from DMA buffer into USART RX buffer (uart_rx_ring)
	for(i = 0; i < qtty_dma_rx_in; i++)
		ring_put_ch(&uart_rx_ring, buf_dma_rx[dma_rx_buffer.get_ptr + i]);

	//Update get pointer for the next DMA reading
	dma_rx_buffer.get_ptr = (dma_rx_buffer.get_ptr + i) & (dma_rx_buffer.buf_sm);

	//Now clear USART_SR_IDLE, to avoid IDLE new interrupts without new incoming chars.
	//It will be processed through a read to the USART_SR register followed by a read to the USART_DR register.
	uint8_t bin = USART_SR(USART_PORT); bin = USART_DR(USART_PORT); bin &= 0xFF;
	dma_clear_interrupt_flags(USART_DMA_BUS, USART_DMA_RX_CH, DMA_CGIF);

#if USE_USB == true
	//If the quantity before filled was 0, means that first transmition is necessary. So start it.
	//Afterwards, the CB will be in charge of handling the transmition.
	if(usb_configured && !qtty_uart_rx_ring)
		first_put_ring_content_onto_ep(&uart_rx_ring, EP_UART_DATA_IN);
#endif	//#if USE_USB == true
}
#endif	//#if !(MCU == STM32F401) || !(USART_PORT == USART1)



ISR_USART
{
#if !((MCU == STM32F401) && (USART_PORT == USART1))
	//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
	//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt

	//Check if Idle time detected on USART RX:
	if( (USART_CR1(USART_PORT)&USART_CR1_IDLEIE) && (USART_SR(USART_PORT)&USART_SR_IDLE) )
	{
		//Idle time detected
		usart_rx_read_dma();
	}

#else	//#if !((MCU == STM32F401) && (USART_PORT == USART1))

	// Check if we were called because of RXNE.
 	if( (USART_CR1(USART_PORT)&USART_CR1_RXNEIE) && (USART_SR(USART_PORT)&USART_SR_RXNE) )
	{
		//Suppose not be here, but in case of, clear USART error flags
		uint8_t bin = USART_SR(USART_PORT); bin = USART_DR(USART_PORT); bin &= 0xFF;

#if USE_USB == true
		uint16_t qtty_uart_rx_ring;
		qtty_uart_rx_ring = (uart_rx_ring.buf_sm + 1 - uart_rx_ring.get_ptr + uart_rx_ring.put_ptr) & (uart_rx_ring.buf_sm);
#endif	//#if USE_USB == true

		// Retrieve the data from the peripheral and put in uart_rx_ring.
		ring_put_ch(&uart_rx_ring, (uint8_t)usart_recv(USART_PORT));

#if USE_USB == true
		//If the quantity before filled was 0, means that first transmition is necessary. So start it.
		//Afterwards, the CB will be in charge of handling the transmition.
		if(usb_configured && !qtty_uart_rx_ring)
			first_put_ring_content_onto_ep(&uart_rx_ring, EP_UART_DATA_IN);
#endif	//#if USE_USB == true
	}
#endif	//#if !((MCU == STM32F401) && (USART_PORT == USART1))
}



#if !((MCU == STM32F401) && (USART_PORT == USART1))
	//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
	//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt
//ISR for DMA USART RX => Transfer completed and half transfer
ISR_DMA_CH_USART_RX
{
	usart_rx_read_dma();
}

#endif	//#if !(MCU == STM32F401) || !(USART_PORT == USART1)



//ISR for DMA USART TX
ISR_DMA_CH_USART_TX
{
	uint16_t num_avail_uart_tx, qty_in_buffer;
	
	//Stop DMA
	usart_disable_tx_dma(USART_PORT);
	dma_disable_ch(USART_DMA_BUS, USART_DMA_TX_CH);//DMA disable transmitter
	dma_clear_interrupt_flags(USART_DMA_BUS, USART_DMA_TX_CH, DMA_CGIF);

	num_avail_uart_tx = ring_avail_get_ch(&uart_tx_ring);

#if (USE_USB == true)
	if(usb_configured)
	{
		if ( (num_avail_uart_tx >= (uart_tx_ring.buf_sm >> 1)) && (nak_cleared[EP_UART_DATA_OUT]) )
			//Uart_tx_ring is running out of space. Set NAK on endpoint.
			set_nak_endpoint(EP_UART_DATA_OUT);
		else if ( (num_avail_uart_tx < (uart_tx_ring.buf_sm >> 2)) && (!nak_cleared[EP_UART_DATA_OUT]) )
			//Now uart_tx_ring has space. Clear NAK on endpoint.
			clear_nak_endpoint(EP_UART_DATA_OUT);
	}
#endif	//#if (USE_USB == true)

	if (num_avail_uart_tx > TX_DMA_SIZE)
		num_avail_uart_tx = TX_DMA_SIZE;

	if (!num_avail_uart_tx)
	{	//No available chars in uart_tx_ring
#if (USE_USB == true)
		if(usb_configured)
			//Condition USB is configured => USART working as CDCACM (serial to USB converter)
			//No more data is available to send to USART. Let DMA disabled: It's no longer needed.
			return;
		else	//if(usb_configured)
		{
			//Condition USB NOT configured => USART working as console => Check XON/XOFF
			CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
				buf_dma_tx[0] = data;
				//And so, reinit DMA.
				usart_enable_tx_dma(USART_PORT);
				dma_set_number_of_data(USART_DMA_BUS, USART_DMA_TX_CH, sizeof(uint8_t)); //Only xonxoff is available to send to USART.
				USART_SR(USART_PORT) &= ~USART_SR_TC;
				return;
			CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET	//CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
			else	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
				//No more data is available to send to USART. Let DMA disabled: It's no longer needed.
				return;
		}	//else if(usb_configured) //Condition USB NOT configured
#else	//#if (USE_USB == true)
		//#Condition NO USE_USB => USART working as console => Check XON/XOFF
		CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
			buf_dma_tx[0] = data;
			//And so, reinit DMA.
			dma_set_number_of_data(USART_DMA_BUS, USART_DMA_TX_CH, sizeof(uint8_t)); //Only xonxoff is available to send to USART.
			/*USART_SR(USART_PORT) &= ~USART_SR_TC;*/
			dma_enable_ch(USART_DMA_BUS, USART_DMA_TX_CH);
			usart_enable_tx_dma(USART_PORT);
			return;
		CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET	//CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
		else	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
		{
			//No more data is available to send to USART. Let DMA disabled: It's no longer needed.
			return;
		}	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
#endif	//#if (USE_USB == true)
	}	//if (num_avail_uart_tx == 0)
	else	//if (num_avail_uart_tx == 0)
#if (USE_USB == true)	//Bypass XON/XOFF processing if serial is only a CDC adapter
	{	//else if (num_avail_uart_tx == 0)
		if(usb_configured)
		{	//Move from uart_tx_ring to buf_dma_tx
			for(uint16_t i = 0; i < num_avail_uart_tx; i++)
				buf_dma_tx[i] = (uint8_t)ring_get_ch(&uart_tx_ring, &qty_in_buffer);
		} //if(usb_configured)
		else	//if(usb_configured)
		{	//Condition: USE_USB, USB NOT configured => USART working as console => Check XON/XOFF
			CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
				buf_dma_tx[0] = data;
				//Move from uart_tx_ring to buf_dma_tx
				for(uint16_t i = 1; i < num_avail_uart_tx; i++)	//Started from 1 because buf_dma_tx[0]=0
					buf_dma_tx[i] = (uint8_t)ring_get_ch(&uart_tx_ring, &qty_in_buffer);
			CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET	//CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
			else	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
			{	//Condition USE_USB, USB NOT configured => USART working as console => No check XON/XOFF
				//Move from uart_tx_ring to buf_dma_tx
				for(uint16_t i = 0; i < num_avail_uart_tx; i++)
					buf_dma_tx[i] = (uint8_t)ring_get_ch(&uart_tx_ring, &qty_in_buffer);
			}	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
		}	//else if(usb_configured)
	}	//else if (num_avail_uart_tx == 0)
#else	//#if (USE_USB == true)
	//Condition NO USE_USB. Consider XON/XOFF processing, as serial is only a Console
	CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
		buf_dma_tx[0] = data;
		//Move from uart_tx_ring to buf_dma_tx
		for(uint16_t i = 1; i < num_avail_uart_tx; i++)	//Started from 1 because buf_dma_tx[0]=0
			buf_dma_tx[i] = (uint8_t)ring_get_ch(&uart_tx_ring, &qty_in_buffer);
	CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET	//if (xonoff_sendnow)
	else	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
	{
		//Move from uart_tx_ring to buf_dma_tx
		for(uint16_t i = 0; i < num_avail_uart_tx; i++)
			buf_dma_tx[i] = (uint8_t)ring_get_ch(&uart_tx_ring, &qty_in_buffer);
	}	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
#endif	//#if (USE_USB == true)	//Bypass XON/XOFF processing if serial is only a CDC adapter
	//And so, reinit DMA.
	dma_set_number_of_data(USART_DMA_BUS, USART_DMA_TX_CH, num_avail_uart_tx);
	USART_SR(USART_PORT) &= ~USART_SR_TC;
	dma_enable_ch(USART_DMA_BUS, USART_DMA_TX_CH);
	usart_enable_tx_dma(USART_PORT);
}
