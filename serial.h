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

#if !defined SERIAL_H
#define SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/cdc.h>

#include "system.h"


struct sring
{
	uint8_t *data;
	uint16_t bufSzMask;
	uint16_t put_ptr;
	uint16_t get_ptr;
};

#define X_ON								17
#define X_OFF 							19


void serial_setup(void);

void serial_rx_start(void);

void usart_update_comm_param(struct usb_cdc_line_coding*);

void ring_init(struct sring*, uint8_t*, uint16_t);

void pascal_string_init(struct s_pascal_string*, uint8_t*, uint8_t);

void do_dma_usart_tx_ring(uint16_t number_of_data);

// Append an ASCIIZ (uint8_t) string at the end of s_pascal_string buffer.
void string_append(uint8_t*, struct s_pascal_string*);

// Put a char (uint8_t) on serial buffer.
// They return number of chars are in the buffer or 0xFFFF when there was no room to add this char.
// They are non blocking functions
uint16_t ring_put_ch(struct sring*, uint8_t);

//It is used to start a usart transmission, of not started, fill DMA if it is idle, and fill the excedent characters in the ring TX buffer,
//It returns the number of put_in_uart_tx bytes.
//It also returns the number of bytes are in the uart_tx_ring in the address of the third parameter.
uint16_t uart_tx_ring_dma_send_buf(uint8_t*, uint16_t, uint16_t*);

// Send a ASCIIZ string to serial (up to 127 chars).
// It is a non blocking function while there is room on TX Buffer
void con_send_string(uint8_t*);

// If there is an available char in choosen RX ring, it returns true.
// They are non blocking functions
uint16_t ring_avail_get_ch(struct sring*);
uint16_t con_available_get_char(void);

// If there is an available char in serial, it returns with an uint8_t, but
// before, you have to use con_available_get_char or ring_avail_get_ch to check their availability.
// They are non blocking functions.
uint8_t ring_get_ch(struct sring*, uint16_t*);
uint8_t con_get_char(void);

// Read a line from console. You can limit how many chars will be available to enter.
// It returns how many chars were read.
// It is a blocking function
uint8_t console_get_line(uint8_t*, uint16_t);

//Force next console reading ch
void insert_in_con_rx(uint8_t);

/* To be used with printf */
int _write(int, char*, int);

/*Functions to convert strings*/
// Convert a word (32 bit) into a up to 8 char string.
void conv_uint32_to_dec(uint32_t, uint8_t*);

// Convert a two byte string pointed by i into a binary byte. 
uint8_t conv_2a_hex_to_uint8(uint8_t*, int16_t);

// Convert a word (32 bit binary) to into a 8 char string. 
void conv_uint32_to_8a_hex(uint32_t, uint8_t*);

// Convert a half-word (16 bit binary) to into a 4 char string. 
void conv_uint16_to_4a_hex(uint16_t, uint8_t*);

// Convert a byte (8 bit binary) to into a 2 char string. 
void conv_uint8_to_2a_hex(uint8_t, uint8_t*);

// Convert a half-word (16 bit binary) to into a 5 char dec string. 
void conv_uint16_to_dec(uint16_t, uint8_t*);



#ifdef __cplusplus
}
#endif

#endif	//#ifndef SERIAL_H
