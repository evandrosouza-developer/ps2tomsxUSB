/** @defgroup 03 USART USART_Group
 *
 * @ingroup infrastructure_apis
 *
 * @file serial.h USART with DMA support routines on STM32F1 and STM32F4.
 *
 * @brief <b>USART with DMA support routines on STM32F1 and STM32F4. Header file of serial.c.</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 01 September 2022
 *
 * This library supports the USART with DMA in the STM32F4 and STM32F1
 * series of ARM Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms ref lgpl_license
 */

/*
 * This file is part of the PS/2 to MSX keyboard Converter Enviroment,
 * covering MSX keyboard Converter and MSX Keyboard Subsystem Emulator
 * designs, based on libopencm3 project.
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


#if !defined SERIAL_H
#define SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/cdc.h>

#include "system.h"
#include "hr_timer.h"
/*#if (USE_USB == true)
#include "cdcacm.h"
#endif  //#if (USE_USB == true)*/


/** @brief Setup the USART and DMA
 *
*/
void serial_setup(void);


/** @brief Restart the USART
 *
*/
void serial_rx_restart(void);


/**
 * @brief Puts the struct usb_cdc_line_coding parameters onto serial.
 *
 * @param usart_comm_param pointer to struct usb_cdc_line_coding.
 */
void usart_update_comm_param(struct usb_cdc_line_coding*usart_comm_param);


/**
 * @brief Does prepare DMA if it is idle if DMA is idle. It is used force a start DMA sending of uart_tx_ring, to let DMA routines take control until buffer is flushed.
 *
 * @param ring pointer to struct sring.
 * @param buf pointer to already defined buffer.
 * @param bufsize size of the already defined buffer.
 */
void pascal_string_init(struct s_pascal_string* ring, uint8_t* buf, uint8_t bufsize);


/** @brief If DMA is idle, it will be set to the "get pointer" of the uart_tx_ring.
 *
 * @param number_of_data Number of data bytes to DMA to send.
This number will update the "get pointer" to restart TX.
 */
void do_dma_usart_tx_ring(uint16_t number_of_data);


/**
 * @brief Inits the struct sring ring.
 *
 * @param ring pointer to struct sring.
 * @param buf pointer to already defined buffer.
 * @param buffer_size size of the already defined buffer.
 */
void ring_init(struct sring *ring, uint8_t *buf, uint16_t buffer_size);


/**
 * @brief Appends an ASCIIZ (uint8_t) string at the end of s_pascal_string buffer.
 * @param string_org pointer to ASCIIz string to to struct sring+
 * @param str_mount_buff pointer to struct sring.
 */
void string_append(uint8_t *string_org, struct s_pascal_string *str_mount_buff);


/**
 * @brief Puts a byte in the specified ring. It is a non blocking function.
 *
 * @param ring pointer to struct sring.
 * @param ch byte to put.
 * @return the effective number of bytes available in the specified ring.
 */
uint16_t ring_put_ch(struct sring* ring, uint8_t ch);


/**
 * @brief Send a ASCIIZ string to serial (up to 127 chars) to console buffer and starts sending. It is a non blocking function while there is room on TX Buffer.
 *
 * @param string pointer to string to send via console.
 */
void con_send_string(uint8_t* string);


/**
 * @brief It returns the number of availabe bytes in the specified ring. It is a non blocking function
 *
 * @param ring pointer to struct sring.
 * @return number of availabe bytes are available in the specified ring.
 */
uint16_t ring_avail_get_ch(struct sring* ring);


/**
 * @brief Used to verify the availability in the actual console buffer. It is a non blocking function
 *
 * @return the effective number of bytes are available in the actual console buffer.
 */
uint16_t con_available_get_char(void);


// If there is an available char in serial, it returns with an uint8_t, but
// before, you have to use con_available_get_char or ring_avail_get_ch to check their availability.
// They are non blocking functions.
/**
 * @brief If there is an available char in serial, it returns with an uint8_t. It is a non blocking function
 *
 * @param ring pointer to struct sring.
 * @param qty_in_buffer pointer of how many bytes are available to read in the specified ring.
 * @return the effective number of bytes
 */
uint8_t ring_get_ch(struct sring *ring, uint16_t *qty_in_buffer);


/**
 * @brief If there is an available char in console ring, it returns with an uint8_t. It is a non blocking function
 *
 * @return the effective number of bytes
 */
uint8_t con_get_char(void);


/**
 * @brief Read a line from console. It is a blocking function.
 *
 * @param s Pointer with the address to put reading.
 * @param len Maximum number of chars to read.
 * @return How many chars were read.
 */
uint8_t console_get_line(uint8_t *s, uint16_t len);


//Force next console reading ch
/**
 * @brief Forces console next reading ch. It is assumed that actual console buffer is empty.
 * @param ch char to buf buffer to copy data to
 */
void insert_in_con_rx(uint8_t ch);


/**
 * @brief To be used with printf.
 *
 * @param file file number
 * @param ptr pointer of char with the address of the string
 * @param  len length of the string to print
 * @return the effective number of bytes in the string or -1 when error.
 */
int _write(int file, char *ptr, int len);


/*Functions to convert strings*/

/**
 * @brief Convert a word (32 bit) into a up to 8 char string.
 *
 * @param value word (32 bit binary)
 * @param outstring address to where put the stringz result
 */
void conv_uint32_to_dec(uint32_t value, uint8_t *outstring);


/**
 * @brief Convert a two byte string pointed by i into a binary byte.
 *
 * @param instring address of the string.
 * @param i index of first byte (high nibble) into the string to be converted.
 * @return byte with the convertion.
 */
uint8_t conv_2a_hex_to_uint8(uint8_t *instring, int16_t i);


/**
 * @brief Convert a word (32 bit binary) to into a 8 char string. 
 *
 * @param value word (32 bit binary)
 * @param outstring address to where put the stringz result
 */
void conv_uint32_to_8a_hex(uint32_t value, uint8_t *outstring);


/**
 * @brief Convert a half-word (16 bit binary) to into a 4 char string.
 *
 * @param value half-word (16 bit binary) number to be converted
 * @param outstring address to where put the stringz result
 */
void conv_uint16_to_4a_hex(uint16_t value, uint8_t *outstring);


/**
 * @brief Convert a byte (8 bit binary) to into a 2 char string.
 *
 * @param value byte (8 bit binary) number to be converted
 * @param outstring address to where put the stringz result
 */
void conv_uint8_to_2a_hex(uint8_t value, uint8_t *outstring);


/**
 * @brief Check if uint16_t index is inside bounds.
 *
 * @param idx_u16 16 bit index to binary number to be converted
 * @param base_u32 address to where put the stringz result
 */
void check_idx_u16(uint16_t idx_u16, uintptr_t base_u32, uint16_t size);



#ifdef __cplusplus
}
#endif

#endif  //#ifndef SERIAL_H
