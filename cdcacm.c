/*
 * This file is part of the PS/2 to MSX keyboard Converter and 
 * MSX Keyboard Subsystem Emulator projects, using libopencm3 project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * Based on CDCACM example of:
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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


#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>

#include "system.h"
#include "cdcacm.h"
#include "usb_descriptors.h"
#include "hr_timer.h"
#include "serial_no.h"
#include "serial.h"


//Global variables
#if USE_USB == true
usbd_device *usb_dev;
int usb_configured;
bool nak_cleared[6];
uint8_t usbd_control_buffer[4 * USBD_DATA_BUFFER_SIZE];	// Buffer to be used for control requests.
extern struct sring con_tx_ring;									//Declared on serial.c
extern struct sring con_rx_ring;									//Declared on serial.c
#endif	//#if USE_USB == true
extern struct sring uart_tx_ring;									//Declared on serial.c
extern struct sring uart_rx_ring;									//Declared on serial.c
extern bool		enable_xon_xoff;										//Declared on serial.c
extern bool		xon_condition;											//Declared on serial.c
extern bool		xoff_condition;											//Declared on serial.c
extern bool		xonoff_sendnow;											//Declared on serial.c
extern bool		ok_to_rx;														//Declared on serial.c



#if USE_USB == true	
static const struct usb_cdc_line_coding line_coding = {
	.dwDTERate = 115200,
	.bCharFormat = USB_CDC_1_STOP_BITS,
	.bParityType = USB_CDC_NO_PARITY,
	.bDataBits = 8
};


static void usb_cdc_set_state(usbd_device *usbd_dev, const uint16_t iface, const uint8_t ep)
{
	uint8_t buf[10];
	struct usb_cdc_notification *notif = (void*)buf;
	/* We echo signals back to host as notification */
	notif->bmRequestType = 0xA1;
	notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notif->wValue = 0;
	notif->wIndex = iface;
	notif->wLength = 2;
	buf[8] = 3U;
	buf[9] = 0U;
	usbd_ep_write_packet(usbd_dev, ep, buf, sizeof(buf));
}


static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bmRequestType)
	{
		case SEND_ENCAPSULATED_COMMAND_bmRequestType:							//0x21
			if(req->bRequest == 0)
			{
				//send a RESPONSE_AVAILABLE notification
				uint8_t loc_buf[4];
				loc_buf[0] = 0x01;
				loc_buf[1] = 0;
				loc_buf[2] = 0;
				loc_buf[3] = 0;
				usbd_ep_write_packet(usbd_dev, EP_UART_COMM_IN, loc_buf, sizeof(uint32_t));
			}
		break;	//case SEND_ENCAPSULATED_COMMAND_bmRequestType:
		case GET_ENCAPSULATED_RESPONSE_bmRequestType:							//0xA1
			if(req->bRequest == GET_ENCAPSULATED_RESPONSE_bRequest) //0x01
			{
				usbd_ep_write_packet(usbd_dev, EP_UART_COMM_IN, &line_coding, sizeof(line_coding));
			}
		break;	//case GET_ENCAPSULATED_RESPONSE_bmRequestType:
		default:
		break;
	}
	switch (req->bRequest)
	{
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE: 		//0x22
			/*
			* This Linux cdc_acm driver requires this to be implemented
			* even though it's optional in the CDC spec, and we don't
			* advertise it in the ACM functional descriptor.
			*/

			/* We echo signals back to host as notification. */
			//usb_cdc_set_state(usbd_dev, INTF_CON_COMM, EP_CON_COMM_IN);
			usb_cdc_set_state(usbd_dev, INTF_UART_COMM, EP_UART_COMM_IN);
			return USBD_REQ_HANDLED;
		break;
		case USB_CDC_REQ_SET_LINE_CODING:						//0x20
			if (*len < sizeof(struct usb_cdc_line_coding))
				return USBD_REQ_NOTSUPP;
			switch(req->wIndex)
			{
				case 0:
					return USBD_REQ_NOTSUPP; // Ignore on CON Port
				break;
				case 2:
					usart_update_comm_param((struct usb_cdc_line_coding*)*buf);
					return USBD_REQ_HANDLED;
				break;
				default:
					return USBD_REQ_HANDLED;
				break;
			}
		break;
		case USB_CDC_REQ_GET_LINE_CODING:
			*buf = (uint8_t *)&line_coding;
			usbd_ep_write_packet(usbd_dev, EP_UART_COMM_IN, &line_coding, sizeof(line_coding));
			return USBD_REQ_HANDLED;
		break;
		default:
			return USBD_REQ_NOTSUPP;
		break;
	}
	return USBD_REQ_NOTSUPP;
}


static void cdcacm_con_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)usbd_dev;
	(void)ep;
	char buf_con[USBD_DATA_BUFFER_SIZE];
	uint16_t len_con, result = 0;

	len_con = (uint16_t)usbd_ep_read_packet(usbd_dev, EP_CON_DATA_OUT, buf_con, USBD_DATA_BUFFER_SIZE);
	if (len_con) 
	{
		for(uint16_t i = 0; (i < len_con) && (result != 0xFFFF); i++)
			while( (result = ring_put_ch(&con_rx_ring, buf_con[i])) == 0xFFFF) __asm("nop");
		if(result > (3 * con_rx_ring.bufSzMask / 4)) //X_OFF_TRIGGER
		{
			//Put EP in nak. On con_rx_ring read, check con_rx_ring room (X_ON_TRIGGER) to clear nak.
			nak_cleared[EP_CON_DATA_OUT] = false;
			usbd_ep_nak_set(usbd_dev, EP_CON_DATA_OUT, 1);
		}
	}
}


static void cdcacm_con_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)usbd_dev;
	(void)ep;
	char buf[USBD_DATA_BUFFER_SIZE];
	uint16_t i, len, max_transf, qty_accepted, local_getptr;

	len = QTTY_CHAR_IN(con_tx_ring);
	if(len)
	{
		max_transf = (len > (USBD_DATA_BUFFER_SIZE - 1)) ? (USBD_DATA_BUFFER_SIZE - 1) : len;
		local_getptr = con_tx_ring.get_ptr;

		CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
			// Transmit char through usb console.
			buf[0] = data;
			for(i = 1; i < max_transf; i++)
			{
				buf[i] = con_tx_ring.data[local_getptr++];
				local_getptr &= con_tx_ring.bufSzMask;
			}
		CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET
		else	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
		{
			for(i = 0; i < max_transf; i++)
			{
				buf[i] = con_tx_ring.data[local_getptr++];
				local_getptr &= con_tx_ring.bufSzMask;
			}
		}	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET

		qty_accepted = usbd_ep_write_packet(usb_dev, EP_CON_DATA_IN, buf, max_transf);
		//This following two passes warranties that con_tx_ring.get_ptr to be an atomic update.
		con_tx_ring.get_ptr = (con_tx_ring.get_ptr + qty_accepted) & con_tx_ring.bufSzMask;
	}
	else	//if(len)
	{	//As len = 0, set this EP to NAK requests, but first check XONXOFF SENDNOW
		CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
			// Put char in console ring.
			ring_put_ch(&con_tx_ring, data);
			buf[0] = data;
			usbd_ep_write_packet(usb_dev, EP_CON_DATA_IN, buf, 1);
		CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET	//CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
		else	//else CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
			usbd_ep_nak_set(usbd_dev, EP_CON_DATA_IN, 1);
	}
}


static void cdcacm_uart_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)usbd_dev;
	(void)ep;
	uint8_t buf_uart[USBD_DATA_BUFFER_SIZE + 1];
	uint16_t result;
	uint32_t len;

	len = (uint16_t)usbd_ep_read_packet(usbd_dev, EP_UART_DATA_OUT, (char*)buf_uart, USBD_DATA_BUFFER_SIZE);
	if (len && ok_to_rx)
	{
		for(uint16_t i = 0; (i < len) && (result != 0xFFFF); i++)
			while( (result = ring_put_ch(&uart_tx_ring, buf_uart[i])) == 0xFFFF)
				do_dma_usart_tx_ring(len);
		do_dma_usart_tx_ring(len);
		if(result > (3 * (uart_tx_ring.bufSzMask + 1) / 4))
		{
			//Put EP in nak. On uart_tx_ring read, check uart_rx_ring room (X_ON_TRIGGER) to clear nak.
			set_nak_endpoint(EP_UART_DATA_OUT);
		}	//if(result > (3 * (uart_tx_ring.bufSzMask + 1) / 4))
	}	//if (len)
}


static void cdcacm_uart_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)usbd_dev;
	(void)ep;
	char buf[USBD_DATA_BUFFER_SIZE];
	uint16_t i, len, max_transf, qty_accepted, local_getptr;

	len = QTTY_CHAR_IN(uart_rx_ring);
	if(len)
	{
		max_transf = (len > (USBD_DATA_BUFFER_SIZE - 1)) ? (USBD_DATA_BUFFER_SIZE - 1) : len;
		local_getptr = uart_rx_ring.get_ptr;
		for(i = 0; i < max_transf; i++)
		{
			buf[i] = uart_rx_ring.data[local_getptr++];
			local_getptr &= (uart_rx_ring.bufSzMask);
		}
		qty_accepted = usbd_ep_write_packet(usb_dev, EP_UART_DATA_IN, buf, max_transf);
		//This following two passes warranties that uart_rx_ring.get_ptr to be an atomic update.
		uart_rx_ring.get_ptr = (uart_rx_ring.get_ptr + qty_accepted) & (uart_rx_ring.bufSzMask);
	}
	/*else
		usbd_ep_nak_set(usbd_dev, EP_UART_DATA_OUT, 1);*/
}


static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	usb_configured = wValue;

	//Console interface
	usbd_ep_setup(usbd_dev, EP_CON_COMM_IN, USB_ENDPOINT_ATTR_INTERRUPT, COMM_PACKET_SIZE, NULL);
	usbd_ep_setup(usbd_dev, EP_CON_DATA_OUT, USB_ENDPOINT_ATTR_BULK, USBD_DATA_BUFFER_SIZE, cdcacm_con_data_rx_cb);
	usbd_ep_setup(usbd_dev, EP_CON_DATA_IN, USB_ENDPOINT_ATTR_BULK, USBD_DATA_BUFFER_SIZE, cdcacm_con_data_tx_cb);
	//UART interface
	usbd_ep_setup(usbd_dev, EP_UART_COMM_IN, USB_ENDPOINT_ATTR_INTERRUPT, COMM_PACKET_SIZE, NULL);
	usbd_ep_setup(usbd_dev, EP_UART_DATA_OUT, USB_ENDPOINT_ATTR_BULK, USBD_DATA_BUFFER_SIZE, cdcacm_uart_data_rx_cb);
	usbd_ep_setup(usbd_dev, EP_UART_DATA_IN, USB_ENDPOINT_ATTR_BULK, USBD_DATA_BUFFER_SIZE, cdcacm_uart_data_tx_cb);

	usbd_register_control_callback(usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);

	/* Notify the host that DCD is asserted.
	 * Allows the use of /dev/tty* devices on *BSD/MacOS
	 */
	usb_cdc_set_state(usbd_dev, INTF_CON_COMM, EP_CON_DATA_OUT);
	usb_cdc_set_state(usbd_dev, INTF_UART_COMM, EP_UART_DATA_OUT);
}


//It is used to start a communication pipe to allow the EP_??_DATA_IN callback continues send to host as needed.
//It returns ring->get_ptr updated.
void first_put_ring_content_onto_ep(struct sring *ring, uint8_t ep)
{
	if(usb_configured)
	{
		char buf[USBD_DATA_BUFFER_SIZE];
		uint16_t i, len, max_transf, qty_accepted, local_getptr;

		len = (ring->bufSzMask + 1 - ring->get_ptr + ring->put_ptr) & ring->bufSzMask;
		local_getptr = ring->get_ptr;	//update ring->get_ptr only at the end of the process
		if(len)
		{
			clear_nak_endpoint(ep);	//disable nak on ep
			max_transf = (len > (USBD_DATA_BUFFER_SIZE - 1)) ? (USBD_DATA_BUFFER_SIZE - 1) : len;
			for(i = 0; i < max_transf; i++)
			{
				buf[i] = ring->data[local_getptr++];
				local_getptr &= ring->bufSzMask;
			}
			qty_accepted = usbd_ep_write_packet(usb_dev, ep, buf, max_transf);
			//This following two passes warranties that ring->get_ptr to be an atomic update.
			ring->get_ptr = (ring->get_ptr + qty_accepted) & ring->bufSzMask;
		}
	}	//if(usb_configured)
}


void disable_usb(void)
{
	/* USB control register (USB_CNTR)
Bit 1 PDWN: Power down
This bit is used to completely switch off all USB-related analog parts if it is required to
completely disable the USB peripheral for any reason. When this bit is set, the USB
peripheral is disconnected from the transceivers and it cannot be used.
0: Exit Power Down.
1: Enter Power down mode.*/
#if MCU == STM32F103
	USB_CNTR_REG |= USB_CNTR_REG_PDWN;
#endif	//#if MCU == STM32F103
# if (MCU == STM32F401)
	// Explicitly disable DP pullup
	OTG_FS_DCTL |= OTG_DCTL_SDIS;
	rcc_periph_clock_disable(USB_RCC_CRC);
# endif	//# if (MCU == STM32F401)
	rcc_periph_clock_disable(USB_RCC_OTGFS);
}


static void usb_reset(void)
{
# if (MCU == STM32F103)
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (uint32_t i = 0; i < 0x500000; i++) __asm__("nop");
	gpio_set(GPIOA, GPIO12);
# endif	//# if (MCU == STM32F103)
# if (MCU == STM32F401)
	for (uint32_t i = 0; i < 0x500000; i++) __asm__("nop");
# endif	//# if (MCU == STM32F401)
}


void set_nak_endpoint(uint8_t ep)
{
	usbd_ep_nak_set(usb_dev, ep, 1);
	nak_cleared[ep] = false;
}


void clear_nak_endpoint(uint8_t ep)
{
	usbd_ep_nak_set(usb_dev, ep, 0);
	nak_cleared[ep] = true;
}


void cdcacm_init(void)
{
	usb_reset();

#if MCU == STM32F401
	// Enable peripherals
	//rcc_periph_clock_enable(USB_RCC_OTGFS);
	rcc_periph_clock_enable(USB_RCC_CRC);

	gpio_mode_setup(OTG_FS_DM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, OTG_FS_DM_PIN | OTG_FS_DP_PIN);
	gpio_set_af(OTG_FS_DM_PORT, GPIO_AF10, OTG_FS_DM_PIN | OTG_FS_DP_PIN);
#endif	//#if MCU == STM32F401

	usb_dev = usbd_init(&USB_DRIVER, &dev_desc, &config, usb_strings, NUM_STRINGS,
											usbd_control_buffer, sizeof(usbd_control_buffer));

#if MCU == STM32F401
	//Disable VBUS sensing => https://github.com/libopencm3/libopencm3/pull/1256#
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
#if USART_PORT == USART1
	//Now, if STM32F401 and USART1, recover A9 and A10 back to serial 1:
	gpio_set_af(GPIO_BANK_USART_TX, GPIO_AF7, GPIO_PIN_USART_TX | GPIO_PIN_USART_RX);
#endif	//#if USART_PORT == USART1
#endif	//#if MCU == STM32F401

	usbd_register_set_config_callback(usb_dev, cdcacm_set_config);

	nvic_set_priority(USB_NVIC, IRQ_PRI_USB);
	nvic_enable_irq(USB_NVIC);
}


/*************************************************************************************************/
/******************************************** ISR ************************************************/
/*************************************************************************************************/
USB_ISR
{
	usbd_poll(usb_dev);
}	//USB_ISR
#endif	//#if USE_USB == true


