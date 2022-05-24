/*
 * This file is part of the PS/2 Keyboard Adapter for MSX, using libopencm3 project.
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

 //Use Tab width=2

#pragma once

#ifndef SYSTEM_H
#define SYSTEM_H

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/cdc.h>

#include "webusb.h"
#include "winusb.h"
#include "usb21_standard.h"
#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Microcontroller STM32F103 or STM32F401 */
#define	STM32F103C8								0x410			//Blue Pill
#define	STM32F401CC								0x423			//WeAct Black Pill

#define	MCU												STM32F401CC

/* Define the usage of USB */
#define	USE_USB										true

//#define USB21_INTERFACE					true				//  Enable USB 2.1 with WebUSB and BOS support.
#define USB21_INTERFACE						false				//  Disable USB 2.1 with WebUSB and BOS support.

/* Turn on the usage the debug messages to UART */
#define	SERIAL_DEBUG							true

/* Turn on the usage of semihost debug */
	//  Warning: This code will trigger a breakpoint and hang unless a debugger is connected.
	//  That's how ARM Semihosting sends a command to the debugger to print a message.
	//  This code MUST be disabled on production devices.

	//  To see the message you need to run opencd:
	//    openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -f scripts/connect.ocd
//#ifndef SEMIHOST_DEBUG
//#define	SEMIHOST_DEBUG
//#endif

/* Here we define how PS/2 Clock Interrupts are taken */
#	define	TIMxCC1_INT							0
#	define	GPIO_INT								1
#ifndef PS2_CLK_INTERRUPT
#if MCU == STM32F103C8
#define	PS2_CLK_INTERRUPT					TIMxCC1_INT
#endif	//#if MCU == STM32F103C8
#if MCU == STM32F401CC
#define	PS2_CLK_INTERRUPT					GPIO_INT		//TODO: Use TIM2 CCP, like on Blue Pill
#endif	//#if MCU == STM32F401CC
#endif	//#ifndef PS2_CLK_INTERRUPT

//Used to calculate timeout to user interact with PS2-MSX Console,
//to avoid locking in automatic reset procedures.
#define	FREQ_INT_SYSTICK					30					//Freq in Hz
#define	MAX_TIMEOUT2RX_INTEL_HEX	11					//Timeout in sec to user to start sending Intel Hex
#define	MAX_TIMEOUT2AMPERSAND			6						//Timeout in sec to user to confirm Database erasure

// Place to get the microcontroller unique id to compute serial number
#ifndef DESIG_UNIQ_ID_BASE
#if MCU == STM32F103C8
#define	DESIG_UNIQ_ID_BASE				0x1FFFF7E8;
#define	LEN_SERIAL_No							8
#endif
#if MCU == STM32F401CC
#define	DESIG_UNIQ_ID_BASE				0x1FFF7A10;
#define	LEN_SERIAL_No							12
#endif	//#if MCU == STM32F401CC
#endif	//#ifndef DESIG_UNIQ_ID_BASE

/* High Resolution Timer definitions */
#define TIM_HR_TIMER							TIM2				//Here we define which timer we will use for hrtimer
#if TIM_HR_TIMER == TIM2
#define RCC_TIMX									RCC_TIM2
#define RST_TIMX									RST_TIM2
#define NVIC_TIMX_IRQ							NVIC_TIM2_IRQ
#endif	//#if TIM_HR_TIMER == TIM2
#if TIM_HR_TIMER == TIM3
#define RCC_TIMX									RCC_TIM3
#define RST_TIMX									RST_TIM3
#define NVIC_TIMX_IRQ							NVIC_TIM3_IRQ
#endif	//#if TIM_HR_TIMER == TIM3
#if TIM_HR_TIMER == TIM4
#define RCC_TIMX									RCC_TIM4
#define RST_TIMX									RST_TIM4
#define NVIC_TIMX_IRQ							NVIC_TIM4_IRQ
#endif	//#if TIM_HR_TIMER == TIM4

/* Database related informations */
#define DB_NUM_COLS								8						// Number of colunms of each line of the Database
#define N_DATABASE_REGISTERS			320
#define DATABASE_SIZE 						N_DATABASE_REGISTERS * DB_NUM_COLS	//It will resolve 2560

/* Interrupt priority definitions. Low numbers are high priority.
 * IRQ_PRI_PS2_CLOCK and IRQ_PRI_Y_SCAN share the highest priority.
 */
#define IRQ_PRI_PS2_CLOCK					(1 << 4)
#define IRQ_PRI_Y_SCAN						(1 << 4)
#define IRQ_PRI_USB								(3 << 4)
#define IRQ_PRI_TIM								(4 << 4)
#define IRQ_PRI_USART							(5 << 4)
#define IRQ_PRI_SYSTICK						(9 << 4)

#define X_ON											17
#define X_OFF 										19

/* Port allocation */
/* Start of STM32F103C8 Port allocation */
#if MCU == STM32F103C8			/* STM32F103C8 Port allocation */
#define	USE_USB										false		//STM32F103C8 does NOT have enough 5V tolerant gpio pins to have USB implemented
#define USART_PORT 								USART2
#define EMBEDDED_BLUE_LED_PORT		GPIOC
#define EMBEDDED_BLUE_LED_PIN			GPIO13
#define X7_port										GPIOB
#define X7_pin_id									GPIO9
#define X6_port										GPIOB
#define X6_pin_id									GPIO8
#define X5_port										GPIOB
#define X5_pin_id									GPIO10
#define X4_port										GPIOB
#define X4_pin_id									GPIO15
#define X3_port										GPIOB
#define X3_pin_id									GPIO11
#define X2_port										GPIOB
#define X2_pin_id									GPIO14
#define X1_port										GPIOB
#define X1_pin_id									GPIO13
#define X0_port										GPIOB
#define X0_pin_id									GPIO12
#define Y3_port										GPIOA
#define Y3_pin_id									GPIO11
#define Y3_exti										EXTI11
//#define Y2_port									GPIOA
#define Y2_port										GPIOB		//port A10 broken
//#define Y2_pin_id								GPIO10
#define Y2_pin_id									GPIO4		//port A10 broken
#define Y2_exti										EXTI4
#define Y1_port										GPIOA
#define Y1_pin_id									GPIO9
#define Y1_exti										EXTI9
#define Y0_port										GPIOA
#define Y0_pin_id									GPIO8
#define Y0_exti										EXTI8
#define CAPSLOCK_port							GPIOA
#define CAPSLOCK_pin_id						GPIO12
#define CAPSLOCK_exti							EXTI12
#define KANA_port									GPIOB
#define KANA_pin_id								GPIO6
#define KANA_exti									EXTI6

#define PS2_DATA_PIN_PORT					GPIOB
#define PS2_DATA_PIN_ID						GPIO7
#define PS2_CLOCK_PIN_PORT				GPIOA
#define PS2_CLOCK_PIN_ID					GPIO15
#define PS2_POWER_CTR_PORT				GPIOB	
#define PS2_POWER_CTR_PIN					GPIO0	

//Debug facilities
#define SYSTICK_port							GPIOA
#define SYSTICK_pin_id						GPIO0
#define TIM2CC1_pin_id						GPIO1
#define BIT0_pin_port							GPIOA
#define BIT0_pin_id								GPIO4
#define INT_TIM2_port							GPIOA
#define TIM2UIF_pin_id						GPIO5
#define Dbg_Yint_port							GPIOA
#define Dbg_Yint0e1_pin_id				GPIO6
#define Dbg_Yint2e3_pin_id				GPIO7

//Available resources
#define AVAILABLE_A3_PORT					GPIOA
#define AVAILABLE_A3_PIN_ID				GPIO3
#define AVAILABLE_B3_PORT					GPIOB
#define AVAILABLE_B3_PIN_ID				GPIO3

//Layout relationship definitions. 
//The following numbers were computed at "Pins From-To Rev 211009.xlsx"
#define X7_SET_AND								0xFDFFFFFF  // 4261412863
#define X6_SET_AND								0xFEFFFFFF  // 4278190079
#define X5_SET_AND								0xFBFFFFFF  // 4227858431
#define X4_SET_AND								0xF7FFFFFF  // 4160749567
#define X3_SET_AND								0x7FFFFFFF  // 2147483647
#define X2_SET_AND								0xBFFFFFFF  // 3221225471
#define X1_SET_AND								0xDFFFFFFF  // 3758096383
#define X0_SET_AND								0xEFFFFFFF  // 4026531839
#define X7_CLEAR_OR								0x02000000  // 33554432
#define X6_CLEAR_OR								0x01000000  // 16777216
#define X5_CLEAR_OR								0x04000000  // 67108864
#define X4_CLEAR_OR								0x08000000  // 134217728
#define X3_CLEAR_OR								0x80000000  // 2147483648
#define X2_CLEAR_OR								0x40000000  // 1073741824
#define X1_CLEAR_OR								0x20000000  // 536870912
#define X0_CLEAR_OR								0x10000000  // 268435456
#define X7_CLEAR_AND							0xFFFFFDFF  // 4294966783
#define X6_CLEAR_AND							0xFFFFFEFF  // 4294967039
#define X5_CLEAR_AND							0xFFFFFBFF  // 4294966271
#define X4_CLEAR_AND							0xFFFFF7FF  // 4294965247
#define X3_CLEAR_AND							0xFFFF7FFF  // 4294934527
#define X2_CLEAR_AND							0xFFFFBFFF  // 4294950911
#define X1_CLEAR_AND							0xFFFFDFFF  // 4294959103
#define X0_CLEAR_AND							0xFFFFEFFF  // 4294963199
#define X7_SET_OR									0x00000200  // 512
#define X6_SET_OR									0x00000100  // 256
#define X5_SET_OR									0x00000400  // 1024
#define X4_SET_OR									0x00000800  // 2048
#define X3_SET_OR									0x00008000  // 32768
#define X2_SET_OR									0x00004000  // 16384
#define X1_SET_OR									0x00002000  // 8192
#define X0_SET_OR									0x00001000  // 4096

#endif	//#if MCU == STM32F103C8			/* STM32F103C8 Port allocation */
/* End of STM32F103C8 Port allocation */

/* Start of STM32F401CC Port allocation */
#if MCU == STM32F401CC			/* STM32F401CC Port allocation */
#define USART_PORT 								USART1
#if USE_USB == true
#define CONSOLE_OVER_SERIAL				true				//Here the programmer can change definition
#else		//#if USE_USB == true
#define CONSOLE_OVER_SERIAL				true				//ALWAYS use CONSOLE_OVER_SERIAL=true when not using USB
#endif	//#if USE_USB == true
#define EMBEDDED_BLUE_LED_PORT		GPIOC
#define EMBEDDED_BLUE_LED_PIN			GPIO13
#define X_port										GPIOB
#define X7_pin_id									GPIO0
#define X6_pin_id									GPIO15
#define X5_pin_id									GPIO1
#define X4_pin_id									GPIO14
#define X3_pin_id									GPIO3
#define X2_pin_id									GPIO13
#define X1_pin_id									GPIO10
#define X0_pin_id									GPIO12
#define Y3_port										GPIOA
#define Y3_pin_id									GPIO5
#define Y3_exti										EXTI5
#define Y2_port										GPIOA
#define Y2_pin_id									GPIO6
#define Y2_exti										EXTI6
#define Y1_port										GPIOA
#define Y1_pin_id									GPIO7
#define Y1_exti										EXTI7
#define Y0_port										GPIOA
#define Y0_pin_id									GPIO8
#define Y0_exti										EXTI8
#define CAPSLOCK_PORT							GPIOB
#define CAPSLOCK_PIN_ID						GPIO4
#define CAPSLOCK_exti							EXTI4
#define KANA_PORT									GPIOB
#define KANA_PIN_ID								GPIO6
#define KANA_exti									EXTI6

#define Y_MASK										0x1E0				//Valid bits: 16, 15, 14 and 13.

#define PS2_DATA_PIN_PORT					GPIOB
#define PS2_DATA_PIN_ID						GPIO5
#define PS2_CLOCK_PIN_PORT				GPIOA
#define PS2_CLOCK_PIN_ID					GPIO15
#define PS2_CLOCK_PIN_EXTI				EXTI15
#define PS2_POWER_CTR_PORT				GPIOA
#define PS2_POWER_CTR_PIN					GPIO4	
#define USB_FS_PORT								GPIOA
#define USB_FS_DM_PIN_ID	 				GPIO11			//USB-
#define USB_FS_DP_PIN_ID					GPIO12			//USB+


//Debug facilities
#define USER_KEY_PORT							GPIOA
#define USER_KEY_PIN_ID						GPIO0
#define INT_PS2_PIN_PORT					GPIOA
#define INT_PS2_PIN_ID						GPIO1
#define PS2_START_SEND_PORT				GPIOA
#define PS2_START_SEND_PIN_ID			GPIO2
#define Dbg_Yint_port							GPIOB
#define Dbg_Yint_pin_id						GPIO7
#define TIMxUIF_PORT							GPIOB
#define TIMxUIF_PIN_ID						GPIO8
#define TIMxCC1_PORT							GPIOB
#define TIMxCC1_PIN_ID						GPIO9

//Available resources
#define AVAILABLE_A3_PORT					GPIOA
#define AVAILABLE_A3_PIN_ID				GPIO3
#define AVAILABLE_B2_PORT					GPIOB
#define AVAILABLE_B2_PIN_ID				GPIO2				//Boot1 pin (There is a R=10K to GND)

//Layout relationship definitions
//The following numbers were computed at "Pins From-To Rev 211009.xlsx"
#define X7_SET_AND								0xFFFEFFFF  // 4294901759
#define X6_SET_AND								0x7FFFFFFF  // 2147483647
#define X5_SET_AND								0xFFFDFFFF  // 4294836223
#define X4_SET_AND								0xBFFFFFFF  // 3221225471
#define X3_SET_AND								0xFFF7FFFF  // 4294443007
#define X2_SET_AND								0xDFFFFFFF  // 3758096383
#define X1_SET_AND								0xFBFFFFFF  // 4227858431
#define X0_SET_AND								0xEFFFFFFF  // 4026531839
#define X7_CLEAR_OR								0x00010000  // 65536
#define X6_CLEAR_OR								0x80000000  // 2147483648
#define X5_CLEAR_OR								0x00020000  // 131072
#define X4_CLEAR_OR								0x40000000  // 1073741824
#define X3_CLEAR_OR								0x00080000  // 524288
#define X2_CLEAR_OR								0x20000000  // 536870912
#define X1_CLEAR_OR								0x04000000  // 67108864
#define X0_CLEAR_OR								0x10000000  // 268435456
#define X7_CLEAR_AND							0xFFFFFFFE  // 4294967294
#define X6_CLEAR_AND							0xFFFF7FFF  // 4294934527
#define X5_CLEAR_AND							0xFFFFFFFD  // 4294967293
#define X4_CLEAR_AND							0xFFFFBFFF  // 4294950911
#define X3_CLEAR_AND							0xFFFFFFF7  // 4294967287
#define X2_CLEAR_AND							0xFFFFDFFF  // 4294959103
#define X1_CLEAR_AND							0xFFFFFBFF  // 4294966271
#define X0_CLEAR_AND							0xFFFFEFFF  // 4294963199
#define X7_SET_OR									0x00000001  // 1
#define X6_SET_OR									0x00008000  // 32768
#define X5_SET_OR									0x00000002  // 2
#define X4_SET_OR									0x00004000  // 16384
#define X3_SET_OR									0x00000008  // 8
#define X2_SET_OR									0x00002000  // 8192
#define X1_SET_OR									0x00000400  // 1024
#define X0_SET_OR									0x00001000  // 4096
#endif	//#if MCU == STM32F401CC	/* STM32F401CC Port allocation */
/* End of STM32F401CC Port allocation */


//#define MMIO32(addr)						(*(volatile uint32_t *)(addr))
#define DBGMCU_APB1_FZ						MMIO32(DBGMCU_BASE+8)
#define DBG_TIM2_STOP							1<<0
#define DBG_TIM3_STOP							1<<1
#define DBG_TIM4_STOP							1<<2
#define DBG_TIM5_STOP							1<<3
#define DBG_RTC_STOP							1<<10
#define DBG_WWDG_STOP							1<<11
#define DBG_IWDG_STOP							1<<12

/* USB related definitions */
#if !defined(CDC_ONLY_ON_USB)
#define CDC_ONLY_ON_USB						false				//  If we are providing serial interfaces only...
#endif	//#if !defined(CDC_ONLY_ON_USB)

#if USE_USB == true
#if !defined(USB_DRIVER)
#if MCU == STM32F401CC
#define USB_DRIVER      					otgfs_usb_driver
#endif	//#if MCU == STM32F401CC
#if MCU == STM32F103C8												//It is here for didatic purposes only, as STM32F103C8 don't have enough 5V tolerant pins to support a working USB in this application
#define USB_DRIVER     						stm32f107_usb_driver
#endif	//#if MCU == STM32F103C8
#endif	//#if !defined(USB_DRIVER)

#define OTG_DCTL									0x804
#define OTG_FS_DCTL								MMIO32(USB_OTG_FS_BASE + OTG_DCTL)
#define OTG_FS_DCTL_SDIS					2						//0: Normal operation. 2: The core generates a device disconnect event to the USB host.
#define OTG_FS_DSTS								MMIO32(USB_OTG_FS_BASE + OTG_DSTS)
#define OTG_FS_DSTS_SUSPSTS				1<<0				//0: Suspend condition is detected on the USB. 1: Normal operation.
#endif	//#if USE_USB == true

#define USB_CLASS_MISCELLANEOUS 	0xEF  //  Copy from Blue Pill Bootloader

#define	USB_VID										0x0483
#define	USB_PID										0x5740
//#define	USB_VID									0x0ACE			//ZyXEL Omni FaxModem 56K Plus
//#define	USB_PID									0x1611			//ZyXEL Omni FaxModem 56K Plus
//#define	USB_VID									0x0A05			//Same VID as my USB hub			
//#define	USB_PID									0x5740			//Ex.: My hub is 0x7211


/*  Index of each USB interface. Must be consecutive and must sync with interfaces[].*/
enum INTF{
	INTF_CON_COMM =									0,
	INTF_CON_DATA,
	INTF_UART_COMM,
	INTF_UART_DATA,
};

//  USB Endpoints addresses
enum ENDPOINT{
	CON_DATA_OUT		=								1,
	CON_DATA_IN			=								CON_DATA_OUT	| USB_REQ_TYPE_IN,
	CON_COMM				=								2,
	UART_DATA_OUT		=								3,
	UART_DATA_IN		=								UART_DATA_OUT	| USB_REQ_TYPE_IN,
	UART_COMM				=								4,
};

#define COMM_PACKET_SIZE					32
#define MAX_USB_PACKET_SIZE				2 * COMM_PACKET_SIZE
#define USBD_CONTROL_BUFFER_SIZE	MAX_USB_PACKET_SIZE

#define GET_STATUS_bReq						0x00				// Device/Interface/Endpoint Status
#define CLEAR_FEATURE_bReq				0x01				// Device/Interface/Endpoint 
#define SET_FEATURE_bReq					0x03				// Device/Interface/Endpoint 
#define SET_ADDRESS_bReq					0x05				// Device None
#define GET_DESCRIPTOR_bReq				0x06				// Device None
#define SET_DESCRIPTOR_bReq				0x07				// Device None
#define GET_CONFIGURATION_bReq		0x08				// Device Descriptor
#define SET_CONFIGURATION_bReq		0x09				// Device Descriptor
#define GET_INTERFACE_bReq				0x0A				// Interface Alternate Interface
#define SET_INTERFACE_bReq				0x11				// Interface None
#define SYNCH_FRAME_bReq					0x12				// Endpoint FrameNumber

#define DEV_OUT_bmReqTyp					0x00
#define DEV_IN_bmReqTyp						0x80
#define INTF_OUT_bmReqTyp 				0x01
#define INTF_IN_bmReqTyp					0x81
#define ENDP_OUT_bmReqTyp					0x02
#define ENDP_IN_bmReqTyp					0x82



#ifdef __cplusplus
}
#endif	//#ifdef __cplusplus

#endif	//#ifndef SYSTEM_H
