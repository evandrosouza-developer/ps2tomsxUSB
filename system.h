/** @defgroup Manager_group Main
 *
 * @ingroup Main_design_definitions
 *
 * @file system.h System main definitions of the project+
 *
 * @brief <b>System main definitions of the project. Header file of tester-ps2-msx.cpp.</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 01 September 2022
 *
 * This file has the main definitions about the design. I supports both the 
 * STM32F4 and STM32F1 series of ARM Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms ref lgpl_license
 */

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

#ifdef __cplusplus
extern "C" {
#endif

#if !defined T_SYSTEM_H
#define T_SYSTEM_H


#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_fs.h>


/* Microcontroller STM32F103 or STM32F401 */
#define STM32F103                 0x410     //Blue Pill
#define STM32F401                 0x423     //WeAct MiniF4 Black Pill

#define MCU                       STM32F401

// Place to get the microcontroller unique id to compute serial number
#ifndef DESIG_UNIQ_ID_BASE
#if MCU == STM32F103
#define DESIG_UNIQ_ID_BASE        0x1FFFF7E8;
#define LEN_SERIAL_No             8
#endif  //#if MCU == STM32F103
#if MCU ==STM32F401
#define DESIG_UNIQ_ID_BASE        0x1FFF7A10;
#define LEN_SERIAL_No             12
#endif  //#if MCU ==STM32F401
#endif  //#ifndef DESIG_UNIQ_ID_BASE

/* High Resolution Timer definitions */
#define TIM_HR                    TIM2        //Here we define which timer we will use for hrtimer
#if TIM_HR == TIM2
#define RCC_TIM_HR                RCC_TIM2
#define RST_TIM_HR                RST_TIM2
#define NVIC_TIM_HR_IRQ           NVIC_TIM2_IRQ
#define ISR_TIM_HR                void tim2_isr(void)
#endif  //#if TIM_HR_TIMER == TIM2

/* Interrupt priority definitions. From 0 to 15. Low numbers are high priority.*/
/** Interrupt priority definitions.
 *
 * From 0 to 15. Low numbers are high priority.
@{*/
#define IRQ_PRI_TIM_HR            (1 << 4)
#define IRQ_PRI_SYSTICK           (2 << 4)
#define IRQ_PRI_USB               (3 << 4)
#define IRQ_PRI_USART             (5 << 4)
#define IRQ_PRI_USART_DMA         (5 << 4)
/**@}*/

/* General definitions about MSX keyboard scan control */
/** General definitions about MSX keyboard scan control
 *
*/
#define SCAN_POINTER_SIZE         18
#define DELAY_TO_READ_SIZE        14
#define INIT_SCAN_POINTER         15
#define INIT_DELAY_TO_READ_X_SCAN 6

/* STM32F103 Hardware port definitions */
#if MCU == STM32F103
#define	HARDWARE_BASE             "STM32F103C6T6 (Blue Pill) "
#define USART_PORT                USART2
#define EMBEDDED_LED_port         GPIOC
#define EMBEDDED_LED_pin          GPIO13
#define X7_port                   GPIOB
#define X7_pin_id                 GPIO15
#define X7_exti                   EXTI15
#define X6_port                   GPIOB
#define X6_pin_id                 GPIO14
#define X6_exti                   EXTI14
#define X5_port                   GPIOB
#define X5_pin_id                 GPIO13
#define X5_exti                   EXTI13
#define X4_port                   GPIOB
#define X4_pin_id                 GPIO12
#define X4_exti                   EXTI12
#define X3_port                   GPIOB
#define X3_pin_id                 GPIO9
#define X3_exti                   EXTI9
#define X2_port                   GPIOB
#define X2_pin_id                 GPIO8
#define X2_exti                   EXTI8
#define X1_port                   GPIOB
#define X1_pin_id                 GPIO7
#define X1_exti                   EXTI7
#define X0_port                   GPIOB
#define X0_pin_id                 GPIO6
#define X0_exti                   EXTI6
#define Y_port                    GPIOA
#define Y_pin_id                  GPIO0
#define Y3_port                   GPIOA
#define Y3_pin_id                 GPIO7
#define Y2_port                   GPIOA
#define Y2_pin_id                 GPIO6
#define Y1_port                   GPIOA
#define Y1_pin_id                 GPIO5
#define Y0_port                   GPIOA
#define Y0_pin_id                 GPIO4
#define CAPS_port                 GPIOB
#define CAPS_pin_id               GPIO4
#define KANA_port                 GPIOB
#define KANA_pin_id               GPIO5
#define OTG_FS_DM_PORT            GPIOA
#define OTG_FS_DM_PIN             GPIO11
#define OTG_FS_DP_PORT            GPIOA
#define OTG_FS_DP_PIN             GPIO12
//SERIAL1_port                    GPIOA (Pre-defined)
//SERIAL1_TX_pin_id               GPIO9 (Pre-defined)
//SERIAL1_RX_pin_id               GPIO10(Pre-defined)
//SERIAL2_port                    GPIOA (Pre-defined)
//SERIAL2_TX_pin_id               GPIO2 (Pre-defined)
//SERIAL2_RX_pin_id               GPIO3 (Pre-defined)
//SERIAL3_port                    GPIOB (Pre-defined)
//SERIAL3_TX_pin_id               GPIO10(Pre-defined)
//SERIAL3_RX_pin_id               GPIO11(Pre-defined)

//Force Y_port_id pin (Sync pin) to high, so the first time slot is a low => Falling transition on the start of frame
//Force Xint_pin_id to low. portXread in msxmap.cpp will put this in high at each port update: to be possible mesaure real time of reading.
#define Y_0                       0x00F20001  // 15859713
#define Y_1                       0x00E20011  // 14811153
#define Y_2                       0x00D20021  // 13762593
#define Y_3                       0x00C20031  // 12714033
#define Y_4                       0x00B20041  // 11665473
#define Y_5                       0x00A20051  // 10616913
#define Y_6                       0x00920061  // 9568353
#define Y_7                       0x00820071  // 8519793
#define Y_8                       0x00720081  // 7471233
#define Y_9                       0x00620091  // 6422673
#define Y_A                       0x005200A1  // 5374113
#define Y_B                       0x004200B1  // 4325553
#define Y_C                       0x003200C1  // 3276993
#define Y_D                       0x002200D1  // 2228433
#define Y_E                       0x001200E1  // 1179873
#define Y_F                       0x000200F1  // 131313

//To debug
#define Xint_port                 GPIOA
#define Xint_pin_id               GPIO1
#define INT_TIM2_port             GPIOB
#define TIM2UIF_pin_id            GPIO0
#define SYSTICK_port              GPIOB
#define SYSTICK_pin_id            GPIO1

#endif  //#if MCU == STM32F103


/* STM32F401 Hardware port definitions */
#if MCU == STM32F401
#define	HARDWARE_BASE             "STM32F401CCU6 miniF4 (Black Pill v2.0+) "
#define USART_PORT                USART1
#define EMBEDDED_LED_port         GPIOC
#define EMBEDDED_LED_pin          GPIO13
#define X7_port                   GPIOB
#define X7_pin_id                 GPIO0
#define MSX_X_BIT7                0
#define X6_port                   GPIOB
#define X6_pin_id                 GPIO15
#define MSX_X_BIT6                15
#define X5_port                   GPIOB
#define X5_pin_id                 GPIO1
#define MSX_X_BIT5                1
#define X4_port                   GPIOB
#define X4_pin_id                 GPIO14
#define MSX_X_BIT4                14
#define X3_port                   GPIOB
#define X3_pin_id                 GPIO3
#define MSX_X_BIT3                3
#define X2_port                   GPIOB
#define X2_pin_id                 GPIO13
#define MSX_X_BIT2                13
#define X1_port                   GPIOB
#define X1_pin_id                 GPIO10
#define MSX_X_BIT1                10
#define X0_port                   GPIOB
#define X0_pin_id                 GPIO12
#define MSX_X_BIT0                12
#define Y_port                    GPIOA
#define Y_pin_id                  GPIO4
#define Y3_port                   GPIOA
#define Y3_pin_id                 GPIO5
#define Y2_port                   GPIOA
#define Y2_pin_id                 GPIO6
#define Y1_port                   GPIOA
#define Y1_pin_id                 GPIO7
#define Y0_port                   GPIOA
#define Y0_pin_id                 GPIO8
#define CAPS_port                 GPIOB
#define CAPS_pin_id               GPIO4
#define KANA_port                 GPIOB
#define KANA_pin_id               GPIO6
#define OTG_FS_DM_PORT            GPIOA
#define OTG_FS_DM_PIN             GPIO11
#define OTG_FS_DP_PORT            GPIOA
#define OTG_FS_DP_PIN             GPIO12
//SERIAL2_port                    GPIOA (Pre-defined)
//SERIAL2_TX_pin_id               GPIO2 (Pre-defined)
//SERIAL2_RX_pin_id               GPIO3 (Pre-defined)
//SERIAL3_port                    GPIOB (Pre-defined)
//SERIAL3_TX_pin_id               GPIO10(Pre-defined)
//SERIAL3_RX_pin_id               GPIO11(Pre-defined)

//Force Y_port_id pin (Sync pin) to high, so the first time slot is a low => Falling transition on the start of frame
//Force Xint_pin_id to low. portXread in msxmap.cpp will put this in high at each port update: to be possible mesaure real time of reading.
#define Y_0                       0x01E20010  // 31588368
#define Y_1                       0x00E20110  // 14811408
#define Y_2                       0x01620090  // 23199888
#define Y_3                       0x00620190  // 6422928
#define Y_4                       0x01A20050  // 27394128
#define Y_5                       0x00A20150  // 10617168
#define Y_6                       0x012200D0  // 19005648
#define Y_7                       0x002201D0  // 2228688
#define Y_8                       0x01C20030  // 29491248
#define Y_9                       0x00C20130  // 12714288
#define Y_A                       0x014200B0  // 21102768
#define Y_B                       0x004201B0  // 4325808
#define Y_C                       0x01820070  // 25297008
#define Y_D                       0x00820170  // 8520048
#define Y_E                       0x010200F0  // 16908528
#define Y_F                       0x000201F0  // 131568

//To debug
#define Xint_port                 GPIOA
#define Xint_pin_id               GPIO1
#define INT_TIM2_port             GPIOB
#define TIM2UIF_pin_id            GPIO7
#define SYSTICK_port              GPIOB
#define SYSTICK_pin_id            GPIO8

#endif  //#if MCU == STM32F401



/* USB related definitions */

/* Define the usage of USB */
#define USE_USB                   true

#define CDC_ONLY_ON_USB           false     // If we are providing serial interfaces only => True

#if USE_USB == true
#if MCU == STM32F103
#define USB_DRIVER                st_usbfs_v1_usb_driver
#define USB_ISR                   void usb_lp_can_rx0_isr(void)
#define USB_NVIC                  NVIC_USB_LP_CAN_RX0_IRQ
#define USB_RCC_OTGFS             RCC_OTGFS
#endif  //#if MCU == STM32F103C8
#if MCU == STM32F401
#define USB_DRIVER                otgfs_usb_driver
#define USB_ISR                   void otg_fs_isr(void)
#define USB_NVIC                  NVIC_OTG_FS_IRQ
#define USB_RCC_OTGFS             RCC_OTGFS
#define USB_RCC_CRC               RCC_CRC
#endif  //#if MCU == STM32F401CC

#define OTG_DCTL                  0x804
#define OTG_FS_DCTL               MMIO32(USB_OTG_FS_BASE + OTG_DCTL)
#define OTG_FS_DCTL_SDIS          1<<1        //0: Normal operation. 2: The core generates a device disconnect event to the USB host.
#define OTG_FS_DSTS               MMIO32(USB_OTG_FS_BASE + OTG_DSTS)  
#define OTG_FS_DSTS_SUSPSTS       1<<0        //0: Suspend condition is detected on the USB. 1: Normal operation.
/* USB Control register */
#if MCU == STM32F103
#define USB_CNTR_REG              MMIO32(USB_DEV_FS_BASE + 0x40)
#define USB_CNTR_REG_PDWN         1<<1        //0: Exit Power Down. 1: Enter Power down mode.
#endif  //#if MCU == STM32F103

#define USB_CLASS_MISCELLANEOUS   0xEF  //  Idea taked from Blue Pill Bootloader
#define USB_CDC_REQ_GET_LINE_CODING 0x21 // Not defined in libopencm3
#define SEND_ENCAPSULATED_COMMAND_bmRequestType 0x21//Get in https://docs.microsoft.com/en-us/windows-hardware/drivers/network/control-channel-characteristics
#define GET_ENCAPSULATED_RESPONSE_bmRequestType 0xA1//Get in https://docs.microsoft.com/en-us/windows-hardware/drivers/network/control-channel-characteristics
#define SEND_ENCAPSULATED_COMMAND_bRequest 0  //Get in https://docs.microsoft.com/en-us/windows-hardware/drivers/network/control-channel-characteristics
#define GET_ENCAPSULATED_RESPONSE_bRequest 1  //Get in https://docs.microsoft.com/en-us/windows-hardware/drivers/network/control-channel-characteristics
#undef  USB_REQ_TYPE_IN                       //To avoid erros in Visual Studio Code. It is already defined in libopencm3.
#define USB_REQ_TYPE_IN           0x80        //To avoid erros in Visual Studio Code

#define USB_VID                   0x1d50      //OpenMoko
#define USB_PID                   0x5740      //ST CDC from various forums

#endif  //#if USE_USB == true

#define DESIGN_DEF                "MSX keyboard subsystem emulator "

/* Index of each USB interface. Must be consecutive and must sync with interfaces[]. */
/**  Index of each USB interface.
 *
 * Must be consecutive and must sync with interfaces[].
 @enum INTF Index of each USB interface. Must be consecutive and must sync with interfaces[].*/
enum INTF{
  INTF_CON_COMM =                 0,
  INTF_CON_DATA,
  INTF_UART_COMM,
  INTF_UART_DATA,
};


/* USB Endpoints addresses. Starts with 1, as endpoint 0 is the default. */
/**  USB Endpoints addresses.
 *
 * Starts with 1, as endpoint 0 is the default.
 @enum ENDPOINT USB Endpoints addresses. Starts with 1, as endpoint 0 is the default.*/
enum ENDPOINT{
  EP_CON_DATA_OUT =               1,          //CDC Data OUT of FIRST endpoint
  EP_CON_COMM_OUT,                            //CDC Command of FIRST endpoint: uses this as +0x80
  EP_UART_DATA_OUT,                           //CDC Data OUT of SECOND endpoint
  EP_UART_COMM_OUT,                           //CDC Command of SECOND endpoint: uses this as +0x80
  //CDC Data IN of First endpoint.
  EP_CON_DATA_IN  =               EP_CON_DATA_OUT | USB_REQ_TYPE_IN,
  EP_CON_COMM_IN,                             //First endpoint: valid add CDC Command 
  EP_UART_DATA_IN,                            //CDC Data IN of Second endpoint
  EP_UART_COMM_IN,                            //Second endpoint: CDC Command
};

/**  USB buffers sizes.
 *
@{*/
#define MAX_USB_PACKET_SIZE       64
#define COMM_PACKET_SIZE          MAX_USB_PACKET_SIZE / 4
#define USBD_DATA_BUFFER_SIZE     MAX_USB_PACKET_SIZE
/**@}*/

//#define USB21_INTERFACE         true        //  Enable USB 2.1 with WebUSB and BOS support.
#define USB21_INTERFACE           false       //  Disable USB 2.1 with WebUSB and BOS support.



/* USART related definitions */

/**  Defines X_ON and X_OFF.
 *
@{*/
#define X_ON                17
#define X_OFF               19
/**@}*/


/**  USART buffers sizes.
 *
@{*/
#if MCU == STM32F103
#define RX_DMA_SIZE               256
#define MNTSTR_SIZE               128
#define BASE_RING_BUFFER_SIZE_POWER 10
#endif  //#if MCU == STM32F103

#if MCU == STM32F401
#define RX_DMA_SIZE               256
#define MNTSTR_SIZE               128
#define BASE_RING_BUFFER_SIZE_POWER 12
#endif  //#if MCU ==STM32F401

#define BASE_RING_BUFFER_SIZE     (1 << BASE_RING_BUFFER_SIZE_POWER)
#define CON_TX_RING_BUFFER_SIZE   (BASE_RING_BUFFER_SIZE >> 1)
#define CON_RX_RING_BUFFER_SIZE   BASE_RING_BUFFER_SIZE
#if USE_USB == true
#define UART_TX_RING_BUFFER_SIZE  (RX_DMA_SIZE << 1)
#define UART_RX_RING_BUFFER_SIZE  (RX_DMA_SIZE << 1)
#else //#if USE_USB == true
#define UART_TX_RING_BUFFER_SIZE  (BASE_RING_BUFFER_SIZE >> 1)
#define UART_RX_RING_BUFFER_SIZE  BASE_RING_BUFFER_SIZE
#endif  //#if USE_USB == true

#if USE_USB == true
#define X_OFF_TRIGGER             (3 * CON_RX_RING_BUFFER_SIZE / 4)
#define X_ON_TRIGGER              (CON_RX_RING_BUFFER_SIZE / 2)
#else //#if USE_USB == true
#define X_OFF_TRIGGER             (3 * UART_TX_RING_BUFFER_SIZE / 4)
#define X_ON_TRIGGER              (UART_TX_RING_BUFFER_SIZE / 2)
#endif  //#if USE_USB == true
/**@}*/



/**  USART - Specific USART number combined with MCU macros.
 *
@{*/
#if (MCU == STM32F103)
#if (USART_PORT == USART1)
#define GPIO_BANK_USART_TX        GPIOA
#define GPIO_PIN_USART_TX         GPIO9
#define GPIO_BANK_USART_RX        GPIOA
#define GPIO_PIN_USART_RX         GPIO10
#define RCC_USART                 RCC_USART1
#define ISR_USART                 void usart1_isr(void)
#define NVIC_USART_IRQ            NVIC_USART1_IRQ
#define RCC_DMA                   RCC_DMA1
#define USART_DMA_BUS             DMA1
#define USART_DMA_TX_CH           DMA_CHANNEL4
#define USART_DMA_TX_IRQ          NVIC_DMA1_CHANNEL4_IRQ
#define ISR_DMA_CH_USART_TX       void dma1_channel4_isr(void)
#define USART_DMA_RX_CH           DMA_CHANNEL5
#define USART_DMA_RX_IRQ          NVIC_DMA1_CHANNEL5_IRQ
#define ISR_DMA_CH_USART_RX       void dma1_channel5_isr(void)
#endif  //#if (USART_PORT == USART1)

#if(USART_PORT == USART2)
#define GPIO_BANK_USART_TX        GPIOA
#define GPIO_PIN_USART_TX         GPIO2
#define GPIO_BANK_USART_RX        GPIOA
#define GPIO_PIN_USART_RX         GPIO3
#define RCC_USART                 RCC_USART2
#define ISR_USART                 void usart2_isr(void)
#define NVIC_USART_IRQ            NVIC_USART2_IRQ
#define RCC_DMA                   RCC_DMA1
#define USART_DMA_BUS             DMA1
#define USART_DMA_TX_CH           DMA_CHANNEL7
#define USART_DMA_TX_IRQ          NVIC_DMA1_CHANNEL7_IRQ
#define ISR_DMA_CH_USART_TX       void dma1_channel7_isr(void)
#define USART_DMA_RX_CH           DMA_CHANNEL6
#define USART_DMA_RX_IRQ          NVIC_DMA1_CHANNEL6_IRQ
#define ISR_DMA_CH_USART_RX       void dma1_channel6_isr(void)
#endif  //#if (USART_PORT == USART2)
//Common for STM32F103 DMA
#define DMA_MSIZE_8BIT            DMA_CCR_MSIZE_8BIT
#define DMA_PSIZE_8BIT            DMA_CCR_PSIZE_8BIT
#define DMA_PL_HIGH               DMA_CCR_PL_HIGH
#define DMA_CR                    DMA_CCR
#define DMA_CR_EN                 DMA_CCR_EN
#define DMA_CGIF                  DMA_IFCR_CGIF1
#define dma_ch_reset              dma_channel_reset
#define dma_enable_ch             dma_enable_channel
#define dma_disable_ch            dma_disable_channel
#endif  //#if (MCU == STM32F103)

#if MCU == STM32F401
#if USART_PORT == USART2
#define GPIO_BANK_USART_TX        GPIOA
#define GPIO_PIN_USART_TX         GPIO2
#define GPIO_BANK_USART_RX        GPIOA
#define GPIO_PIN_USART_RX         GPIO3
#define RCC_USART                 RCC_USART2
#define ISR_USART                 void usart2_isr(void)
#define NVIC_USART_IRQ            NVIC_USART2_IRQ
#define RCC_DMA                   RCC_DMA1
#define USART_DMA_BUS             DMA1
#define USART_DMA_TX_CH           DMA_STREAM6
#define USART_DMA_TX_IRQ          NVIC_DMA1_STREAM6_IRQ
#define ISR_DMA_CH_USART_TX       void dma1_stream6_isr(void)
#define USART_DMA_RX_CH           DMA_STREAM5
#define USART_DMA_RX_IRQ          NVIC_DMA1_STREAM5_IRQ
#define ISR_DMA_CH_USART_RX       void dma1_stream5_isr(void)
#define USART_DMA_TRG_CHSEL       DMA_SxCR_CHSEL_4  //The same for USART 1 & 2
#endif  //#if USART_PORT == USART2

#if USART_PORT == USART1
#define GPIO_BANK_USART_TX        GPIOA
#define GPIO_PIN_USART_TX         GPIO9
#define GPIO_BANK_USART_RX        GPIOA
#define GPIO_PIN_USART_RX         GPIO10
#define RCC_USART                 RCC_USART1
#define ISR_USART                 void usart1_isr(void)
#define NVIC_USART_IRQ            NVIC_USART1_IRQ
#define RCC_DMA                   RCC_DMA2
#define USART_DMA_BUS             DMA2
#define USART_DMA_TX_CH           DMA_STREAM7
#define USART_DMA_TX_IRQ          NVIC_DMA2_STREAM7_IRQ
#define ISR_DMA_CH_USART_TX       void dma2_stream7_isr(void)
#define USART_DMA_RX_CH           DMA_STREAM2 // or DMA_STREAM5, which does NOT work!
#define USART_DMA_RX_IRQ          NVIC_DMA2_STREAM2_IRQ // or NVIC_DMA2_STREAM5_IRQ, which does NOT work!
#define ISR_DMA_CH_USART_RX       void dma2_stream2_isr(void) // or dma2_stream5_isr, which does NOT work!
#define USART_DMA_TRG_CHSEL       DMA_SxCR_CHSEL_4  //The same for USART 1 & 2
#endif  //#if USART_PORT == USART1

#if USART_PORT == USART6
#define RCC_DMA                   RCC_DMA2
#define USART_DMA_BUS             DMA2
#define USART_DMA_TX_CH           DMA_STREAM6 //or DMA_STREAM7
#define USART_DMA_TX_IRQ          NVIC_DMA2_STREAM6_IRQ //or NVIC_DMA2_STREAM7_IRQ
#define ISR_DMA_CH_USART_TX       void dma2_stream6_isr(void) //ordma2_stream7_isr
#define USART_DMA_RX_CH           DMA_STREAM1 //or DMA_STREAM2
#define USART_DMA_RX_IRQ          NVIC_DMA2_STREAM1_IRQ //NVIC_DMA2_STREAM2_IRQ
#define ISR_DMA_CH_USART_RX       void dma2_stream1_isr(void) //or dma2_stream2_isr
#define USART_DMA_TRG_CHSEL       DMA_SxCR_CHSEL_6
#endif  //#if USART_PORT == USART6
//Common for STM32F401 DMA
#define DMA_MSIZE_8BIT            DMA_SxCR_MSIZE_8BIT
#define DMA_PSIZE_8BIT            DMA_SxCR_PSIZE_8BIT
#define DMA_PL_HIGH               DMA_SxCR_PL_HIGH
#define DMA_CR                    DMA_SCR
#define DMA_CR_EN                 DMA_SxCR_EN
#define DMA_CGIF                  (DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_DMEIF | DMA_FEIF)
#define dma_ch_reset              dma_stream_reset
#define dma_enable_ch             dma_enable_stream
#define dma_disable_ch            dma_disable_stream
#endif  //#if MCU == STM32F401
/**@}*/


//It's a repetitive piece of code that can not be a function.
//It opens a bracket {.
#define CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET \
  if(xonoff_sendnow)\
  {\
    uint8_t data;\
    xonoff_sendnow = false;\
    if(xoff_condition) data = X_OFF;\
    if(xon_condition) data = X_ON;

//Close } bracket, to mitigate the risk of programming errors due to unpairing "{}"
#define CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET \
  }


/**  Compute ring available characters.
 *
 * It is faster to compute an AND than if used IF.
@{*/
#define QTTY_CHAR_IN(RING)        ((RING.bufSzMask + 1 - RING.get_ptr + RING.put_ptr) & RING.bufSzMask)
/**@}*/


/**  Defines a pascal type string to be used in auxiliary routines.
 *
 */
struct s_pascal_string
{
/**  Defines the str_len to control number of elements inside the string.
 *
 */
  uint8_t str_len;
/**  Defines the bufSzMask to mask the relevant bits when computing position and number of available characters.
 *
 */
  uint8_t bufSzMask;
/**  Defines the data to manage a pascal type string to be used in auxiliary routines.
 *
 */
  uint8_t *data;
};


/**  Defines the structure of the main buffers.
 *
 */
struct sring
{
/**  Defines the data buffer.
 *
 */
  uint8_t *data;
/**  Defines the data buffer size mask.
 *
 */
  uint16_t bufSzMask;
/**  Defines the data buffer put pointer.
 *
 * Makes it possible to adjust its size (delay) dynamicly according to speed.
 */
  uint16_t put_ptr;
/**  Defines the data buffer get pointer.
 *
 */
  uint16_t get_ptr;
};


#endif  //#ifndef T_SYSTEM_H

#ifdef __cplusplus
}
#endif  //#ifdef __cplusplus
