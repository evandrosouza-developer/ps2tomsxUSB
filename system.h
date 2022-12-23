/** @defgroup 01 Manager_group Main
 *
 * @ingroup infrastructure_apis
 *
 * @file system.h System main definitions of the project.
 *
 * @brief <b>System main definitions of the project. Header file of all other headers.</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 25 September 2022
 *
 * This file has the main definitions about the design.
 * This original SW is compiled to a Sharp/Epcom MSX HB-8000 and a brazilian ABNT2 PS/2 keyboard (ID=275)
 * But it is possible to update the table sending a Intel Hex File through serial or USB
 *
 * It supports both the STM32F4 and STM32F1 series of ARM Cortex Microcontrollers
 * by ST Microelectronics.
 *
 * LGPL License Terms ref lgpl_license
 */

/*
 * This file is part of the PS/2 to MSX Keyboard converter enviroment:
 * PS/2 to MSX keyboard Converter and MSX Keyboard Subsystem Emulator
 * designs, based on libopencm3 project.
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



#ifdef __cplusplus
extern "C" {
#endif

#ifndef T_SYSTEM_H
#define T_SYSTEM_H


#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_fs.h>

/* Microcontroller STM32F103 or STM32F401 */
#define STM32F103                 0x410     //Blue Pill
#define STM32F401                 0x423     //WeAct Studio MiniF4 Black Pill

#define MCU                       STM32F401


// Place to get the microcontroller unique id to compute serial number
#ifndef DESIG_UNIQ_ID_BASE
#if MCU == STM32F103
#define DESIG_UNIQ_ID_BASE        0x1FFFF7E8;
#define LEN_SERIAL_No             8
#endif  //#if MCU == STM32F103
#if MCU == STM32F401
#define DESIG_UNIQ_ID_BASE        0x1FFF7A10;
#define LEN_SERIAL_No             12
#endif  //#if MCU == STM32F401
#endif  //#ifndef DESIG_UNIQ_ID_BASE


/* Database allocation in flash */
#define DB_NUM_COLS               8
#define N_DATABASE_REGISTERS      320
#define DATABASE_SIZE             N_DATABASE_REGISTERS * DB_NUM_COLS

#if MCU == STM32F103
#define NUM_DATABASE_IMG          2
//Address of Base of flash page, used to put various Databases without need of erase each time
//update process is done.
#define FLASH_BASE_ADD            0x08000000
#define INITIAL_DATABASE          0x08007600  //Place to put the initial (compilation time) database
#define DATABASE_BASE_ADD         0x08006C00  //This address marks the beggining of page 27
#define DATABASE_BASE_PAGE        27
#define DATABASE_TOP_ADDR         0x08007FFF  //STM32F103C6T6 (32K Flash 10K RAM)
#define DATABASE_TOP_PAGE         31
#define FLASH_PAGE_SIZE           0x400       //1K in STM32F103
#endif  //#if MCU == STM32F103

#if MCU == STM32F401
#define NUM_DATABASE_IMG          6           //to fit in a 16k window
//Address of Base of flash page, used to put various Databases without need of erase each time
//update process is done. In STM32F4CCU6, the Database remains on the following flash address,
//with an amount of 16K (Flash Sector 3: 0x800C000 to 0x800FFFF)
#define INITIAL_DATABASE          0x0800F600  //SECTOR 3 - Place to put the initial (compilation time) database
#define FLASH_SECTOR3_BASE        0x0800C000
#define FLASH_SECTOR3_TOP         0x0800FFFF
#define FLASH_SECTOR3_NUMBER      3
#endif  //#if MCU == STM32F401


/* High Resolution Timer definitions */
#define TIM_HR                    TIM2        //Here we define which timer we will use for hrtimer
#if TIM_HR == TIM2
#define RCC_TIM_HR                RCC_TIM2
#define RST_TIM_HR                RST_TIM2
#define NVIC_TIM_HR_IRQ           NVIC_TIM2_IRQ
#define ISR_TIM_HR                void tim2_isr(void)
#endif  //#if TIM_HR_TIMER == TIM2


/* Interrupt priority definitions. From 0 to 15. Low numbers are high priority. */
/** Interrupt priority definitions.
 *
 * From 0 to 15. Low numbers are high priority.
@{*/
#define IRQ_PRI_Y_SCAN            (1 << 4)    //Colunm rows from 8255 (MSX)
#define IRQ_PRI_TIM_HR            (2 << 4)
#define IRQ_PRI_EXT15             (2 << 4)    //Keyboard, while int above is not working
#define IRQ_PRI_SYSTICK           (3 << 4)
#define IRQ_PRI_USB               (4 << 4)
#define IRQ_PRI_USART             (5 << 4)
#define IRQ_PRI_USART_DMA         (5 << 4)
/**@}*/


/* General definitions about timings */
/** General definitions about timings
 *
 * Timings used by systick and timeout of user input in user entry of Database update
@{*/
#define FREQ_INT_SYSTICK          30
#define MAX_TIMEOUT2RX_INTEL_HEX  11 * FREQ_INT_SYSTICK
#define MAX_TIMEOUT2AMPERSAND     6  * FREQ_INT_SYSTICK
/**@}*/


/* USB related definitions */

/* Define the usage of USB */
/** Define the usage of USB 
 * 
 * 
 */
#define USE_USB                   true      //Here you can change the USB support in M4 family.

#define CDC_ONLY_ON_USB           false     // If we are providing serial interfaces only => True

#if (MCU == STM32F103) && (USE_USB == true) //***** DO NOT CHANGE IT! In PS/2 keyboard Interface for MSX design blue pill DO NOT have sufficient hardware resouces (5V tolerant pins and Flash room) to support USB and basic target functionality.
#undef  USE_USB                             //***** DO NOT CHANGE IT! In PS/2 keyboard Interface for MSX design blue pill DO NOT have sufficient hardware resouces (5V tolerant pins and Flash room) to support USB and basic target functionality.
#define USE_USB                   false     //***** DO NOT CHANGE IT! In PS/2 keyboard Interface for MSX design blue pill DO NOT have sufficient hardware resouces (5V tolerant pins and Flash room) to support USB and basic target functionality.
#endif  //##if (MCU == STM32F103) && (USE_USB == true)
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

#undef  USB_REQ_TYPE_IN
#define USB_REQ_TYPE_IN           0x80
#define OTG_DCTL                  0x804
#define OTG_FS_DCTL               MMIO32(USB_OTG_FS_BASE + OTG_DCTL)
#define OTG_FS_DCTL_SDIS          1<<1        //0: Normal operation. 2: The core generates a device disconnect event to the USB host.
#define OTG_FS_DSTS               MMIO32(USB_OTG_FS_BASE + OTG_DSTS)  
#define OTG_FS_DSTS_SUSPSTS       1<<0        //0: Suspend condition is detected on the USB. 1: Normal operation.
/* USB Control register */
#if MCU == STM32F103
#define USB_CNTR_REG              MMIO32(USB_DEV_FS_BASE +                0x40)
#define USB_CNTR_REG_PDWN         1<<1        //0: Exit Power Down. 1: Enter Power down mode.
#endif  //#if MCU == STM32F103

#define USB_CLASS_MISCELLANEOUS   0xEF        //  Idea taked from Blue Pill Bootloader
#define USB_CDC_REQ_GET_LINE_CODING 0x21      // Not defined in libopencm3
#define SEND_ENCAPSULATED_COMMAND_bmRequestType 0x21//Get in https://docs.microsoft.com/en-us/windows-hardware/drivers/network/control-channel-characteristics
#define GET_ENCAPSULATED_RESPONSE_bmRequestType 0xA1//Get in https://docs.microsoft.com/en-us/windows-hardware/drivers/network/control-channel-characteristics
#define SEND_ENCAPSULATED_COMMAND_bRequest 0  //Get in https://docs.microsoft.com/en-us/windows-hardware/drivers/network/control-channel-characteristics
#define GET_ENCAPSULATED_RESPONSE_bRequest 1  //Get in https://docs.microsoft.com/en-us/windows-hardware/drivers/network/control-channel-characteristics

#define USB_PID                   0x5740      //ST CDC from various forums
#define USB_VID                   0x1d50      //OpenMoko

#endif  //#if USE_USB == true

/*  Index of each USB interface. Must be consecutive and must sync with interfaces[].*/
/*  Index of each USB interface. Must be consecutive and must sync with interfaces[].*/
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

//  USB Endpoints addresses. Starts with 1, as endpoint 0 is the default.
/**  USB Endpoints addresses.
 *
 * Starts with 1, as endpoint 0 is the default.
 @enum ENDPOINT USB Endpoints addresses. Starts with 1, as endpoint 0 is the default.*/
enum ENDPOINT{
  EP_CON_DATA_OUT =               1,          //CDC Data OUT of FIRST endpoint
  EP_CON_COMM_OUT,                            //CDC Command of FIRST endpoint: uses this as +0x80
  EP_UART_DATA_OUT,                           //CDC Data OUT of SECOND endpoint
  EP_UART_COMM_OUT,                           //CDC Command of SECOND endpoint: uses this as +0x80
  //CDC Data IN of First endpoint. (0x80=USB_REQ_TYPE_IN)
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



/* Hardware port definitions */
#define TIMxCC1_INT               1
#define GPIO_INT                  2

#if MCU == STM32F103
/**
 * STM32F103 Hardware port definitions
 *
 @{*/ 
#define HARDWARE_BASE             "Blue Pill (STM32F103C6 C8T6 and up)"
#define PS2_CLK_INTERRUPT        TIMxCC1_INT
#define USART_PORT                USART2
#define EMBEDDED_LED_PORT         GPIOC
#define EMBEDDED_LED_PIN          GPIO13
#define X_PORT                    GPIOB
#define X7_PORT                   GPIOB
#define X7_PIN                    GPIO9
#define X6_PORT                   GPIOB
#define X6_PIN                    GPIO8
#define X5_PORT                   GPIOB
#define X5_PIN                    GPIO10
#define X4_PORT                   GPIOB
#define X4_PIN                    GPIO15
#define X3_PORT                   GPIOB
#define X3_PIN                    GPIO11
#define X2_PORT                   GPIOB
#define X2_PIN                    GPIO14
#define X1_PORT                   GPIOB
#define X1_PIN                    GPIO13
#define X0_PORT                   GPIOB
#define X0_PIN                    GPIO12
#define Y3_PORT                   GPIOA
#define Y3_PIN                    GPIO12      //port A10 (Y2) broken mirrored in Kicad design
#define Y3_exti                   EXTI12
#define Y2_PORT                   GPIOA       //port A10 (Y2) broken mirrored in Kicad design
#define Y2_PIN                    GPIO11      //port A10 (Y2) broken mirrored in Kicad design
#define Y2_exti                   EXTI11
#define Y1_PORT                   GPIOA
#define Y1_PIN                    GPIO9
#define Y1_exti                   EXTI9
#define Y0_PORT                   GPIOA
#define Y0_PIN                    GPIO8
#define Y0_exti                   EXTI8
#define CAPSLOCK_PORT             GPIOB
#define CAPSLOCK_PIN              GPIO4
#define CAPSLOCK_exti             EXTI4
#define KANA_PORT                 GPIOB
#define KANA_PIN                  GPIO6
#define KANA_exti                 EXTI6
//SERIAL2_PORT                    GPIOA (Pre-defined)
//SERIAL2_TX_PIN                  GPIO2 (Pre-defined)
//SERIAL2_RX_PIN                  GPIO3 (Pre-defined)

#define Y_MASK                    0x1B00      //Valid bits: A12 as Y3, A11 as Y2, A9 is Y1 and A8 is Y0.

//Force Y_PORT_id pin (Sync pin) to high, so the first time slot is a low => Falling transition on the start of frame
//Force Xint_PIN to low. portXread in msxmap.cpp will put this in high at each port update: to be possible mesaure real time of reading.
#define X7_SET_AND                0xFDFFFFFF  // 4261412863
#define X6_SET_AND                0xFEFFFFFF  // 4278190079
#define X5_SET_AND                0xFBFFFFFF  // 4227858431
#define X4_SET_AND                0xF7FFFFFF  // 4160749567
#define X3_SET_AND                0x7FFFFFFF  // 2147483647
#define X2_SET_AND                0xBFFFFFFF  // 3221225471
#define X1_SET_AND                0xDFFFFFFF  // 3758096383
#define X0_SET_AND                0xEFFFFFFF  // 4026531839
#define X7_CLEAR_OR               0x02000000  // 33554432
#define X6_CLEAR_OR               0x01000000  // 16777216
#define X5_CLEAR_OR               0x04000000  // 67108864
#define X4_CLEAR_OR               0x08000000  // 134217728
#define X3_CLEAR_OR               0x80000000  // 2147483648
#define X2_CLEAR_OR               0x40000000  // 1073741824
#define X1_CLEAR_OR               0x20000000  // 536870912
#define X0_CLEAR_OR               0x10000000  // 268435456
#define X7_CLEAR_AND              0xFFFFFDFF  // 4294966783
#define X6_CLEAR_AND              0xFFFFFEFF  // 4294967039
#define X5_CLEAR_AND              0xFFFFFBFF  // 4294966271
#define X4_CLEAR_AND              0xFFFFF7FF  // 4294965247
#define X3_CLEAR_AND              0xFFFF7FFF  // 4294934527
#define X2_CLEAR_AND              0xFFFFBFFF  // 4294950911
#define X1_CLEAR_AND              0xFFFFDFFF  // 4294959103
#define X0_CLEAR_AND              0xFFFFEFFF  // 4294963199
#define X7_SET_OR                 0x00000200  // 512
#define X6_SET_OR                 0x00000100  // 256
#define X5_SET_OR                 0x00000400  // 1024
#define X4_SET_OR                 0x00000800  // 2048
#define X3_SET_OR                 0x00008000  // 32768
#define X2_SET_OR                 0x00004000  // 16384
#define X1_SET_OR                 0x00002000  // 8192
#define X0_SET_OR                 0x00001000  // 4096

#define PS2_DATA_PORT             GPIOB
#define PS2_DATA_PIN              GPIO7
#define PS2_CLK_I_PORT            GPIOA
#define PS2_CLK_I_PIN             GPIO15
#define PS2_CLK_I_EXTI            EXTI15
#define PS2_CLK_O_PORT            GPIOA       //Same pin of PS2_CLK_I_PIN
#define PS2_CLK_O_PIN             GPIO15      //Same pin of PS2_CLK_I_PIN
#define PS2_POWER_CTR_PORT        GPIOB 
#define PS2_POWER_CTR_PIN         GPIO1

//To debug
#define SYSTICK_PORT              GPIOA       //A0
#define SYSTICK_PIN               GPIO0
#define TIM2CC1_PIN               GPIO1       //A1
#define BIT0_PORT                 GPIOA
#define BIT0_PIN                  GPIO4       //A4
#define INT_TIM2_PORT             GPIOA
#define TIM2UIF_PIN               GPIO5       //A5
#define Dbg_Yint_PORT             GPIOA
#define Dbg_Yint0and1_PIN         GPIO6       //A6
#define Dbg_Yint2and3_PIN         GPIO7       //A7

//Available resources
#define AVAILABLE_B0_PORT         GPIOB
#define AVAILABLE_B0_PIN          GPIO0
#define AVAILABLE_B3_PORT         GPIOB
#define AVAILABLE_B3_PIN          GPIO3
#define AVAILABLE_A11_PORT        GPIOA
#define AVAILABLE_A11_PIN         GPIO11
#define AVAILABLE_A12_PORT        GPIOA
#define AVAILABLE_A12_PIN         GPIO12

/**@}*/
#endif  //#if MCU == STM32F103


#if MCU == STM32F401
/**
 * STM32F401 Hardware port definitions
 * 
 @{*/ 
#define USART_PORT                USART1
#define PS2_CLK_INTERRUPT         GPIO_INT
#define HARDWARE_BASE             "WeAct MiniF4 - Black Pill v2.0+ (STM32F401CxU6)"
#define EMBEDDED_LED_PORT         GPIOC
#define EMBEDDED_LED_PIN          GPIO13
#define X_PORT                    GPIOB
#define X7_PORT                   GPIOB
#define X7_PIN                    GPIO0
#define MSX_X_BIT7                0
#define X6_PORT                   GPIOB
#define X6_PIN                    GPIO15
#define MSX_X_BIT6                15
#define X5_PORT                   GPIOB
#define X5_PIN                    GPIO1
#define MSX_X_BIT5                1
#define X4_PORT                   GPIOB
#define X4_PIN                    GPIO14
#define MSX_X_BIT4                14
#define X3_PORT                   GPIOB
#define X3_PIN                    GPIO3
#define MSX_X_BIT3                3
#define X2_PORT                   GPIOB
#define X2_PIN                    GPIO13
#define MSX_X_BIT2                13
#define X1_PORT                   GPIOB
#define X1_PIN                    GPIO10
#define MSX_X_BIT1                10
#define X0_PORT                   GPIOB
#define X0_PIN                    GPIO12
#define MSX_X_BIT0                12
#define Y_PORT                    GPIOA
#define Y_PIN                     GPIO4
#define Y3_PORT                   GPIOA
#define Y3_PIN                    GPIO5
#define Y3_exti                   EXTI5
#define Y2_PORT                   GPIOA
#define Y2_PIN                    GPIO6
#define Y2_exti                   EXTI6
#define Y1_PORT                   GPIOA
#define Y1_PIN                    GPIO7
#define Y1_exti                   EXTI7
#define Y0_PORT                   GPIOA
#define Y0_PIN                    GPIO8
#define Y0_exti                   EXTI8
#define CAPSLOCK_PORT             GPIOB
#define CAPSLOCK_PIN              GPIO4
#define CAPSLOCK_exti             EXTI4
#define KANA_PORT                 GPIOB
#define KANA_PIN                  GPIO6
#define KANA_exti                 EXTI6
#if (USE_USB == true)
#define OTG_FS_DM_PORT            GPIOA
#define OTG_FS_DM_PIN             GPIO11
#define OTG_FS_DP_PORT            GPIOA
#define OTG_FS_DP_PIN             GPIO12
#endif  //#if (USE_USB == true)
//SERIAL2_PORT                    GPIOA (Pre-defined)
//SERIAL2_TX_PIN                  GPIO2 (Pre-defined)
//SERIAL2_RX_PIN                  GPIO3 (Pre-defined)
//SERIAL3_PORT                    GPIOB (Pre-defined)
//SERIAL3_TX_PIN                  GPIO10(Pre-defined)
//SERIAL3_RX_PIN                  GPIO11(Pre-defined)

#define Y_MASK                    0x1E0 //Valid bits: A5 as Y3, A6 as Y2, A7 is Y1 and A8 is Y0.

#define PS2_DATA_PORT             GPIOB
#define PS2_DATA_PIN              GPIO5
#define PS2_CLK_I_PORT            GPIOA
#define PS2_CLK_I_PIN             GPIO15
#define PS2_CLK_I_EXTI            EXTI15
#define PS2_CLK_O_PORT            GPIOB
#define PS2_CLK_O_PIN             GPIO7
#define PS2_POWER_CTR_PORT        GPIOA
#define PS2_POWER_CTR_PIN         GPIO4 

//Debug facilities
#define USER_KEY_PORT             GPIOA
#define USER_KEY_PIN              GPIO0
#define INT_PS2_PORT              GPIOA
#define INT_PS2_PIN               GPIO1
#define PS2_START_SEND_PORT       GPIOA
#define PS2_START_SEND_PIN        GPIO2
#define Dbg_Yint_PORT             GPIOB
#define Dbg_Yint_PIN              GPIO7
#define TIM2UIF_PORT              GPIOB
#define INT_TIM2_PORT             GPIOB
#define TIM2UIF_PIN               GPIO8
#define TIM2CC1_PORT              GPIOB
#define TIM2CC1_PIN               GPIO9

//Available resources
#define AVAILABLE_A3_PORT         GPIOA
#define AVAILABLE_A3_PIN          GPIO3
#define AVAILABLE_B2_PORT         GPIOB
#define AVAILABLE_B2_PIN          GPIO2     //Boot1 pin (There is a R=10K to GND)
#if !(USE_USB == true)
#define AVAILABLE_A11_PORT        GPIOA
#define AVAILABLE_A11_PIN         GPIO11
#define AVAILABLE_A12_PORT        GPIOA
#define AVAILABLE_A12_PIN         GPIO12
#endif  //#if !(USE_USB == true)


#define X7_SET_AND                0xFFFEFFFF  // 4294901759
#define X6_SET_AND                0x7FFFFFFF  // 2147483647
#define X5_SET_AND                0xFFFDFFFF  // 4294836223
#define X4_SET_AND                0xBFFFFFFF  // 3221225471
#define X3_SET_AND                0xFFF7FFFF  // 4294443007
#define X2_SET_AND                0xDFFFFFFF  // 3758096383
#define X1_SET_AND                0xFBFFFFFF  // 4227858431
#define X0_SET_AND                0xEFFFFFFF  // 4026531839
#define X7_CLEAR_OR               0x00010000  // 65536
#define X6_CLEAR_OR               0x80000000  // 2147483648
#define X5_CLEAR_OR               0x00020000  // 131072
#define X4_CLEAR_OR               0x40000000  // 1073741824
#define X3_CLEAR_OR               0x00080000  // 524288
#define X2_CLEAR_OR               0x20000000  // 536870912
#define X1_CLEAR_OR               0x04000000  // 67108864
#define X0_CLEAR_OR               0x10000000  // 268435456
#define X7_CLEAR_AND              0xFFFFFFFE  // 4294967294
#define X6_CLEAR_AND              0xFFFF7FFF  // 4294934527
#define X5_CLEAR_AND              0xFFFFFFFD  // 4294967293
#define X4_CLEAR_AND              0xFFFFBFFF  // 4294950911
#define X3_CLEAR_AND              0xFFFFFFF7  // 4294967287
#define X2_CLEAR_AND              0xFFFFDFFF  // 4294959103
#define X1_CLEAR_AND              0xFFFFFBFF  // 4294966271
#define X0_CLEAR_AND              0xFFFFEFFF  // 4294963199
#define X7_SET_OR                 0x00000001  // 1
#define X6_SET_OR                 0x00008000  // 32768
#define X5_SET_OR                 0x00000002  // 2
#define X4_SET_OR                 0x00004000  // 16384
#define X3_SET_OR                 0x00000008  // 8
#define X2_SET_OR                 0x00002000  // 8192
#define X1_SET_OR                 0x00000400  // 1024
#define X0_SET_OR                 0x00001000  // 4096

//#define MMIO32(addr)  (*(volatile uint32_t *)(addr))
#define DBGMCU_APB1_FZ            MMIO32(DBGMCU_BASE+8)
#define DBG_I2C3_SMBUS_TIMEOUT    1<<23
#define DBG_I2C2_SMBUS_TIMEOUT    1<<22
#define DBG_I2C1_SMBUS_TIMEOUT    1<<21
#define DBG_IWDG_STOP             1<<12
#define DBG_WWDG_STOP             1<<11
#define DBG_RTC_STOP              1<<10
#define DBG_TIM5_STOP             1<<11
#define DBG_TIM4_STOP             1<<11
#define DBG_TIM3_STOP             1<<11
#define DBG_TIM2_STOP             1<<0

/**@}*/
#endif  //#if MCU == STM32F401




/* USART related definitions */

#if MCU == STM32F103
#define RX_DMA_SIZE               256
#define MNTSTR_SIZE               128
#define BASE_RING_BUFFER_SZ_POWER 10
#endif  //#if MCU == STM32F103

#if MCU == STM32F401
#define RX_DMA_SIZE               256
#define MNTSTR_SIZE               128
#define BASE_RING_BUFFER_SZ_POWER 12
#endif  //#if MCU == STM32F401

#define BASE_RING_BUFFER_SIZE     (1 << BASE_RING_BUFFER_SZ_POWER)
#define CON_TX_RING_BUFFER_SIZE   (BASE_RING_BUFFER_SIZE >> 1)
#define CON_RX_RING_BUFFER_SIZE   BASE_RING_BUFFER_SIZE
#if USE_USB == true
#define UART_TX_RING_BUFFER_SIZE  (RX_DMA_SIZE)
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


/**  Defines X_ON and X_OFF.
 *
@{*/
#define X_ON                      17
#define X_OFF                     19
/**@}*/


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


#if MCU == STM32F401
/**  Defines "passwords" to get core reseted
 *
 @{*/
#define BOOTMAGIC0 0xb007da7a
#define BOOTMAGIC1 0xbaadfeed
/**@}*/
#endif  //#if MCU == STM32F401


/**  Compute ring available characters.
 *
 * It is faster to compute an AND than if used IF.
@{*/
#define QTTY_CHAR_IN(RING)        ((RING.bufSzMask + 1 - RING.get_ptr + RING.put_ptr) & RING.bufSzMask)
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
