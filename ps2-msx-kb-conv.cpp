/** @addtogroup 01 Manager_group Main
 *
 * @ingroup Main_design_definitions
 *
 * @file ps2-msx-kb-conv.cpp Main code. Created as Bare Metal (no OS needed).
 *
 * @brief <b>Main code. Created as Bare Metal (no OS needed).</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 25 September 2022
 *
 * This is the main file of PS/2 to MSX Keyboard Converter with supports
 * the STM32F4 and STM32F1 series of ARM Cortex Microcontrollers by ST Microelectronics.
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

//Use Tab width=2

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/usart.h>

#include "system.h"
#include "sys_timer.h"
#include "serial.h"
#include "hr_timer.h"
#include "ps2handl.h"
#include "msxmap.h"
#include "SpecialFaultHandlers.h"
#include "serial_no.h"
#include "dbasemgt.h"
#if USE_USB == true
#include "cdcacm.h"
#endif  //#if USE_USB == true
#include "version.h"

//#define DO_PRAGMA(x) _Pragma (#x)
//#define TODO(x) DO_PRAGMA(message (#x))

#define  DELAY_JHONSON  6

//Variáveis globais
extern uint32_t systicks;                         //Declared on sys_timer.cpp
extern bool     mount_scancode_OK;                //Declared on ps2handl.c
extern bool     ps2_keyb_detected;                //Declared on ps2handl.c
extern bool     ps2numlockstate;                  //Declared on ps2handl.c
extern bool     command_running, ps2int_RX_bit_idx;//Declared on ps2handl.c
extern bool     caps_state, kana_state;           //Declared on ps2handl.c
extern bool     caps_former, kana_former;         //Declared on ps2handl.c
extern bool     update_ps2_leds;                  //Declared on msxmap.cpp
extern bool     compatible_database;              //Declared on msxmap.cpp
extern uint8_t  scancode[4];                      //Declared on msxmap.cpp
extern uint32_t formerscancode;                   //Declared on msxmap.cpp
extern bool     do_next_keep_alive;               //Declared on msxmap.cpp
extern uint8_t  serial_no[LEN_SERIAL_No + 1];     //Declared on serial_no.c
#if USE_USB == true
extern int      usb_configured;                   //Declared on cdcacm.c
int             usb_configured_prev;
#endif  //#if USE_USB === true


//Prototype area
void end_of_code(uint32_t*);

int main(void)
{
  uint32_t reset_org;
  end_of_code(&reset_org);

#if MCU == STM32F103
  //Blue Pill
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

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
#endif  //#if MCU == STM32F103

#if MCU == STM32F401
  //WeAct miniF4 STM32F401CCU6 and up, V2.0 and up (Black Pill)
  rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
#endif  //#if MCU == STM32F401

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

  power_on_ps2_keyboard();

  //Debug & performance measurement
  general_debug_setup();

  //Minimize interferencies from unused pins left open
  put_pullups_on_non_used_pins();

  //Get Serial number based on unique microcontroller factory masked number
  serialno_read(serial_no);

  serial_setup();

#if USE_USB == true
  cdcacm_init();
  //Give some time to USB be enumerated and recognized by the OS and terminal app
  for (uint32_t i = 0; i < 0x4000000; i++) __asm__("nop");
#endif  //#if USE_USB == true

  con_send_string((uint8_t*)"\r\n\n\r\nPS/2 to MSX Keyboard Converter " FIRMWARE_VERSION);
  con_send_string((uint8_t*)"\r\nBased on " HARDWARE_BASE "\r\nSerial number ");
  con_send_string((uint8_t*)serial_no);
  con_send_string((uint8_t*)"\r\nFirmware built on ");
  con_send_string((uint8_t*)__DATE__);
  con_send_string((uint8_t*)" ");
  con_send_string((uint8_t*)__TIME__);
  con_send_string((uint8_t*)"\r\n\nThis boot was requested from ");


  uint8_t mnt_str[MNTSTR_SIZE];
  s_pascal_string pascal_string;
  // Compute, prepare and print user message about last reset.
  pascal_string_init(&pascal_string, mnt_str, MNTSTR_SIZE);

  if(reset_org & RCC_CSR_PINRSTF)
    string_append((uint8_t*)"NRST_pin", &pascal_string);
  if(reset_org & RCC_CSR_PORRSTF){
    if(pascal_string.str_len == 8)
      string_append((uint8_t*)" and PowerOn", &pascal_string);
    else
      string_append((uint8_t*)"Software", &pascal_string);}
  if(reset_org & RCC_CSR_SFTRSTF){
    if(pascal_string.str_len >= 8)
      string_append((uint8_t*)" and Software", &pascal_string);
    else
      string_append((uint8_t*)"Software", &pascal_string);}
  if(reset_org & RCC_CSR_IWDGRSTF){
    if(pascal_string.str_len >= 8)
      string_append((uint8_t*)" and IWDG", &pascal_string);
    else
      string_append((uint8_t*)"IWDG", &pascal_string);}
  if(reset_org & RCC_CSR_WWDGRSTF){
    if(pascal_string.str_len >= 8)
      string_append((uint8_t*)" and WWDG", &pascal_string);
    else
      string_append((uint8_t*)"WWDG", &pascal_string);}
  if(reset_org & RCC_CSR_LPWRRSTF){
    if(pascal_string.str_len >= 8)
      string_append((uint8_t*)" and Low-power", &pascal_string);
    else
      string_append((uint8_t*)"Low-power", &pascal_string);}
  con_send_string(pascal_string.data);
  //After send pascal_string.data to console, clear it.
  pascal_string.str_len = 0;
  pascal_string.data[0] = 0;

  con_send_string((uint8_t*)". Booting...");

  con_send_string((uint8_t*)"\r\n\r\nConfiguring:\r\n");

#if USE_USB == true
  if(usb_configured)
    con_send_string((uint8_t*)". USB has been enumerated => Console and UART are over USB;\r\n");
  else
    con_send_string((uint8_t*)". USB host not found => Using Console over UART;\r\n. USB not disabled\r\n");
#else //#if USE_USB == true
  con_send_string((uint8_t*)". Non USB version => Console is over UART.\r\n");
#endif  //#if USE_USB == true

#if USE_USB == true
  usb_configured_prev = usb_configured;
#endif  //#if USE_USB == true

  con_send_string((uint8_t*)". PS/2 Port powered up.\r\n");
  con_send_string((uint8_t*)". ARM System Timer;\r\n");

  systick_setup();
  
  con_send_string((uint8_t*)". Independent Watch Dog Timer;\r\n");

  // Turn on the Independent WatchDog Timer
  iwdg_set_period_ms(100);  // 3 x sys_timer
  iwdg_start();

  con_send_string((uint8_t*)". High resolution Timer;\r\n");

  // Now configure High Resolution Timer for PS/2 Clock interrupts (via CC) and micro second Delay
  tim_hr_setup(TIM_HR);

  con_send_string((uint8_t*)". PS/2 Port: Waiting up to 2.5s (75 ticks) with powered on keyboard\r\n");
  con_send_string((uint8_t*)"  to proceed BAT: |.....|\r  to proceed BAT: |");

  ps2_keyb_detect();

  con_send_string((uint8_t*)". Database with know-how to manage and interface PS/2 Keyboard to MSX:\r\n");
  //Check the Database version, get y_dummy, ps2numlockstate and enable_xon_xoff
  database_setup();

  con_send_string((uint8_t*)". 5V compatible pin ports and interrupts to interface to MSX.\r\n");

  msxmap object;
  object.msx_interface_setup();

  if (!compatible_database || !ps2_keyb_detected)
  {
    //Here it will allow update the Database Conversion, as the PS/2 keyboard was not detected
    //or Database is incompatible, so at this point, this module is not working as target planned.

    //No PS/2 keyboard, so power off the port.
    power_off_ps2_keyboard();

    //Init procedure to fillin MSXTABLE by receiving an INTEL HEX through USART1
    //Disables all interrupts but systicks and USART
    exti_disable_request(Y3_exti | Y2_exti | Y1_exti | Y0_exti);
    timer_disable_irq(TIM_HR, TIM_DIER_CC1IE | TIM_DIER_UIE);
    exti_disable_request(PS2_CLK_I_EXTI);

    //Read the MSX Database Table Intel Hex by serial (or USB when available) and flashes 2560 bytes, from 0x08007600 to 0x08007FFF
    //Up to 3 databases can be put in flash without need to erase. It is automatically managed.
    flash_rw();

    //Halt here.
    for(;;);
  }

  con_send_string((uint8_t*)"\r\nBoot complete. Be welcome!\r\n");

  //Test keyboard leds for humans, using Jhonson Counter mode.
  uint32_t systicks_base = systicks;
  ps2_update_leds(false, false, false);
  while (systicks < (systicks_base + 1 * DELAY_JHONSON)) __asm("nop");

  ps2_update_leds(true, false, false);
  while (systicks < (systicks_base + 2 * DELAY_JHONSON)) __asm("nop");

  ps2_update_leds(true, true, false);
  while (systicks < (systicks_base + 3 * DELAY_JHONSON)) __asm("nop");

  ps2_update_leds(true, true, true);
  while (systicks < (systicks_base + 4 * DELAY_JHONSON)) __asm("nop");

  ps2_update_leds(false, true, true);
  while (systicks < (systicks_base + 5 * DELAY_JHONSON)) __asm("nop");

  ps2_update_leds(false, false, true);
  while (systicks < (systicks_base + 6 * DELAY_JHONSON)) __asm("nop");

  ps2_update_leds(false, false, false);
  while (systicks < (systicks_base + 7 * DELAY_JHONSON)) __asm("nop");


  /*********************************************************************************************/
  /************************************** Main Loop ********************************************/
  /*********************************************************************************************/
  uint32_t* ptr_scancode = (uint32_t*)scancode;
  for(;;)
  {
    //The first functionality running in the main loop
    //New key processing:
    if (mount_scancode())
    {
      //At this point, we already have the assembled compound keys code, so
      //to avoid unnecessary processing at convert2msx, check if this last scancode
      //is different from the former one.
      if (*ptr_scancode != formerscancode)
      {
        //Serial message the keyboard change
        /*uint8_t mountstring[3];
        con_send_string((uint8_t*)"Bytes qty=");
        conv_uint8_to_2a_hex(scancode[0], mountstring);
        con_send_string(mountstring);
        con_send_string((uint8_t*)"; Scan code=");
        conv_uint8_to_2a_hex(scancode[1], mountstring);
        con_send_string(mountstring);
        con_send_string((uint8_t*)"; ");
        conv_uint8_to_2a_hex(scancode[2], mountstring);
        con_send_string(mountstring);
        con_send_string((uint8_t*)"; ");
        conv_uint8_to_2a_hex(scancode[3], mountstring);
        con_send_string(mountstring);
        con_send_string((uint8_t*)"\r\n"); */
        //Toggle led each PS/2 keyboard change (both new presses and releases).
        gpio_toggle(EMBEDDED_LED_PORT, EMBEDDED_LED_PIN); //Toggle led to sinalize a scan code is beeing send to convert2msx
        // Do the MSX search and conversion
        msxmap objeto;
        objeto.convert2msx();
      } //if (scancode != formerscancode)
      //update former keystroke
      formerscancode = *ptr_scancode;
      //Now we can reset to prepair to do a new mount_scancode (Clear to read a new key press or release)
      *ptr_scancode = 0;
      mount_scancode_OK = false;
    } //if (mount_scancode())

    //If keyboard is not responding to commands, reinit it through reset
    if(!(systicks & (uint32_t)0x7F))
    { //each ~4s (128 * 1/30)
      if(do_next_keep_alive)  //when time comes up, do it just once
      {
        do_next_keep_alive = false;
        if(!keyboard_check_alive())
        {
          con_send_string((uint8_t*)"Keyboard is non responsive (KeepAlive): Reset requested by the system.\r\n");
          reset_requested();
        }
      }
    }
    else
      do_next_keep_alive = true;

    //The second functionality running in main loop: Update the keyboard leds
    if( !command_running  &&    //Only does led update when the previous one is concluded
        update_ps2_leds )
    {
      update_ps2_leds = false;
      caps_former = caps_state;
      kana_former = kana_state;
      ps2_update_leds(ps2numlockstate, !caps_state, !kana_state);
    } //if ( update_ps2_leds || (caps_state != caps_former) || (kana_state != kana_former) )

    //Keep RX serial buffer empty and echoes to output
    while(con_available_get_char())
    {
      uint8_t ch, m_str[4];

      ch = con_get_char();
      m_str[0] = ch;
      m_str[1] = 0;
      //Don't send X_OFF to avoid to lock the remote terminal, if it has X_ON/X_OFF enabled
      if(ch != X_OFF)
        con_send_string(m_str);
      else
        con_send_string((uint8_t*)"<X_OFF>");
    }

#if USE_USB == true
    if(!usb_configured_prev && usb_configured)
      con_send_string((uint8_t*)"\r\n\n. USB has been enumerated => Console and UART are now over USB!\r\n");
    usb_configured_prev = usb_configured;
#endif  //#if USE_USB = true

  } //for(;;)

  return 0; //Suppose never reach here
} //int main(void)

/// @brief Mark an end of code
///
/// @param *reset_org Pointer to get which was the cause of reset
void end_of_code(uint32_t*reset_org)
{
  //Query the reset cause
  *reset_org = RCC_CSR;
  RCC_CSR = RCC_CSR_RMVF;
} //int main(void)

