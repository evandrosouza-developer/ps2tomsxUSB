/** @addtogroup  07 hr_timer High Resolution Timer
 *
 * @file hr_timer.c High Resolution Timer 1us delay and PS/2 clocks measurements routines.
 *
 * @brief <b>High Resolution Timer 1us delay and PS/2 clocks measurements routines.</b>
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
 * This file is part of PS/2 to MSX keyboard adapter
 * This file was part of the libopencm3 project.
 *
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
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

#include "hr_timer.h"

volatile uint16_t state_overflow_tim2;
volatile uint64_t TIM2_Update_Cnt;                //Overflow of time_between_ps2clk
volatile uint64_t time_between_ps2clk ;
volatile uint64_t acctimeps2data0;

//Local Prototypes
void time_capture(void);
void (*next_routine) (void);


void tim_hr_setup(uint32_t timer_peripheral)
{
  //Used to receive PS/2 Clock interrupt

  // Enable TIM2 clock
  rcc_periph_clock_enable(RCC_TIM2);

  /* Reset TIM2 peripheral to defaults. */
  rcc_periph_reset_pulse(RST_TIM2);

  //PS/2 Clock interrupt
  // Enable TIM2 interrupt.
  nvic_enable_irq(NVIC_TIM2_IRQ);
  nvic_set_priority(NVIC_TIM2_IRQ, IRQ_PRI_TIM_HR);

  //Configuring PA15 as input and Alternate function of T2C1
  //gpio_mode_setup(PS2_CLK_I_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PS2_CLK_I_PIN);  //Default config
  //gpio_set_af(PS2_CLK_I_PORT, GPIO_AF1, PS2_CLK_I_PIN);

  /* Timer global mode:
   * - Prescaler = 84 (Prescler module=83)
   * - Counter Enable
   * - Direction up
   */
  timer_set_mode(timer_peripheral, TIM_CR1_CKD_CK_INT, TIM_CR1_CEN, TIM_CR1_DIR_UP);

  /*
   * Please take note that the clock source for STM32 timers
   * might not be the raw APB1/APB2 clocks.  In various conditions they
   * are doubled.  See the Reference Manual for full details!
   * In our case, TIM2 on APB1 is running at double frequency
   */
  timer_set_prescaler(timer_peripheral, ((rcc_apb2_frequency * 2) / 1000000) - 1);

  // count full range
  timer_set_period(timer_peripheral, 0xFFFFFFFF);
  //TIM_ARR(timer_peripheral) = 0xFFFFFFFF;

  // Enable preload.
  timer_enable_preload(timer_peripheral);

  /*
  TIM2_Update_Cnt = 0;
  time_between_ps2clk = 0;
  state_overflow_tim2 = TIME_CAPTURE;

  //. Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S
  //bits to 01 in the TIMx_CCMR1 register. As soon as CC1S becomes different from 00,
  //the channel is configured in input and the TIMx_CCR1 register becomes read-only.
  //Bits 1:0 CC1S: Capture/Compare 1 selection
  //This bit-field defines the direction of the channel (input/output) as well as the used input.
  //00: CC1 channel is configured as output.
  //01: CC1 channel is configured as input, IC1 is mapped on TI1.
  //10: CC1 channel is configured as input, IC1 is mapped on TI2.
  //11: CC1 channel is configured as input, IC1 is mapped on TRC. This mode is working only if
  //an internal trigger input is selected through TS bit (TIMx_SMCR register)
  //Note: CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER)
  TIM_CCER(TIM2) = 0;//To zero TIM_CCER_CC1E and prepair to select active input
  TIM_CCMR1(TIM2) = TIM_CCMR1_CC1S_IN_TI1;
  //. Select the edge of the active transition on the TI1 channel by writing the CC1P bit to 1
  //in the TIMx_CCER register (falling edge in this case).
  //Bit 1 CC1P: Capture/Compare 1 configured as input:
  //This bit selects whether IC1 is used for trigger or capture operations.
  //0: non-inverted: capture is done on a rising edge of IC1. When used as external trigger, IC1 is non-inverted.
  //1: inverted: capture is done on a falling edge of IC1. When used as external trigger, IC1 is inverted.
  TIM_CCER(TIM2) = TIM_CCER_CC1P;
  //. Program the input prescaler. In our use, we wish the capture to be performed at
  //each valid transition, so the prescaler is disabled (write IC1PS bits to 00 in the
  //TIMx_CCMR1 register).
  //Bits 3:2 IC1PSC: Input capture 1 prescaler
  //This bit-field defines the ratio of the prescaler acting on CC1 input (IC1).
  //The prescaler is reset as soon as CC1E=0 (TIMx_CCER register).
  //00: no prescaler, capture is done each time an edge is detected on the capture input
  //01: capture is done once every 2 events
  //10: capture is done once every 4 events
  //11: capture is done once every 8 events
  TIM_CCMR1(TIM2) &= (TIM_CCMR1_IC1F_OFF | 0b11110011); //No prescaler and no filter, sampling is done at fDTS
  */

  //Here there is no timer_peripheral run and/or interrupts enabled
  prepares_capture(timer_peripheral);
}


//void delay_usec(uint16_t usec, int16_t next_state)
void delay_usec(uint32_t timer_peripheral, uint16_t usec, void next_step (void))
{
  //Here we init TIM2 to a single shot,
  //and have an interrupt at the counting overflow

  // Counter disable
  TIM_CR1(timer_peripheral) &= ~TIM_CR1_CEN;

  //Disable capture from the counter into the capture register by setting the CC1E bit in the TIMx_CCER register.
  //Bit 0 CC1E: Capture/Compare 1 configured as input:
  //This bit determines if a capture of the counter value can actually be done into the input 
  //capture/compare register 1 (TIMx_CCR1) or not.
  //0: Capture disabled.
  //1: Capture enabled.
  TIM_CCER(timer_peripheral) &= ~TIM_CCER_CC1E;

  // Bit 2 URS: Update request source
  // This bit is set and cleared by software to select the UEV event sources.
  // 0: Any of the following events generate an update interrupt or DMA request if enabled.
  // These events can be:
  // – Counter overflow/underflow
  // – Setting the UG bit
  // – Update generation through the slave mode controller
  // 1: Only counter overflow/underflow generates an update interrupt or DMA request if enabled.  
  TIM_CR1(timer_peripheral) |= TIM_CR1_URS;

  //Now clear timer through set UG:
  //Generate a UEV (UG bit in the TIMx_EGR register)
  //Bit 0 UG: Update generation
  //This bit can be set by software, it is automatically cleared by hardware.
  //0: No action
  //1: Re-initialize the counter and generates an update of the registers. Note that the prescaler
  //counter is cleared too (anyway the prescaler ratio is not affected). The counter is cleared if
  //the center-aligned mode is selected or if DIR=0 (upcounting), else it takes the auto-reload
  //value (TIMx_ARR) if DIR=1 (downcounting).
  TIM_EGR(timer_peripheral) |= TIM_EGR_UG;
  // Clear timer Update Interrupt Flag after set UG (Update Generation)
  //timer_clear_flag(TIM2, TIM_SR_UIF);
  TIM_SR(timer_peripheral) &= ~TIM_SR_UIF;

  //timer_set_period(TIM2, 0xFFFFFFFF-usec-1);
  TIM_CNT(timer_peripheral) = 0xFFFFFFFF + 1 - usec;

  next_routine = next_step;

  //Enables OPM (One Pulse Mode)
  TIM_CR1(timer_peripheral) |= TIM_CR1_OPM;

  // Counter enable
  //timer_enable_counter(TIM2);
  TIM_CR1(timer_peripheral) |= TIM_CR1_CEN;

  //timer_enable_irq(TIM2, TIM_DIER_UIE);
  TIM_DIER(timer_peripheral) |= TIM_DIER_UIE;
}


void prepares_capture(uint32_t timer_peripheral)
{
  TIM_CR2(timer_peripheral) &= ~TIM_CR2_TI1S;
  TIM_CR2(timer_peripheral) = 0;

  // Dummy read the Input Capture value (to clear CC1IF flag)
  TIM_CCR1(timer_peripheral);
  //Clear TIM2 Capture compare interrupt pending bit
  //timer_clear_flag(timer_peripheral, TIM_SR_CC1IF);             //if the above dummy read didn`t do the task
  //timer_clear_flag(timer_peripheral, TIM_SR_CC1OF);
  TIM_SR(timer_peripheral) &= ~(TIM_SR_CC1IF | TIM_SR_CC1OF);         //if the above dummy read didn`t do the task

  //. Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S
  //bits to 01 in the TIMx_CCMR1 register. As soon as CC1S becomes different from 00,
  //the channel is configured in input and the TIMx_CCR1 register becomes read-only.
  //Bits 1:0 CC1S: Capture/Compare 1 selection
  //This bit-field defines the direction of the channel (input/output) as well as the used input.
  //00: CC1 channel is configured as output.
  //01: CC1 channel is configured as input, IC1 is mapped on TI1.
  //10: CC1 channel is configured as input, IC1 is mapped on TI2.
  //11: CC1 channel is configured as input, IC1 is mapped on TRC. This mode is working only if
  //an internal trigger input is selected through TS bit (TIMx_SMCR register)
  //Note: CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER)
  TIM_CCER(timer_peripheral) = 0;//To zero TIM_CCER_CC1E and prepair to select active input
  TIM_CCMR1(timer_peripheral) = TIM_CCMR1_CC1S_IN_TI1;
  //. Select the edge of the active transition on the TI1 channel by writing the CC1P bit to 1
  //in the TIMx_CCER register (falling edge in this case).
  //Bit 1 CC1P: Capture/Compare 1 configured as input:
  //This bit selects whether IC1 is used for trigger or capture operations.
  //0: non-inverted: capture is done on a rising edge of IC1. When used as external trigger, IC1 is non-inverted.
  //1: inverted: capture is done on a falling edge of IC1. When used as external trigger, IC1 is inverted.
  TIM_CCER(timer_peripheral) = TIM_CCER_CC1P;
  //. Program the input prescaler. In our use, we wish the capture to be performed at
  //each valid transition, so the prescaler is disabled (write IC1PS bits to 00 in the
  //TIMx_CCMR1 register).
  //Bits 3:2 IC1PSC: Input capture 1 prescaler
  //This bit-field defines the ratio of the prescaler acting on CC1 input (IC1).
  //The prescaler is reset as soon as CC1E=0 (TIMx_CCER register).
  //00: no prescaler, capture is done each time an edge is detected on the capture input
  //01: capture is done once every 2 events
  //10: capture is done once every 4 events
  //11: capture is done once every 8 events
  TIM_CCMR1(timer_peripheral) &= (TIM_CCMR1_IC1F_OFF | 0b11110011); //No prescaler and no filter, sampling is done at fDTS

  //Reenable TIM2 as Capture Timer:
  //Enable capture from the counter into the capture register by setting the CC1E bit in the TIMx_CCER register.
  //Bit 0 CC1E: Capture/Compare 1 configured as input:
  //This bit determines if a capture of the counter value can actually be done into the input 
  //capture/compare register 1 (TIMx_CCR1) or not.
  //0: Capture disabled.
  //1: Capture enabled.
  TIM_CCER(timer_peripheral) |= TIM_CCER_CC1E;
  timer_set_period(timer_peripheral, 0xFFFFFFFF);
  //Disables OPM (One Pulse Mode)
  TIM_CR1(timer_peripheral) &= ~TIM_CR1_OPM;
  //Setup counters
  time_between_ps2clk = 0;
  acctimeps2data0 = 0;
  TIM2_Update_Cnt = 0;
  //Now points UIF (overflow int) to "normal" state: TIME_CAPTURE
  //state_overflow_tim2 = TIME_CAPTURE;
  next_routine = time_capture;

  //Clear TIM2_CR1_URS;
  TIM_CR1(timer_peripheral) &= ~TIM_CR1_URS;

  //Counter enable
  //timer_enable_counter(timer_peripheral);
  TIM_CR1(timer_peripheral) |= TIM_CR1_CEN;

  timer_enable_irq(timer_peripheral, TIM_DIER_CC1IE | TIM_DIER_UIE);
}


void time_capture(void)
{
  TIM2_Update_Cnt++;
}



/*************************************************************************************************/
/*************************************************************************************************/
/******************************************* ISR's ***********************************************/
/*************************************************************************************************/
/*************************************************************************************************/
void tim2_isr(void)
{
  //Verify if Update Interrupt Flag
  if (timer_get_flag(TIM2, TIM_SR_UIF))
  {
    //Debug & performance measurement
    gpio_clear(INT_TIM2_PORT, TIM2UIF_PIN); //Signs start of interruption

    // Clear timer Update Interrupt Flag
    timer_clear_flag(TIM2, TIM_SR_UIF);
    next_routine();

    //Debug & performance measurement
    gpio_set(INT_TIM2_PORT, TIM2UIF_PIN); //Signs end of interruption
  } //if (timer_get_flag(TIM2, TIM_SR_UIF))
  else if (timer_get_flag(TIM2, TIM_SR_CC1IF))
  {
    //When an input capture occurs:
    //. The TIMx_CCR1 register gets the value of the counter on the active transition.
    //. CC1IF flag is set (interrupt flag). CC1OF is also set if at least two consecutive captures
    //occurred whereas the flag was not cleared.
    //. An interrupt is generated depending on the CC1IE bit.
    //. A DMA request is generated depending on the CC1DE bit.
    //In order to handle the overcapture, it is recommended to read the data before the
    //overcapture flag. This is to avoid missing an overcapture which could happen after reading
    //the flag and before reading the data.
    //Note: IC interrupt and/or DMA requests can be generated by software by setting the
    //corresponding CCxG bit in the TIMx_EGR register.
    
    //Debug & performance measurement
    gpio_clear(INT_TIM2_PORT, TIM2CC1_PIN); //Signs start of interruption

    // Clear TIM2 Capture compare interrupt pending bit
    //timer_clear_flag(TIM2, TIM_SR_CC1OF |  TIM_SR_CC1IF);
    TIM_SR(TIM2) &= ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
    // Get the Input Capture value
    time_between_ps2clk = TIM2_Update_Cnt << 32 | TIM_CCR1(TIM2);
    //Clear Counter2 (Reset both CNT & PSC plus )
    TIM_CR1(TIM2) |= TIM_CR1_URS; 
    TIM_EGR(TIM2) |= TIM_EGR_UG;
    TIM_SR(TIM2) &= ~TIM_SR_UIF;
    //Clear TIM2_CR1_URS;
    TIM_CR1(TIM2) &= ~TIM_CR1_URS;

    TIM2_Update_Cnt = 0;

    //This is the ISR of PS/2 clock pin. It jumps to ps2_clock_update.
    //It is an important ISR, but it does not require critical timming resources as MSX Y scan does.
    // EXTI15 & TIM2_CCR1
    bool ps2datapin_logicstate=gpio_get(PS2_DATA_PORT, PS2_DATA_PIN);
    ps2_clock_update(ps2datapin_logicstate);

    //Debug & performance measurement
    gpio_set(INT_TIM2_PORT, TIM2CC1_PIN); //Signs end of TIM2 interruption. Default condition is "1"
  }
}
