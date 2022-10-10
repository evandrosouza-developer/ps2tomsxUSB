/** @addtogroup 03 USART USART_Group
 *
 * @ingroup infrastructure_apis
 *
 * @file serial.c USART with DMA support routines on STM32F1 and STM32F4.
 *
 * @brief <b>USART with DMA support routines on STM32F1 and STM32F4</b>
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


//Use Tab width=2

#include "serial.h"

// See the inspiring file:
// https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/usart_irq_printf/usart_irq_printf.c

bool enable_xon_xoff = true, xon_condition = true, xoff_condition = false, xonoff_sendnow = false;

/**
 * @brief Defines line_coding structure.
 * 
 */
struct sring uart_tx_ring;
struct sring uart_rx_ring;
uint8_t uart_tx_ring_buffer[UART_TX_RING_BUFFER_SIZE];
uint8_t uart_rx_ring_buffer[UART_RX_RING_BUFFER_SIZE];
struct sring dma_rx_ring;
uint8_t buf_dma_rx[RX_DMA_SIZE];
/**
 * Number of bytes requested to transmit by DMA (Global variable)
 * 
 * As there is no dedicated dma tx buffer, dma process shares uart_tx_ring_buffer with other processes that write on it.
 * It brings information of how many bytes dma has just sent to be used on ISR_DMA_CH_USART_TX to update
 * uart_tx_ring.get_ptr only after DMA process is finished.
 * So, to clear room for new writings only after dma is concluded, it assures no loss of information and optimize uart_tx_ring_buffer usage.
 */
uint16_t last_dma_tx_set_number_of_data;

//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by DMA_STREAM2

#if USE_USB == true
struct sring con_tx_ring;
struct sring con_rx_ring;
uint8_t con_tx_ring_buffer[CON_TX_RING_BUFFER_SIZE];
uint8_t con_rx_ring_buffer[CON_RX_RING_BUFFER_SIZE];

extern  bool nak_cleared[6];                    //Declared on cdcacm.c
extern  int usb_configured;                     //Declared on cdcacm.c
#endif  //#if USE_USB == true
bool  ok_to_rx;


void ring_init(struct sring *ring, uint8_t *buf, uint16_t buffer_size)
{
  ring->data = buf;
  ring->bufSzMask = buffer_size - 1;            //buffer_size_mask;
  ring->put_ptr = 0;
  ring->get_ptr = 0;
}


void pascal_string_init(struct s_pascal_string* ring, uint8_t* buf, uint8_t bufsize)
{
  ring->str_len = 0;
  ring->bufSzMask = bufsize - 1;
  ring->data = buf;
}

/*************************************************************************************************/
/******************************************* Setup ***********************************************/
/*************************************************************************************************/
//Ready to be used outside this module.
// Setup serial used in main.
void serial_setup(void)
{
  ok_to_rx = false;

  // Initialize input and output ring buffers.
  ring_init(&uart_tx_ring, uart_tx_ring_buffer, UART_TX_RING_BUFFER_SIZE);
  ring_init(&uart_rx_ring, uart_rx_ring_buffer, UART_RX_RING_BUFFER_SIZE);
#if USE_USB == true
  ring_init(&con_tx_ring, con_tx_ring_buffer, CON_TX_RING_BUFFER_SIZE);
  ring_init(&con_rx_ring, con_rx_ring_buffer, CON_RX_RING_BUFFER_SIZE);
#endif  //#if USE_USB == true

//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by DMA_STREAM2

  ring_init(&dma_rx_ring, buf_dma_rx, RX_DMA_SIZE);

  // Enable clocks for USART_PORT and DMA.
  rcc_periph_clock_enable(RCC_USART);
  //Enable clocks now for DMA
  rcc_periph_clock_enable(RCC_DMA);

  // Setup GPIO pin GPIO_USART_RX_TX on GPIO port A for transmit.
#if MCU == STM32F103
  gpio_set_mode(GPIO_BANK_USART_TX, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_PIN_USART_TX);
  gpio_set(GPIO_BANK_USART_RX, GPIO_PIN_USART_RX); //pull up resistor
  gpio_set_mode(GPIO_BANK_USART_RX, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO_PIN_USART_RX);
#endif  //#if MCU == STM32F103
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
#endif  //#if MCU == STM32F401

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
  //Now I don't have buf_dma_tx anymore. I use the availability of uart_tx_ring
  //dma_set_memory_address(USART_DMA_BUS, USART_DMA_TX_CH, (uint32_t)buf_dma_tx);
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
  dma_enable_direct_mode(USART_DMA_BUS, USART_DMA_TX_CH);   //not using DMA FIFO
#endif

  /* Technical information brief about USART RX technics:
   * As Serial input data behaviour is normally unknown (from a single reveived
   * byte to a complete file transfer), and to unload CPU processing, serial
   * input data would be done based on DMA, so some definitions had to be done:
   * * The DMA buffer is read at three different occasions:
   * * * 1) Whenever there is a Idle time (no new serial ativity was observed
   * * *    after the last character was completed);
   * * * 2) Half transfer and,
   * * * 3) Transfer completed, when there is s sustained communication stream.
   * * RX DMA is implemented using the concept of non stop running. It is
   *   programmed as circular mode, at address buf_dma_rx, which is part
   *   of the structure "dma_rx_ring". 
   * * This structure is the same as implemented at the serial buffers,
   *   as they have the size of the buf_dma_rx and the get_ptr. This get_ptr
   *   is used to point where is the begining address to read the dma buffer.
   *   The structure has also the put_ptr, managed by information originated
   *   from DMA, that is computed through a reading of DMA_SNDTR.
   * 
   * After implements this, it comes with a prize: A completely independent
   * process running in background!
  */
//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by DMA_STREAM2

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
#endif  //#if MCU == STM32F103
#if MCU == STM32F401
  dma_set_transfer_mode(USART_DMA_BUS, USART_DMA_RX_CH, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
  dma_channel_select(USART_DMA_BUS, USART_DMA_RX_CH, USART_DMA_TRG_CHSEL);
  dma_set_dma_flow_control(USART_DMA_BUS, USART_DMA_RX_CH);
  dma_enable_direct_mode(USART_DMA_BUS, USART_DMA_RX_CH);   //not using DMA FIFO
#endif  //#if MCU == STM32F401

  // Prepare RX DMA interrupts
  dma_clear_interrupt_flags(USART_DMA_BUS, USART_DMA_RX_CH, DMA_CGIF);
  nvic_set_priority(USART_DMA_RX_IRQ, IRQ_PRI_USART_DMA);
  nvic_enable_irq(USART_DMA_RX_IRQ);

//getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
//disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by DMA_STREAM2

  // Init dma tx communication variable
  last_dma_tx_set_number_of_data = 0;

  // Prepare TX DMA interrupts
  usart_disable_tx_complete_interrupt(USART_PORT);
  nvic_set_priority(USART_DMA_TX_IRQ, IRQ_PRI_USART_DMA);
  nvic_enable_irq(USART_DMA_TX_IRQ);

  // Enable the USART RX.
  usart_enable_idle_interrupt(USART_PORT); usart_disable_rx_interrupt(USART_PORT);
  usart_enable_rx_dma(USART_PORT);
  // Enable the DMA engine of USART RX. RX is never shutdown.
  dma_enable_ch(USART_DMA_BUS, USART_DMA_RX_CH);

  // Enable the USART RX.
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
#endif  //#if USE_USB == true


void serial_rx_restart(void)
{
  dma_disable_ch(USART_DMA_BUS, USART_DMA_RX_CH);
  //wait until it is really disabled
  while(DMA_CR(USART_DMA_BUS, USART_DMA_RX_CH) & DMA_CR_EN) __asm("nop");
  //Clear USART_SR_IDLE, to avoid IDLE new interrupts without new incoming chars.
  uint8_t bin = USART_SR(USART_PORT); bin = USART_DR(USART_PORT); bin &= 0xFF;
  dma_set_number_of_data(USART_DMA_BUS, USART_DMA_RX_CH, RX_DMA_SIZE);
  dma_rx_ring.put_ptr = 0;
  dma_rx_ring.get_ptr = 0;
  uart_rx_ring.put_ptr = 0;
  uart_rx_ring.get_ptr = 0;
#if USE_USB == true
  con_rx_ring.put_ptr = 0;
  con_rx_ring.get_ptr = 0;
#endif  //#if USE_USB == true
  dma_enable_ch(USART_DMA_BUS, USART_DMA_RX_CH);
}


/** @brief If DMA is idle, it will be set to the "get pointer" of the uart_tx_ring.
 *
 * @param number_of_data Number of data bytes to DMA to send.
This number will update the "get pointer" to restart TX.
 */
void do_dma_usart_tx_ring(uint16_t number_of_data)
{
  if(!dma_get_number_of_data(USART_DMA_BUS, USART_DMA_TX_CH))
  {
    dma_set_memory_address(USART_DMA_BUS, USART_DMA_TX_CH, (uint32_t)&uart_tx_ring.data[uart_tx_ring.get_ptr]);
    dma_set_number_of_data(USART_DMA_BUS, USART_DMA_TX_CH, number_of_data);
    last_dma_tx_set_number_of_data = number_of_data;
    dma_enable_ch(USART_DMA_BUS, USART_DMA_TX_CH);
    usart_enable_tx_dma(USART_PORT);
  }
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
  i = ring->put_ptr;                      //i is the original position
  i_next = (i + 1) & ring->bufSzMask; //i_next is the next position of i
  if(i_next != ring->get_ptr)
  {
    #if defined CHECK_INDEX
    check_idx_u16(i, (uintptr_t)ring->data, ring->bufSzMask+1);
    #endif
    ring->data[i] = ch;     //saves in the put_ptr position 
    ring->put_ptr = i_next; //now put_ptr points to the next position of i
    //Optimizing calculations inside the interrupt => The general formula is:
    //CharsInBuffer = (RING_BUFFER_SIZE - ring.get_ptr + ring.put_ptr) % RING_BUFFER_SIZE;
    //but BASE_RING_BUFFER_SIZE is a power of two, so the rest of the division is computed zeroing
    //the higher bits of the summed buffer lenght, so in the case of 256 (2**8), you have to keep only
    //the lowest 8 bits: (BASE_RING_BUFFER_SIZE - 1).
    return (uint16_t)(ring->bufSzMask + 1 - ring->get_ptr + i_next) & ring->bufSzMask;
  }
  else
  {
    //No room for more (Buffer full)
    return 0xFFFF;
  }
}


//Ready to be used outside this module.
// Put an ASCIIZ (uint8_t) string on serial buffer.
// Wait until buffer is filled.
void con_send_string(uint8_t* string)
{
  uint16_t iter = 0;

#if USE_USB == true
  uint16_t qtty;
  qtty =  QTTY_CHAR_IN(con_tx_ring);
#endif  //#if USE_USB == true
  CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
#if USE_USB == true
    if(usb_configured)
    {
      // Put char in console ring.
      ring_put_ch(&con_tx_ring, data);
      while(string[iter])
      {
        while((ring_put_ch(&con_tx_ring, (string[iter]))) == 0xFFFF) __asm("nop");
        iter++;
      }
    } //if(usb_configured)
    else  //else if(usb_configured)
    {
      // Put char in console ring.
      ring_put_ch(&uart_tx_ring, data);
      while(string[iter])
      {
        // Put char in uart_tx_ring.
        while((ring_put_ch(&uart_tx_ring, (string[iter]))) == 0xFFFF)
          do_dma_usart_tx_ring(iter + sizeof(data));
        iter++;
      }
      do_dma_usart_tx_ring(iter + sizeof(data));
    } //else if(usb_configured)
#else   //#else if USE_USB == true
    ring_put_ch(&uart_tx_ring, data);
    while(string[iter])
    {
      while((ring_put_ch(&uart_tx_ring, (string[iter]))) == 0xFFFF)
        do_dma_usart_tx_ring(iter + sizeof(data));
      iter++;
    }
    //And start TX DMA with this count, if it is idle
    do_dma_usart_tx_ring(iter + sizeof(data));
#endif  //#else if USE_USB == true
  CHECK_XONXOFF_SENDNOW_CLOSE_BRACKET //CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
  else  //CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
  {
#if USE_USB == true
    if(usb_configured)
    {
      while(string[iter])
      {
        // Put char in console ring.
        while((ring_put_ch(&con_tx_ring, (string[iter]))) == 0xFFFF) __asm("nop");
        iter++;
      }
    }
    else  //if(usb_configured)
    {
      while(string[iter])
      {
        // Put char in uart_tx_ring.
        while((ring_put_ch(&uart_tx_ring, (string[iter]))) == 0xFFFF)
          do_dma_usart_tx_ring(iter);
        iter++;
      }
      do_dma_usart_tx_ring(iter);
    }
#else   //#else if USE_USB == true
    while(string[iter])
    {
      // Put char in uart_tx_ring.
      while((ring_put_ch(&uart_tx_ring, (string[iter]))) == 0xFFFF)
        do_dma_usart_tx_ring(iter);
      iter++;
    }
    do_dma_usart_tx_ring(iter);
#endif  //#else if USE_USB == true
  } //CHECK_XONXOFF_SENDNOW_START_WITH_OPEN_BRACKET
#if USE_USB == true
  //If the quantity before filled was zero, means that first transmition is necessary. Afterwards, the CB will handle transmition.
  if(usb_configured && !qtty)
    first_put_ring_content_onto_ep(&con_tx_ring, EP_CON_DATA_IN);
#endif  //#if USE_USB == true
}



//---------------------------------------------------------------------------------------
//----------------------------Communication input routines-------------------------------
//---------------------------------------------------------------------------------------
//Used as an internal function.
//Returns true if there is a char available to read in the ring (both TX and RX) or false if not.
uint16_t ring_avail_get_ch(struct sring *ring)
{
  return (ring->bufSzMask + 1 -ring->get_ptr + ring->put_ptr) & ring->bufSzMask;
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
  ring->get_ptr = local_get_ptr & ring->bufSzMask; //if(local_get_ptr >= (uint16_t)BASE_RING_BUFFER_SIZE) i = 0;
  *qty_in_buffer = (ring->bufSzMask + 1 -ring->get_ptr + ring->put_ptr) & ring->bufSzMask;
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
#else //#if USE_USB == true
  return (ring_avail_get_ch(&uart_rx_ring));
#endif  //  //#if USE_USB == true
}


//xon_xoff_control
static void xon_xoff_rx_control(struct sring *ring, uint16_t qty_in_buffer)
{
  if (enable_xon_xoff)
  {
    if (qty_in_buffer >= (3 * ring->bufSzMask / 4)) //X_OFF_TRIGGER)
    {
      xon_condition = false;
      if (!xoff_condition)                          //To send X_OFF only once
      {
        xoff_condition = true;
        xonoff_sendnow = true;
      }
    }
    else if (qty_in_buffer < (ring->bufSzMask / 2)) //(uint16_t)X_ON_TRIGGER)
    {
      xoff_condition = false;
      if (!xon_condition)                           //To send X_ON only once
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
    if ( (*qty_in_buffer >= (3 * con_rx_ring.bufSzMask / 4)) && (nak_cleared[EP_CON_DATA_OUT]) )
      //con_rx_ring is running out of space. Set NAK on endpoint.
      set_nak_endpoint(EP_CON_DATA_OUT);
    else if ( (*qty_in_buffer < (con_rx_ring.bufSzMask / 2)) && (!nak_cleared[EP_CON_DATA_OUT]) )
      //Now con_rx_ring has space. Clear NAK on endpoint.
      clear_nak_endpoint(EP_CON_DATA_OUT);
    return ch;
  } //if(usb_configured)
  else
  { //else if(usb_configured)
    ch = ring_get_ch(&uart_rx_ring, qty_in_buffer);
    xon_xoff_rx_control(&uart_rx_ring, *qty_in_buffer);
    return ch;
  } //else if(usb_configured)
#else //#else #if USE_USB == true
  ch = ring_get_ch(&uart_rx_ring, qty_in_buffer);
  xon_xoff_rx_control(&uart_rx_ring, *qty_in_buffer);
  return ch;
#endif  //#else #if USE_USB == true
}


//Ready to be used from outside of this module.
// If there is an available char in serial, it returns with an uint8_t.
//You MUST use the above function "con_available_get_char" BEFORE this one,
//in order to certificate a valid available reading by this function.
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
#else //#if USE_USB == true
  while(uart_rx_ring.get_ptr == uart_rx_ring.put_ptr) __asm("nop");
  return(con_get_char());
#endif  //#if USE_USB == true
}


// Read a line from serial. You can limit how many chars will be available to enter.
// It returns how many chars were read. It is a blocking function
//It's correct here too: The same call to both reception rings!
uint8_t console_get_line(uint8_t *s, uint16_t len)
{
  uint8_t *t = s;
  uint8_t ch, mount_t[3];
  bool  valid_decimal_digit;

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
//===========================Miscellaneous routines Group================================
//=======================================================================================

// Append an ASCIIZ (uint8_t) string at the end of str_mount_buff.
void string_append(uint8_t *string_org, struct s_pascal_string *str_mount_buff)
{
  uint16_t i = 0;
  while(string_org[i] && (str_mount_buff->str_len < (str_mount_buff->bufSzMask + 1)))
  {
    #if defined CHECK_INDEX
    check_idx_u16(str_mount_buff->str_len + 1, (uintptr_t)str_mount_buff, str_mount_buff->bufSzMask + 1);
    #endif
    str_mount_buff->data[str_mount_buff->str_len++] = string_org[i];
    str_mount_buff->data[str_mount_buff->str_len] = 0;
    i++;
  }
}


//Ready to be used outside this module.
//Force next console reading ch.
//Obs.: It assumes that console buffer is empty, to do next console reading be what you put.
void insert_in_con_rx(uint8_t ch)
{
#if USE_USB == true
  if(usb_configured)
    ring_put_ch(&con_rx_ring, ch);
  else
    ring_put_ch(&uart_rx_ring, ch);
#else   //#if USE_USB == true
  ring_put_ch(&uart_rx_ring, ch);
#endif  //#if USE_USB == true
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
  for(iter=0; iter<=10; iter++) //Fill outstring with \0
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
      } //if (value > power10[iter])
      if (insertzero)
        pos++;
    } //for(iter=0; iter<5; iter++)
  }
}


//Ready to be used outside this module.
// Convert a byte (8 bit binary) to into a 2 char string. 
void conv_uint8_to_2a_hex(uint8_t value, uint8_t *outstring)
{
  uint8_t iter;

  //end of string
  outstring += 2;
  *(outstring--) = 0;

  for(iter=0; iter<2; iter++)
  {
    *(outstring--) = (((value&0xf) > 0x9) ? (0x40 + ((value&0xf) - 0x9)) : (0x30 | (value&0xf)));
    value >>= 4;
  }
}



//Check if uint16_t index is inside bounds
void check_idx_u16(uint16_t idx_u16, uintptr_t base_u32, uint16_t size)
{
  if(idx_u16 >= size)
  {
    uint8_t str_mount[16];
    con_send_string((uint8_t*)"\r\nThis index is out-of-range: ");
    conv_uint16_to_4a_hex(idx_u16, str_mount);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)", while var base 0x");
    conv_uint32_to_8a_hex(base_u32, str_mount);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)" was created with a size of ");
    conv_uint16_to_4a_hex(size, str_mount);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)".");
    idx_u16 = size;

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
  //getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
  //disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by interrupt
//same function used at:
//Idle time detected on USART RX ISR and for DMA USART RX ISR
static void usart_rx_read_dma(void)
{
  uint16_t qtty_dma_rx, qtty_uart_rx, dma_get;

  //Put in uart_rx_ring what was received from USART via DMA
  dma_get = dma_get_number_of_data(USART_DMA_BUS, USART_DMA_RX_CH);

  dma_rx_ring.put_ptr =  (dma_rx_ring.bufSzMask + 1 - dma_get) & dma_rx_ring.bufSzMask;
  qtty_dma_rx = QTTY_CHAR_IN(dma_rx_ring);

  qtty_uart_rx = QTTY_CHAR_IN(uart_rx_ring);

  // Copy data from DMA buffer (dma_rx_ring) into USART RX buffer (uart_rx_ring)
  //Check if there is enough room in uart_rx_ring to receive qtty_dma_rx
  if(qtty_dma_rx > (uart_rx_ring.bufSzMask + 1 - qtty_uart_rx))
    qtty_dma_rx = (uart_rx_ring.bufSzMask + 1 - qtty_uart_rx);  //Discard the excess
  
  //Compute and run how many segments we have to transfer
  if((uart_rx_ring.put_ptr + qtty_dma_rx) < (uart_rx_ring.bufSzMask + 1)){
    //1 segment of memcpy (copy from buf_dma_rx to uart_rx_ring does not reach uart_rx_ring.bufSzMask)
    memcpy(&uart_rx_ring.data[uart_rx_ring.put_ptr], &dma_rx_ring.data[dma_rx_ring.get_ptr], (size_t)qtty_dma_rx);
  }
  else {
    //2 segments of memcpy
    uint32_t partial1 = (size_t)(uart_rx_ring.bufSzMask - uart_rx_ring.put_ptr + 1);
    //First one: from dma_rx_ring.data[uart_rx_ring.put_ptr] to uart_rx_ring.data[bufSzMask]
    memcpy(&uart_rx_ring.data[uart_rx_ring.put_ptr], &dma_rx_ring.data[dma_rx_ring.get_ptr], partial1);
    //Second one: from dma_rx_ring.data[0], moving (qtty_dma_rx - (uart_rx_ring.bufSzMask - uart_rx_ring.put_ptr)) bytes.
    uint32_t partial2 = (size_t)(qtty_dma_rx - partial1);
    memcpy(uart_rx_ring.data, &dma_rx_ring.data[dma_rx_ring.get_ptr + (uint16_t)partial1], partial2);
  }
  //Update uart_rx_ring.put_ptr after fill dma_rx_ring from contents of uart_rx_ring
  uart_rx_ring.put_ptr = ((uart_rx_ring.put_ptr + qtty_dma_rx) & uart_rx_ring.bufSzMask);

  //Update dma get pointer for the next DMA reading
  dma_rx_ring.get_ptr = (dma_rx_ring.get_ptr + qtty_dma_rx) & (dma_rx_ring.bufSzMask);

  //Now clear USART_SR_IDLE, to avoid IDLE new interrupts without new incoming chars.
  //It will be processed through a read to the USART_SR register followed by a read to the USART_DR register.
  uint8_t bin = USART_SR(USART_PORT); bin = USART_DR(USART_PORT); bin &= 0xFF;  //&= to avoid unused var warning 
  dma_clear_interrupt_flags(USART_DMA_BUS, USART_DMA_RX_CH, DMA_CGIF);

#if USE_USB == true
  //If the quantity before filled was 0, means that first transmition is necessary. So start it.
  //After that, the CB will be in charge of handling the transmition.
  if(usb_configured && !qtty_uart_rx && (uart_rx_ring.get_ptr != uart_rx_ring.put_ptr))
    first_put_ring_content_onto_ep(&uart_rx_ring, EP_UART_DATA_IN);
#endif  //#if USE_USB == true
}



ISR_USART
{
  //getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
  //disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by STREAM2.

  //Check if Idle time detected on USART RX:
  if( (USART_CR1(USART_PORT)&USART_CR1_IDLEIE) && (USART_SR(USART_PORT)&USART_SR_IDLE) )
  {
    //Idle time detected
    usart_rx_read_dma();
  }

  // Check if we were called because of RXNE.
  if( (USART_CR1(USART_PORT)&USART_CR1_RXNEIE) && (USART_SR(USART_PORT)&USART_SR_RXNE) )
  {
    //Suppose not be here, but in case of, clear USART error flags
    uint8_t bin = USART_SR(USART_PORT); bin = USART_DR(USART_PORT); bin &= 0xFF;
  }
}



  //getting number of data from STREAM5 does not work, as it seems to be initialized as 0xFFFF,
  //disconsidering what you put. This case (STM32F401 & USART1 RX) will be supported by STREAM2.
//ISR for DMA USART RX => Transfer completed and half transfer
ISR_DMA_CH_USART_RX
{
  usart_rx_read_dma();
}




//ISR for DMA USART TX
ISR_DMA_CH_USART_TX
{
  uint16_t to_put_in_dma_tx;
  
  //Stop DMA
  usart_disable_tx_dma(USART_PORT);
  dma_disable_ch(USART_DMA_BUS, USART_DMA_TX_CH);//DMA disable transmitter
  dma_clear_interrupt_flags(USART_DMA_BUS, USART_DMA_TX_CH, DMA_CGIF);

  //Update uart_tx_ring.get_ptr with last_dma_tx_set_number_of_data.
  uart_tx_ring.get_ptr = (uart_tx_ring.get_ptr + last_dma_tx_set_number_of_data) & uart_tx_ring.bufSzMask;

  last_dma_tx_set_number_of_data = 0;

  if(!QTTY_CHAR_IN(uart_tx_ring))
    //Return with DMA disabled, as it it not necessary anymore. 
    return;

  //I will try to send all content of the buffer, but circular buffers may have the condition of
  //put_ptr be lower than get_ptr. In ths case, I will start to transfer from get_ptr to the upper
  //physical position of the buffer (uart_tx_ring.bufSzMask).
  // Compute new size:
  if(uart_tx_ring.put_ptr < uart_tx_ring.get_ptr)
    to_put_in_dma_tx = uart_tx_ring.bufSzMask - uart_tx_ring.get_ptr + 1;
  else
    to_put_in_dma_tx = uart_tx_ring.put_ptr - uart_tx_ring.get_ptr;
  
  if(!to_put_in_dma_tx)   //Second check to avoid write 0 bytes to DMA
    //Return with DMA disabled, as it it not necessary anymore. 
    return;

  //And so, reinit DMA.
  dma_set_memory_address(USART_DMA_BUS, USART_DMA_TX_CH, (uintptr_t)&uart_tx_ring.data[uart_tx_ring.get_ptr]);
  dma_set_number_of_data(USART_DMA_BUS, USART_DMA_TX_CH, to_put_in_dma_tx);
  last_dma_tx_set_number_of_data = to_put_in_dma_tx;
  dma_enable_ch(USART_DMA_BUS, USART_DMA_TX_CH);
  usart_enable_tx_dma(USART_PORT);
}
