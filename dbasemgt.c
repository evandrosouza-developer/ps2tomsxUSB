/** @addtogroup 05 dbasemgt Database Management
 *
 * @file dbasemgt.c Database check and maintenance routines.
 *
 * @brief <b>Database check and maintenance routines.</b>
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
 * This file is part of the PS/2 to MSX keyboard Converter and 
 * MSX Keyboard Subsystem Emulator projects, using libopencm3 project.
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


//This original SW is compiled to a Sharp/Epcom MSX HB-8000 and a brazilian ABNT2 PS/2 keyboard (ID=275)
//But it is possible to update the table sending a Intel Hex File through serial, using the
//following communication configuration:
//Speed: 115200 bps
//8 bits
//No parity
//1 Stop bit
//no hardware flow control


#include "dbasemgt.h"


//Processor related sizes and adress:
#define STRING_MOUNT_BUFFER_SIZE  20
#define MAX_ERASE_TRIES           3
#define FLASH_WRONG_DATA_WRITTEN  0x80
#define RESULT_OK                 0

//Global var area:
bool            flash_locked, compatible_database;
extern uint8_t  UNUSED_DATABASE[DB_NUM_COLS];     //Declared on msxmap.cpp
extern uint32_t *base_of_database;                //Declared on msxmap.cpp
extern bool     update_ps2_leds;                  //Declared on msxmap.cpp
extern uint8_t  y_dummy;                          //Declared on msxmap.cpp
extern bool     enable_xon_xoff;                  //Declared on serial.c
extern bool     ps2numlockstate;                  //Declared on ps2handl.c



#if MCU == STM32F103


//Prototype area:
uint32_t flash_program_data(uint8_t*);
void check_flash_error(void);
void check_flash_locked(void);
bool check_page_erased(uint32_t, bool*, uint16_t*);



void database_setup(void)
{
  uint8_t ch, str_mount[STRING_MOUNT_BUFFER_SIZE];
  uint32_t iter;
  volatile uint32_t*base_of_database32;
  volatile uint8_t*base_of_database8;
  void *void_ptr;
  bool sector_erased = true;
  uint16_t attempts_erasing_page;

  compatible_database = true;
  flash_locked = true;
  
  //Any character received via serial during the time of waiting PS/2 BAT enables Cleanup Database Flash
  if (con_available_get_char())
  {
    //Clear Serial RX Buffer
    while(con_available_get_char())
      ch = con_get_char();
    con_send_string((uint8_t*)"\r\nCleanup Database Flash. Send ""&"" to proceed or any other key to abort ");
    //Read a key
    while (!con_available_get_char()) __asm("nop");
    ch = con_get_char();
    str_mount[0] = ch;
    str_mount[1] = 0;
    con_send_string(str_mount);
    if(ch == '&')
    {
      //Information to user
      con_send_string((uint8_t*)"\r\nErasing flash memory...\r\nBase Address  Size  Status\r\n");
      for(uint16_t sect_num = DATABASE_BASE_PAGE; sect_num < (DATABASE_TOP_PAGE+1); sect_num++)
      {
        //Erase pages from 24 to 31
        attempts_erasing_page = 0;
        while (attempts_erasing_page < MAX_ERASE_TRIES) //3 tries
        {
          //Information to user
          void_ptr = &str_mount;
          conv_uint32_to_8a_hex(((uint32_t)(sect_num*FLASH_PAGE_SIZE)), void_ptr);
          con_send_string((uint8_t*)" 0x");
          con_send_string((uint8_t*)str_mount);
          con_send_string((uint8_t*)"   ");
          conv_uint32_to_dec((uint32_t)FLASH_PAGE_SIZE, void_ptr);
          con_send_string((uint8_t*)str_mount);
          con_send_string((uint8_t*)"  ");
          //Cleaning is needed only if it is not erased. First check
          sector_erased = true;
          uint8_t *page_fl = (uint8_t *)(sect_num * FLASH_PAGE_SIZE);
          for (iter = 0; iter < FLASH_PAGE_SIZE; iter++)  //All page bytes must be checked
          {
            if( (*(volatile uint8_t *)(page_fl + iter)) != 0xFF )
            {
              sector_erased = false;
              break;  //quit "for (uint16_t iter = 0; iter < FLASH_PAGE_SIZE; iter++)"
            } //if( (*(uint32_t *)(DATABASE_BASE_ADD + iter)) != 0xFFFFFFFF )
          } //for (iter = 0; iter < FLASH_PAGE_SIZE; iter++)
          if (!sector_erased) //Cleaning is needed only if it is not erased. Now do the erase!
          {
            /*wait_tx_ends();*/
            //Erasing sector DATABASE_BASE_PAGEBER
            check_flash_locked();
            check_flash_error();
            flash_erase_page((uint32_t)(sect_num * FLASH_PAGE_SIZE));
            flash_wait_for_last_operation();
            check_flash_error();
          }
          //Now confirm cleaning
          if(check_page_erased((uint32_t)(sect_num*FLASH_PAGE_SIZE), &sector_erased, &attempts_erasing_page))
          {
            base_of_database = (uint32_t*)((uint32_t)(sect_num * FLASH_PAGE_SIZE));
            break;  // this break quits "while (attempts_erasing_page < MAX_ERASE_TRIES)"
          }
          con_send_string((uint8_t*)"\r\n");
        } //while (attempts_erasing_page < MAX_ERASE_TRIES) //3 tries
      } //for(uint16_t sect_num = DATABASE_BASE_PAGE; sect_num < (DATABASE_TOP_PAGE+1); sect_num++)
    } //if(ch == '&')
  } //if (!con_available_get_char())

  // if INITIAL_DATABASE is unprogrammed
  bool empty_database = true;
  base_of_database32 = (uint32_t *)INITIAL_DATABASE;
  for(iter = 0; iter < (DATABASE_SIZE / 4); iter++)
  {
    if(*(base_of_database32 + iter) != 0xFFFFFFFF)
    {
      empty_database = false;
      break;  //quit: for(iter = 0; iter < DATABASE_SIZE; iter += 4)
    }
  }

  //If it is unprogrammed, quit now.
  if (empty_database)
  {
    //Database is empty.
    base_of_database8 = (uint8_t*)&DEFAULT_MSX_KEYB_DATABASE_CONVERSION[0][0];
    base_of_database = (uint32_t*)base_of_database8;
    compatible_database = false;
    con_send_string((uint8_t*)"\r\nDatabase area on flash memory is erased.\r\n\r\n");
    return;
  } //if (empty_database)

  //Searching a valid (useful) Database. Unused databases are marked as UNUSED_DATABASE
  volatile uint32_t displacement = 0;
  base_of_database8 = (uint8_t*)((uint32_t)INITIAL_DATABASE - displacement);

  while(((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)DATABASE_BASE_ADD)
  {
    //Searching a valid (useful) Database. Unused databases are marked as UNUSED_DATABASE
    if(
    *(base_of_database8+0) == UNUSED_DATABASE[0] &&
    *(base_of_database8+1) == UNUSED_DATABASE[1] &&
    *(base_of_database8+2) == UNUSED_DATABASE[2] &&
    *(base_of_database8+3) == UNUSED_DATABASE[3] &&
    *(base_of_database8+4) == UNUSED_DATABASE[4] &&
    *(base_of_database8+5) == UNUSED_DATABASE[5] &&
    *(base_of_database8+6) == UNUSED_DATABASE[5] &&
    *(base_of_database8+7) == UNUSED_DATABASE[7] )
    {
      displacement += DATABASE_SIZE;
      base_of_database8 = (uint8_t*)((uint32_t)INITIAL_DATABASE - displacement);
    }
    else
      break;  //if it is here, it found a valid Database
  }

  //Check Database consistency (CheckSum & BCC of the first 319 blocks of DB_NUM_COLS bytes each)
  uint8_t CheckSum = 0, bcc = 0;  //bcc is a vertical parity
  for (iter = 0; iter < (DATABASE_SIZE - DB_NUM_COLS); iter ++)
  {
    CheckSum += *(base_of_database8 + iter);
    bcc ^= *(base_of_database8 + iter);
  } //for (iter = 0; iter < (DATABASE_SIZE - DB_NUM_COLS); iter ++)

  if( ((*(base_of_database8 + DATABASE_SIZE - 1)) != CheckSum)  ||
      ((*(base_of_database8 + DATABASE_SIZE - 2)) != bcc)       ||
       (*(base_of_database8 + 0)                  != 1)         ||
       (*(base_of_database8 + 1)                  != 0)         )
  {
    compatible_database = false;
    con_send_string((uint8_t*)"\r\n\n!!!Attention!!! => No valid Database found. Please update it!\r\n\n");
  }
  else
  {
    y_dummy         =  *(base_of_database8 + 3) & 0x0F; //Low nibble (no keys at this column)
    ps2numlockstate = (*(base_of_database8 + 3) & 0x10) != 0; //Bit 4
    enable_xon_xoff = (*(base_of_database8 + 3) & 0x20) != 0; //Bit 5
    update_ps2_leds = true;
    base_of_database = (uint32_t*)base_of_database8;
    compatible_database = true;
  }
  flash_lock();
  flash_locked = true;
}


int flash_rw(void)
{
  uint32_t result = 0;
  uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];
  uint8_t flash_buffer_ram[DATABASE_SIZE]; //Local variable, in aim to not consume resources in the main functional machine

  con_send_string((uint8_t*)"To update the Database, please send the new file in Intel Hex format!");
  con_send_string((uint8_t*)"\r\nOr turn off now.");
  get_intelhex_to_RAM(flash_buffer_ram, DATABASE_SIZE);
  
  /*wait_tx_ends();*/
  result = flash_program_data(flash_buffer_ram);

  switch(result)
  {
  case RESULT_OK: //everything ok
    break;
  case FLASH_WRONG_DATA_WRITTEN: //data read from Flash is different than written data
    break;

  default: //wrong flags' values in Flash Status Register (FLASH_SR)
    con_send_string((uint8_t*)"\r\nWrong value of FLASH_SR: ");
    conv_uint32_to_8a_hex(result, str_mount);
    con_send_string(&str_mount[0]);
    break;
  }
  //send end_of_line
  con_send_string((uint8_t*)"\r\n");
  return result;
} //int flashF4_rw(void)



bool check_page_erased(uint32_t page_add, bool *sector_erased, uint16_t *attempts_erasing_page)
{
  //Now confirm cleaning
  *sector_erased = true;
  for (uint32_t iter = 0; iter < (FLASH_PAGE_SIZE / sizeof(uint32_t)); iter++)  //All page bytes must be checked
  {
    if( (*(volatile uint32_t *)(page_add + iter)) != 0xFFFFFFFF )
    {
      (*attempts_erasing_page)++;
      *sector_erased = false;
      //Information to user - continued
      if(*attempts_erasing_page == 1)
        con_send_string((uint8_t*)"Not OK at first attempt");
      if(*attempts_erasing_page == 2)
        con_send_string((uint8_t*)"Not OK at second attempt");
      if(*attempts_erasing_page == MAX_ERASE_TRIES)
        con_send_string((uint8_t*)"Not OK at third attempt\r\n");
      break;  //quit "for (iter = 0; iter < (DATABASE_TOP_ADDR + 1 - DATABASE_BASE_ADD); iter += 4)"
    } //if( (*(uint32_t *)(DATABASE_BASE_ADD + iter)) != 0xFFFFFFFF )
  } //for (iter = 0; iter < (DATABASE_TOP_ADDR + 1 - DATABASE_BASE_ADD); iter+=4)
  if(*sector_erased)
  {
    //Information to user - continued
    con_send_string((uint8_t*)"   Done\r\n");
    //Flash Database zone cleared. Points to default (Initial) Database address
    //base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE);
  } //if(!attempts_erasing_page)
return *sector_erased;
}


void check_flash_locked(void)
{
  //Unlock FLASH_if it is locked
  if(flash_locked)
  {
    flash_unlock();
    flash_locked = false;
    flash_wait_for_last_operation();
  }
}


void check_flash_error(void)
{
  //Read FLASH_SR (Flash status register), searching for errors
  if(FLASH_SR & (1 << 14))  //RDERR (1 << 14): Read Protection Error (pcrop)
  {
    con_send_string((uint8_t*)"RDERR: Read Protection Error (pcrop)\r\n");
    FLASH_SR |= (1 << 14);  //Cleared by writing 1.
  }
  if(FLASH_SR & FLASH_SR_PGERR) //PGSERR: Programming sequence error
  {
    con_send_string((uint8_t*)"PGSERR: Programming sequence error\r\n");
    FLASH_SR |= FLASH_SR_PGERR; //Cleared by writing 1.
  }
  /*if(FLASH_SR & FLASH_SR_PGAERR)  //PGAERR: Programming alignment error
  {
    con_send_string((uint8_t*)"PGAERR: Programming alignment error\r\n");
    FLASH_SR |= FLASH_SR_PGAERR;  //Cleared by writing 1.
  }*/
  if(FLASH_SR & FLASH_SR_WRPRTERR)//WRPERR: Write protection error
  {
    con_send_string((uint8_t*)"FLASH_SR_WRPRTERR: Write protection error\r\n");
    FLASH_SR |= FLASH_SR_WRPRTERR;  //Cleared by writing 1.
  }
  /*if(FLASH_SR & FLASH_SR_OPERR) //OPERR: Operation error. This bit is set only if error interrupts are enabled (ERRIE = 1).
  {
    con_send_string((uint8_t*)"OPERR: Operation error\r\n");
    FLASH_SR |= FLASH_SR_OPERR; //Cleared by writing 1.
  }*/
  /*wait_tx_ends();*/
}



uint32_t flash_program_data(uint8_t *flash_buffer_ram)
{
  uint32_t iter;
  uint32_t displacement;
  bool DBaseSizeErasedPlaceFound = false, sector_erased = true;
  uint8_t str_mount[20];

  //verify:
  //1) If there is DATABASE_SIZE room in sector DATABASE_BASE_PAGE to acomodate a new Database image
  displacement = 0;
  base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE);
  //Information to user
  con_send_string((uint8_t*)"\r\n\nSearching for an empty ");
  void* void_ptr = &str_mount;
  conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
  con_send_string((uint8_t*)str_mount);
  con_send_string((uint8_t*)" bytes in sector 3 of flash memory...\r\nBase Address  Size  Status\r\n");
  while( (((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)DATABASE_BASE_ADD) && !DBaseSizeErasedPlaceFound)
  {
    //Information to user
    void_ptr = &str_mount;
    conv_uint32_to_8a_hex(((uint32_t)INITIAL_DATABASE - displacement), void_ptr);
    con_send_string((uint8_t*)" 0x");
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)"    ");
    conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
    con_send_string((uint8_t*)str_mount);

    DBaseSizeErasedPlaceFound = true;
    for (iter = 0; iter < (DATABASE_SIZE/sizeof(uint32_t)); iter++)
    {
      //Searching a DATABASE_SIZE room in the address range of flash sector FLASH_SECTOR3_NUMBER to acomodate a new Database image
      if( *(base_of_database + iter) != 0xFFFFFFFF )
      {
        con_send_string((uint8_t*)"  Not available\r\n");
        DBaseSizeErasedPlaceFound = false;
        displacement += DATABASE_SIZE;
        base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE - displacement);
        break;  // this break quits 'for (iter; iter < DATABASE_SIZE; iter+=4)'
      }
    } //for (iter; iter < DATABASE_SIZE; iter+=4)
    if(iter >= (DATABASE_SIZE/sizeof(uint32_t)) && DBaseSizeErasedPlaceFound)
    {
      //DATABASE_SIZE page is free on "base_of_database" address
      con_send_string((uint8_t*)"  Ok!\r\n");
    }
  } //while( (((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)DATABASE_BASE_ADD) && !DBaseSizeErasedPlaceFound)

  //2) If there is no room in pages 22 to 31, then erase them
  if (!DBaseSizeErasedPlaceFound)
  {
    //DATABASE_SIZE bytes free room was not found: Perform erase of page 22 to 31
    uint16_t attempts_erasing_page = 0;
    //Information to user
    con_send_string((uint8_t*)"\r\nErasing flash memory...\r\nBase Address  Size  Status\r\n");
    for(uint16_t sect_num = DATABASE_BASE_PAGE; sect_num < (DATABASE_TOP_PAGE+1); sect_num++)
    {
      //0x8005800 (Erase pages from 22 to 31)
      while (attempts_erasing_page < MAX_ERASE_TRIES) //3 tries
      {
        //Information to user
        void_ptr = &str_mount;
        conv_uint32_to_8a_hex(((uint32_t)(sect_num*FLASH_PAGE_SIZE)), void_ptr);
        con_send_string((uint8_t*)" 0x");
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)"   ");
        conv_uint32_to_dec((uint32_t)FLASH_PAGE_SIZE, void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)"  ");
        //Cleaning is needed only if it is not erased. First check
        sector_erased = true;
        uint8_t *page_fl = (uint8_t *)(sect_num * FLASH_PAGE_SIZE);
        for (iter = 0; iter < (FLASH_PAGE_SIZE); iter++)  //All page bytes must be checked
        {
          if( (*(volatile uint8_t *)(page_fl + iter)) != 0xFF )
          {
            sector_erased = false;
            break;  //quit "for (uint16_t iter = 0; iter < (FLASH_PAGE_SIZE / sizeof(uint32_t)); iter++)"
          } //if( (*(uint32_t *)(DATABASE_BASE_ADD + iter)) != 0xFFFFFFFF )
        } //for (iter = 0; iter < (DATABASE_TOP_ADDR + 1 - DATABASE_BASE_ADD); iter+=4)
        if (!sector_erased) //Cleaning is needed only if it is not erased. Now do
        {
          /*wait_tx_ends();*/
          //Erasing page
          check_flash_locked();
          check_flash_error();
          flash_erase_page((uint32_t)(sect_num * FLASH_PAGE_SIZE));
          flash_wait_for_last_operation();
          check_flash_error();
        }
        //Now confirm cleaning
        if(check_page_erased((uint32_t)(sect_num*FLASH_PAGE_SIZE), &sector_erased, &attempts_erasing_page))
        {
          base_of_database = (uint32_t*)((uint32_t)(sect_num * FLASH_PAGE_SIZE));
          break;  // this break quits "while (attempts_erasing_page < 3) //MAX_ERASE_TRIES tries"
        }
        con_send_string((uint8_t*)"\r\n");
      } //0x800C0000 (Sector 3) while (attempts_erasing_page < MAX_ERASE_TRIES) //3 tries
    } //for(uint16_t sect_num = 22; sect_num < 32; sect_num++)
  } //if (!DBaseSizeErasedPlaceFound)
  
  //Programming Flash Memory
  //Information to user
  con_send_string((uint8_t*)"\r\nProgramming flash memory...\r\nBase Address  Size  RAM Address\r\n");
  uintptr_t base_of_database_num = (uintptr_t)base_of_database; //Convert from pointer to integer
  void_ptr = &str_mount;
  conv_uint32_to_8a_hex(((uint32_t)base_of_database_num), void_ptr);
  con_send_string((uint8_t*)" 0x");
  con_send_string((uint8_t*)str_mount);
  con_send_string((uint8_t*)"  ");
  conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
  con_send_string((uint8_t*)str_mount);
  con_send_string((uint8_t*)"  0x");
  conv_uint32_to_8a_hex(((uintptr_t)flash_buffer_ram), void_ptr);
  con_send_string((uint8_t*)str_mount);
  /*serial_wait_tx_ends();*/
  check_flash_locked();
  check_flash_error();
  //programming flash memory
  //flash_program((uint32_t)base_of_database_num, flash_buffer_ram, DATABASE_SIZE);
  for(iter = 0; iter < DATABASE_SIZE; iter += 4)
  {
    //programming word data
    flash_program_word(base_of_database_num+iter, *((uint32_t*)(flash_buffer_ram + iter)));
    flash_wait_for_last_operation();
    uint32_t flash_status = flash_get_status_flags();
    if(flash_status != FLASH_SR_EOP)
      return flash_status;
  }
  check_flash_error();

  con_send_string((uint8_t*)"\r\n\nVerification of written data:\r\n");
  //verify if correct data was written
  for (iter = 0; iter < (DATABASE_SIZE); iter += 4) //All DATABASE_SIZE must be checked
  {
    uint32_t iter_div4 = iter / sizeof(uint32_t);// Pointer of uint32_t has a step of 4 bytes
    if( *(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
    {
      if(*(base_of_database + iter) == 0xFFFFFFFF)
      {
        con_send_string((uint8_t*)"(RAM buffer address = 0x");
        conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)". Was = 0x");
        conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)", while flash address 0x");
        conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)" remains erased.\r\nLocked!");
        /*wait_tx_ends();*/
      }
      else
      {
        con_send_string((uint8_t*)"Wrong data written into flash memory:\r\nDest add => 0x");
        void_ptr = &str_mount;
        conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)", found = 0x");
        conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)". Source add => 0x");
        conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)", was = 0x");
        conv_uint32_to_8a_hex(*((uint32_t*)(flash_buffer_ram + iter)), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)"\r\nLocked!");
      }
      return FLASH_WRONG_DATA_WRITTEN;
    } //if( *(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
  } //for (iter = 0; iter < DATABASE_SIZE; iter+=4) //3K must be checked

  //if flash sector DATABASE_BASE_PAGEBER was NOT just erased, there is no need to "invalidate" former Database,
  //as there is no former Database
  base_of_database_num = (uintptr_t)base_of_database; //Convert from pointer to integer
  void_ptr = &str_mount;

  if (base_of_database_num != INITIAL_DATABASE)
  {
    con_send_string((uint8_t*)"\r\n\nWriting and verifying former Database:\r\n");
    base_of_database += DATABASE_SIZE;  //points to former Database
    base_of_database_num = (uintptr_t)base_of_database; //Convert from pointer to integer
    con_send_string((uint8_t*)str_mount);
    //Make former Databse not useable anymore
    //First copy UNUSED_DATABASE to RAM
    uint16_t *base_of_database16 = (uint16_t*)&UNUSED_DATABASE;
    for(iter = 0; iter < (DB_NUM_COLS / 2); iter ++)
    {
      uint16_t ch16 = *(base_of_database16 + iter);
      conv_uint32_to_8a_hex((uintptr_t)(base_of_database_num + iter), void_ptr);
      con_send_string((uint8_t*)str_mount);
      /*wait_tx_ends();*/
      flash_program_half_word(base_of_database_num+iter, ch16);
      flash_wait_for_last_operation();
      uint32_t flash_status = flash_get_status_flags();
      if(flash_status != FLASH_SR_EOP)
        return flash_status;
      check_flash_error();
      //Now check written data
      if((*(uint16_t*)(base_of_database+iter)) == ch16)
      {
        con_send_string((uint8_t*)" <= Ok\r\n");
      }
      else  //if (*(uint16_t*)(base_of_database+iter) = ch16)
      {
        con_send_string((uint8_t*)" <= Wrong\r\n");
        flash_lock();
        flash_locked = true;
        return FLASH_WRONG_DATA_WRITTEN;
      } //if (*(uint16_t*)(base_of_database+iter) = ch16)
    } //for(iter = 0; iter < (DB_NUM_COLS / 2); iter ++)
  } //if (base_of_database_num != INITIAL_DATABASE)
  con_send_string((uint8_t*)"\r\nSuccessfully written database at 0x");
  con_send_string((uint8_t*)".\r\n\nNow, please TURN OFF to plug the PS/2 keyboard!");
  flash_lock();
  flash_locked = true;
  return RESULT_OK;
} //uint32_t flash_program_data(uint8_t *flash_buffer_ram)
#endif  //#if MCU == STM32F103



#if MCU == STM32F401

//Ref.: RM0368 Rev 5 manual, item 3.8.2, Flash key register (FLASH_KEYR) - Page 61/847..
//Some "stranger thing" occurred here in libopencm3, as it linked wrong definitions.
#define FPEC_KEY1                 ((uint32_t)0x45670123)
#define FPEC_KEY2                 ((uint32_t)0xCDEF89AB)


//Prototype area:
//flash operations
uint32_t flash_program_data(uint8_t*);
void check_flash_error(void);
void check_flash_locked(void);
bool check_sector3_erased(bool*, uint16_t*);
void cleanupFlash(bool*, uint16_t*);


//Global var area:
extern bool ps2_keyb_detected;                    //Declared on ps2handl.c
extern uint32_t systicks;                         //Declared on sys_timer.cpp


void database_setup(void)
{
  uint8_t           ch, str_mount[STRING_MOUNT_BUFFER_SIZE];
  uint32_t          iter;
  volatile uint32_t *base_of_database32;
  volatile uint8_t  *base_of_database8;
  void              *void_ptr;
  uint16_t          attempts_erasing_sector;
  bool              sector_erased = true;
  
  void_ptr = &str_mount;

  if (!ps2_keyb_detected) // The user request to force init Database is done only if there is no keyboard
  {
    //The user wants to reset Database to system's defaults
    //Firstly verify if some character was sent to USART or USER_KEY was pressed during BAT waiting
    if ( con_available_get_char() || (!gpio_get(USER_KEY_PORT, USER_KEY_PIN)) )
    {
      //Cleanup RX serial buffer
      while (con_available_get_char())
        ch = con_get_char();
      if (!gpio_get(USER_KEY_PORT, USER_KEY_PIN)) //USER_KEY is exclusive of WeAct board
      {
        con_send_string((uint8_t*)"\r\n\nOk. Now release user key...");
        while (!gpio_get(USER_KEY_PORT, USER_KEY_PIN))  //But stay here until the button is released
          __asm("NOP");
      }
      con_send_string((uint8_t*)"\r\nReset Database to factory default. Press ""&"" to proceed or any other key to abort\r\n");
      //Wait for user action
      uint32_t lastsysticks = systicks;
      bool print_message = true;
      while (!con_available_get_char())
      {
        if( ((systicks - lastsysticks) % FREQ_INT_SYSTICK) == 0 )
        {
          ch = (MAX_TIMEOUT2AMPERSAND - (systicks - lastsysticks)) / FREQ_INT_SYSTICK;
          if(print_message && ch < (MAX_TIMEOUT2AMPERSAND / FREQ_INT_SYSTICK))
          {
            con_send_string((uint8_t*)"Timeout to answer: ");
            conv_uint32_to_dec((uint32_t)ch, str_mount);
            con_send_string(str_mount);
            con_send_string((uint8_t*)"s \r");
            print_message = false;
          }
        }
        else
        {
          print_message = true;
        }
        //Check timeout
        if( (systicks - lastsysticks) > MAX_TIMEOUT2AMPERSAND )
        {
          //User messages
          con_send_string((uint8_t*)"\r\n\nTimeout to answer: Proceeding without Reset the Database.\r\n");
          //put a " " into console input (con_rx_ring if uart_rx_ring) to answer "no" to the next question
          insert_in_con_rx(' ');
        }
      }
      ch = con_get_char();
      if(ch == '&')
        cleanupFlash(&sector_erased, &attempts_erasing_sector);
    } //if ( con_available_get_char() || (!gpio_get(USER_KEY_PORT, USER_KEY_PIN)) )
  } //if (!ps2_keyb_detected) // The user request to force init Database is done only if there is no keyboard

  // if (INITIAL_DATABASE is unprogrammed)
  bool empty_database = true;
  base_of_database32 = (uint32_t *)INITIAL_DATABASE;
  for(iter = 0; iter < (DATABASE_SIZE / sizeof(uint32_t)); iter++)
  {
    if(*(base_of_database32 + iter) != 0xFFFFFFFF)
    {
      empty_database = false;
      break;  //quit: for(iter = 0; iter < DATABASE_SIZE; iter += 4)
    }
  }

  if (empty_database)
  {
    //Database is empty. Use DEFAULT_MSX_KEYB_DATABASE_CONVERSION
    base_of_database8 = (uint8_t*)&DEFAULT_MSX_KEYB_DATABASE_CONVERSION[0][0];
    con_send_string((uint8_t*)"..  Database OK at 0x");
    conv_uint32_to_8a_hex((uintptr_t)(base_of_database8), void_ptr);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)". Reading system parameters...\r\n");
    y_dummy         =  *(base_of_database8 + 3) & 0x0F; //Low nibble (no keys at this column)
    ps2numlockstate = (*(base_of_database8 + 3) & 0x10) != 0; //Bit 4
    enable_xon_xoff = (*(base_of_database8 + 3) & 0x20) != 0; //Bit 5
    update_ps2_leds = true;
    base_of_database = (uint32_t*)base_of_database8;
    compatible_database = true;
    return;
  } //if (empty_database)

  //Searching a valid (useful) Database. Unused databases are marked as UNUSED_DATABASE
  volatile uint32_t displacement = 0;
  base_of_database8 = (uint8_t*)((uint32_t)INITIAL_DATABASE - displacement);

  while(((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)FLASH_SECTOR3_BASE)
  {
    //  Searching a valid (useful) Database. Unused databases are marked as UNUSED_DATABASE
    if(
    *(base_of_database8+0) == UNUSED_DATABASE[0] &&
    *(base_of_database8+1) == UNUSED_DATABASE[1] &&
    *(base_of_database8+2) == UNUSED_DATABASE[2] &&
    *(base_of_database8+3) == UNUSED_DATABASE[3] &&
    *(base_of_database8+4) == UNUSED_DATABASE[4] &&
    *(base_of_database8+5) == UNUSED_DATABASE[5] &&
    *(base_of_database8+6) == UNUSED_DATABASE[6] &&
    *(base_of_database8+7) == UNUSED_DATABASE[7] )
    {
      displacement += DATABASE_SIZE;
      base_of_database8 = (uint8_t*)((uint32_t)INITIAL_DATABASE - displacement);
    }
    else
    {
      break;  //if it is here, it found a valid Database
    }
  }

  //Check Database consistency (CheckSum & BCC of the first 319 blocks of 8 bytes each)
  uint8_t checksum = 0, bcc = 0;  //bcc is a vertical parity
  for (iter = 0; iter < (DATABASE_SIZE - DB_NUM_COLS); iter ++)
  {
    checksum += *(base_of_database8 + iter);
    bcc ^= *(base_of_database8 + iter);
  } //for (iter = 0; iter < (DATABASE_SIZE - DB_NUM_COLS); iter ++)
  if( ((*(base_of_database8 + (DATABASE_SIZE - 1)) + checksum) != 0)  ||
      ((*(base_of_database8 + (DATABASE_SIZE - 2)))            != bcc)||
       (*(base_of_database8 + 0)                               != 1)  ||
       (*(base_of_database8 + 1)                               != 0)  )
  {
    void_ptr = &str_mount;
    //Display bcc
    /*serial_wait_tx_ends();*/
    con_send_string((uint8_t*)"\r\nError on Database at Base address 0x");
    conv_uint32_to_8a_hex((uintptr_t)(base_of_database8 + 0), void_ptr);
    con_send_string((uint8_t*)str_mount);
    /*serial_wait_tx_ends();*/
    con_send_string((uint8_t*)"\r\n\nBad data at address: 0x");
    conv_uint32_to_8a_hex((uintptr_t)(base_of_database8 + (DATABASE_SIZE - 2)), void_ptr);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)": computed BCC = 0x");
    conv_uint8_to_2a_hex(bcc, void_ptr);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)", but found 0x");
    conv_uint8_to_2a_hex(*(base_of_database8 + (DATABASE_SIZE - 2)), void_ptr);
    con_send_string((uint8_t*)str_mount);
    //Display checksum
    con_send_string((uint8_t*)"\r\nBad data at address: 0x");
    conv_uint32_to_8a_hex((uintptr_t)(base_of_database8 + (DATABASE_SIZE - 1)), void_ptr);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)": computed CheckSum = 0x");
    conv_uint8_to_2a_hex(checksum, void_ptr);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)", but found 0x");
    conv_uint8_to_2a_hex(*(base_of_database8 + (DATABASE_SIZE - 1)), void_ptr);
    con_send_string((uint8_t*)str_mount);
    //Display version
    con_send_string((uint8_t*)"\r\nDatabase version ");
    conv_uint32_to_dec((uint32_t)(*(base_of_database8 + 0)), void_ptr);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)".");
    conv_uint32_to_dec((uint32_t)(*(base_of_database8 + 1)), void_ptr);
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)"\r\n\n..  !!!Attention!!! => No new valid Database found. Using the factory default one.");
    base_of_database8 = (uint8_t*)&DEFAULT_MSX_KEYB_DATABASE_CONVERSION[0][0];
    con_send_string((uint8_t*)"\r\n\n..  !!!If you want to use a different mapping, please update the Database!!!\r\n\n");
  }
  con_send_string((uint8_t*)"..  Database OK at 0x");
  conv_uint32_to_8a_hex((uintptr_t)(base_of_database8), void_ptr);
  con_send_string((uint8_t*)str_mount);
  con_send_string((uint8_t*)". Reading system parameters...\r\n");
  y_dummy         =  *(base_of_database8 + 3) & 0x0F; //Low nibble (no keys at this column)
  ps2numlockstate = (*(base_of_database8 + 3) & 0x10) != 0; //Bit 4
  enable_xon_xoff = (*(base_of_database8 + 3) & 0x20) != 0; //Bit 5
  update_ps2_leds = true;
  base_of_database = (uint32_t*)base_of_database8;
  compatible_database = true;
  flash_lock();
}


void cleanupFlash(bool *sector_erased, uint16_t *attempts_erasing_sector)
{
  uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];

  //0x800C0000 (Sector 3)
  *attempts_erasing_sector = 0;
  while (*attempts_erasing_sector < MAX_ERASE_TRIES) //3 tries
  {
    //Information to user
    con_send_string((uint8_t*)"\r\nErasing flash memory...\r\nBase Address  Size  Status\r\n 0x");
    conv_uint32_to_8a_hex(((uint32_t)FLASH_SECTOR3_BASE), (uint8_t*)&(str_mount));
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)"  ");
    conv_uint32_to_dec((uint32_t)(FLASH_SECTOR3_TOP - FLASH_SECTOR3_BASE + 1), (uint8_t*)&(str_mount));
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)" ");
    /*serial_wait_tx_ends();*/
    //Erase sector FLASH_SECTOR3_NUMBER only if not erased
    for (uint32_t iter = 0; iter < (FLASH_SECTOR3_TOP + 1 - FLASH_SECTOR3_BASE); iter+=4) //All sector bytes must be checked
    {
      if( (*(uint32_t *)(FLASH_SECTOR3_BASE + iter)) != 0xFFFFFFFF )
      {
        //FLASH_SECTOR3_NUMBER is not erased
        check_flash_locked();
        flash_erase_sector(FLASH_SECTOR3_NUMBER, FLASH_CR_PROGRAM_X8); //program_size 0 (8-bit), 1 (16-bit), 2 (32-bit), 3 (64-bit)
        check_flash_error();
        break;  //quit: for(iter = 0; iter < DATABASE_SIZE; iter += 4)
      }
    }
    //Now confirm cleaning
    if(check_sector3_erased(sector_erased, attempts_erasing_sector))
      break;  // this break quits "while (attempts_erasing_sector < 3) //MAX_ERASE_TRIES tries"
    con_send_string((uint8_t*)"\r\n");
  } //0x800C0000 (Sector 3) while (*attempts_erasing_sector < MAX_ERASE_TRIES)
} //if (!gpio_get(USER_KEY_PORT, USER_KEY_PIN))


bool check_sector3_erased(bool *sect_erased, uint16_t *attempts_erasing_sector)
{
  //Now confirm cleaning
  *sect_erased = true;
  for (uint32_t iter = 0; iter < (FLASH_SECTOR3_TOP + 1 - FLASH_SECTOR3_BASE); iter+=4) //All sector bytes must be checked
  {
    if( (*(volatile uint32_t *)(FLASH_SECTOR3_BASE + iter)) != 0xFFFFFFFF )
    {
      (*attempts_erasing_sector)++;
      *sect_erased = false;
      //Information to user - continued
      if(*attempts_erasing_sector == 1)
        con_send_string((uint8_t*)"Not OK at first attempt");
      if(*attempts_erasing_sector == 2)
        con_send_string((uint8_t*)"Not OK at second attempt");
      if(*attempts_erasing_sector == MAX_ERASE_TRIES)
        con_send_string((uint8_t*)"Not OK at third attempt\r\n");
      break;  //quit "for (iter = 0; iter < (FLASH_SECTOR3_TOP + 1 - FLASH_SECTOR3_BASE); iter += 4)"
    } //if( (*(uint32_t *)(FLASH_SECTOR3_BASE + iter)) != 0xFFFFFFFF )
  } //for (iter = 0; iter < (FLASH_SECTOR3_TOP + 1 - FLASH_SECTOR3_BASE); iter+=4)
  if(*sect_erased)
  {
    //Information to user - continued
    con_send_string((uint8_t*)" Successful\r\n");
    //Flash Database zone cleared. Points to default (Initial) Database address
    base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE);
  } //if(!attempts_erasing_sector)
return *sect_erased;
}


void check_flash_locked(void)
{
  //Unlock FLASH_if it is locked
  while(FLASH_SR & FLASH_SR_BSY)  //FLASH_CR is read only when FLASH_SR_BSY is set
    __asm("nop");                 //To avoid compiler optimizations
  if(FLASH_CR & FLASH_CR_LOCK)
  {
    //flash unlock => used to allow access to the Flash control register and so,
    //to allow program and erase operations.
    FLASH_KEYR = FPEC_KEY1;
    FLASH_KEYR = FPEC_KEY2;
  }
}


void check_flash_error(void)
{
  //Read FLASH_SR (Flash status register), searching for errors
  if(FLASH_SR & (1 << 14))  //RDERR (1 << 14): Read Protection Error (pcrop)
  {
    con_send_string((uint8_t*)"RDERR: Read Protection Error (pcrop)\r\n");
    FLASH_SR |= (1 << 14);  //Cleared by writing 1.
  }
  if(FLASH_SR & FLASH_SR_PGSERR)  //PGSERR: Programming sequence error
  {
    con_send_string((uint8_t*)"PGSERR: Programming sequence error\r\n");
    FLASH_SR |= FLASH_SR_PGSERR;  //Cleared by writing 1.
  }
  if(FLASH_SR & FLASH_SR_PGAERR)  //PGAERR: Programming alignment error
  {
    con_send_string((uint8_t*)"PGAERR: Programming alignment error\r\n");
    FLASH_SR |= FLASH_SR_PGAERR;  //Cleared by writing 1.
  }
  if(FLASH_SR & FLASH_SR_WRPERR)//WRPERR: Write protection error
  {
    con_send_string((uint8_t*)"WRPERR: Write protection error\r\n");
    FLASH_SR |= FLASH_SR_WRPERR;  //Cleared by writing 1.
  }
  if(FLASH_SR & FLASH_SR_OPERR) //OPERR: Operation error. This bit is set only if error interrupts are enabled (ERRIE = 1).
  {
    con_send_string((uint8_t*)"OPERR: Operation error\r\n");
    FLASH_SR |= FLASH_SR_OPERR; //Cleared by writing 1.
  }
  /*serial_wait_tx_ends();*/
}



uint32_t flash_program_data(uint8_t *flash_buffer_ram)  //Local flash_buffer_ram declarated at flashF4_rw
{
  uint32_t iter;
  uint32_t displacement;
  bool DBaseSizeErasedPlaceFound = false, sector_erased = true;
  uint8_t str_mount[20];

  //verify:
  //1) If there is enough DATABASE_SIZE room in sector FLASH_SECTOR3_NUMBER to acomodate a new Database image
  displacement = 0;
  base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE);
  //Information to user
  con_send_string((uint8_t*)"\r\n\nSearching for an empty ");
  void *void_ptr = &str_mount;
  conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
  con_send_string((uint8_t*)str_mount);
  //con_send_string((uint8_t*)" bytes in sector 3 of flash memory...\r\nBase Address  Size  Status\r\n");
  con_send_string((uint8_t*)" bytes in sector ");
  conv_uint32_to_dec((uint32_t)FLASH_SECTOR3_NUMBER, void_ptr);
  con_send_string((uint8_t*)str_mount);
  con_send_string((uint8_t*)" of flash memory...\r\nBase Address  Size  Status\r\n");
  while( (((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)FLASH_SECTOR3_BASE) && !DBaseSizeErasedPlaceFound)
  {
    //Information to user
    conv_uint32_to_8a_hex(((uint32_t)INITIAL_DATABASE - displacement), void_ptr);
    con_send_string((uint8_t*)" 0x");
    con_send_string((uint8_t*)str_mount);
    con_send_string((uint8_t*)"   ");
    conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
    con_send_string((uint8_t*)str_mount);

    DBaseSizeErasedPlaceFound = true;
    for (iter = 0; iter < (DATABASE_SIZE/sizeof(uint32_t)); iter++)
    {
      //Searching for a DATABASE_SIZE room in the address range of flash sector FLASH_SECTOR3_NUMBER to acomodate a new Database image
      if( *(base_of_database + iter) != 0xFFFFFFFF )
      {
        con_send_string((uint8_t*)"  Not available\r\n");
        DBaseSizeErasedPlaceFound = false;
        displacement += DATABASE_SIZE;
        base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE - displacement);
        break;  // this break quits 'for (iter; iter < DATABASE_SIZE; iter+=4)'
      }
    } //for (iter; iter < DATABASE_SIZE; iter+=4)
    if(iter >= (DATABASE_SIZE/sizeof(uint32_t)) && DBaseSizeErasedPlaceFound)
    {
      //DATABASE_SIZE page is free on "base_of_database" address
      con_send_string((uint8_t*)"  Ok!\r\n");
    }
  } //while( (((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)FLASH_SECTOR3_BASE) && !DBaseSizeErasedPlaceFound)

  //2) If there is no room in sector FLASH_SECTOR3_NUMBER, then erase this sector
  if (!DBaseSizeErasedPlaceFound)
  {
    //3K free room was not found: Perform erase of sector FLASH_SECTOR3_NUMBER
    uint16_t attempts_erasing_sector = 0;
    cleanupFlash(&sector_erased, &attempts_erasing_sector);
    base_of_database = (uint32_t*)INITIAL_DATABASE;
  } //if (!DBaseSizeErasedPlaceFound)
  
  //Programming Flash Memory
  //Information to user
  con_send_string((uint8_t*)"\r\nProgramming flash memory...\r\nDest address  Size  Origin RAM address\r\n");
  uintptr_t base_of_database_num = (uintptr_t)base_of_database; //Convert from pointer to integer
  void_ptr = &str_mount;
  conv_uint32_to_8a_hex(((uint32_t)base_of_database_num), void_ptr);
  con_send_string((uint8_t*)" 0x");
  con_send_string((uint8_t*)str_mount);
  con_send_string((uint8_t*)"   ");
  conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
  con_send_string((uint8_t*)str_mount);
  con_send_string((uint8_t*)"  0x");
  conv_uint32_to_8a_hex(((uintptr_t)flash_buffer_ram), void_ptr);
  con_send_string((uint8_t*)str_mount);
  /*serial_wait_tx_ends();*/
  check_flash_locked();
  flash_program((uint32_t)base_of_database_num, flash_buffer_ram, DATABASE_SIZE);
  check_flash_error();

  con_send_string((uint8_t*)"\r\n\nVerification of written data:");
  //verify if correct data was written
  for (iter = 0; iter < (DATABASE_SIZE); iter += 4) //All DATABASE_SIZE must be checked
  {
    uint32_t iter_div4 = iter / sizeof(uint32_t);// Pointer of uint32_t has a step of 4 bytes
    if( *(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
    {
      if(*(base_of_database + iter) == 0xFFFFFFFF)
      {
        con_send_string((uint8_t*)"\r\n(RAM buffer address = 0x");
        conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)". Was = 0x");
        conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)", while flash address 0x");
        conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)" remains erased.\r\nLocked!");
        /*serial_wait_tx_ends();*/
      }
      else
      {
        con_send_string((uint8_t*)"\r\nWrong data written into flash memory:\r\n");
        con_send_string((uint8_t*)"Dest add => 0x");
        void_ptr = &str_mount;
        conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)", found = 0x");
        conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)". Source add => 0x");
        conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)", was = 0x");
        conv_uint32_to_8a_hex(*((uint32_t*)(flash_buffer_ram + iter)), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)"\r\nLocked!");
      }
      return FLASH_WRONG_DATA_WRITTEN;
    } //if( *(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
  } //for (iter = 0; iter < DATABASE_SIZE; iter+=4) //3K must be checked
  con_send_string((uint8_t*)" Succesfully done!");

  //if flash sector FLASH_SECTOR3_NUMBER was just erased, there is no need to "invalidate" former Database,
  //as there is no former Database
  base_of_database_num = (uintptr_t)base_of_database; //Convert from pointer to integer
  if (base_of_database_num != INITIAL_DATABASE)
  {
    con_send_string((uint8_t*)"\r\n\nNulling former Database...\r\n");

    base_of_database += (DATABASE_SIZE /sizeof(uint32_t));  //points to former Database
    base_of_database_num = (uintptr_t)base_of_database;     //Convert from pointer to integer

    //Make former Database not useable anymore
    for(iter = 0; iter < sizeof(UNUSED_DATABASE); iter ++)
    {
      uint8_t ch = UNUSED_DATABASE[iter];
      /*serial_wait_tx_ends();*/
      check_flash_locked();
      flash_program_byte(base_of_database_num + iter, ch);
      flash_wait_for_last_operation();
      check_flash_error();
      //Now check written data
      if((*(uint8_t*)(base_of_database_num+iter)) == ch)
      {
        //con_send_string((uint8_t*)" <= Ok (\r\n");
        /*con_send_string((uint8_t*)" <= Ok (0x");
        conv_uint8_to_2a_hex(ch, void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)")\r\n");*/
      }
      else  //if (*(uint8_t*)(base_of_database_num+iter) = ch)
      {
        //Information to user
        con_send_string((uint8_t*)"Address: 0x");
        conv_uint32_to_8a_hex((uintptr_t)(base_of_database_num + iter), void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)" <= Wrong (0x");
        conv_uint8_to_2a_hex((*(uint8_t*)(base_of_database_num+iter)), void_ptr);
        conv_uint8_to_2a_hex(ch, void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)" instead of 0x");
        conv_uint8_to_2a_hex(ch, void_ptr);
        con_send_string((uint8_t*)str_mount);
        con_send_string((uint8_t*)")\r\n");
        flash_lock();
        return FLASH_WRONG_DATA_WRITTEN;
      } //if (*(uint8_t*)(base_of_database+iter) = ch)
    } //for(iter = 0; iter < sizeof(UNUSED_DATABASE); iter ++)
  } //if (base_of_database_num != INITIAL_DATABASE)
  else
  {
    con_send_string((uint8_t*)"\r\n\nNo need to null formmer Database, as it is the first one.\r\n");
  } //else //if (base_of_database_num != INITIAL_DATABASE)
  con_send_string((uint8_t*)"\r\nSuccessful!\r\n\nNow, please TURN OFF to plug the PS/2 keyboard!");
  flash_lock();
  return RESULT_OK;
} //uint32_t flash_program_data(uint32_t *flash_buffer_ram)


int flash_rw(void)  //was main. It is int to allow simulate as a single module
{
  uint32_t result = 0;
  uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];
  uint8_t flash_buffer_ram[DATABASE_SIZE]; //Local variable, in aim to not consume resources in the main functional machine

  con_send_string((uint8_t*)"Ready to update the Database! To do so now, please\r\n");
  con_send_string((uint8_t*)"send the new Database file in Intel Hex format!");
  con_send_string((uint8_t*)"\r\n\nOr turn off now...\r\n");
  get_intelhex_to_RAM(flash_buffer_ram, DATABASE_SIZE);
  
  /*serial_wait_tx_ends();*/
  result = flash_program_data(flash_buffer_ram);
  
  switch(result)
  {
  case RESULT_OK: //everything ok
    break;
  case FLASH_WRONG_DATA_WRITTEN: //data read from Flash is different than written data
    break;

  default: //wrong flags' values in Flash Status Register (FLASH_SR)
    con_send_string((uint8_t*)"\r\nWrong value of FLASH_SR: ");
    conv_uint32_to_8a_hex(result, str_mount);
    con_send_string(&str_mount[0]);
    break;
  }
  //send end_of_line
  con_send_string((uint8_t*)"\r\n");
  return result;
} //int flashF4_rw(void)  //was main. It is int to allow simulate as a single module

#endif  //#if MCU == STM32F401
