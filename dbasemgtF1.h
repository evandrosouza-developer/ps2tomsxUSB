#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if MCU == STM32F103C8
#define NUM_DATABASE_IMG 			3

//Address of Base of flash page, used to put various Databases without need of erase each time
//update process is done.
#define FLASH_BASE_ADD				0x08000000
#define INITIAL_DATABASE			0x08007100	//Place to put the initial (compilation time) database
#define DATABASE_BASE_ADD			0x08005800	//This address marks the beggining of page 22
#define DATABASE_BASE_PAGE		22
#define DATABASE_TOP_ADDR			0x08007FFF	//STM32F103C6T6 (32K Flash 10K RAM)
#define DATABASE_TOP_PAGE			31
#define FLASH_PAGE_SIZE 			0x400				//1K in STM32F103C6T6 and STM32F103C8T6

/*entry point*/
int flashF1_rw(void);
void database_setup(void);

#endif	//#if MCU == STM32F103C8

#ifdef __cplusplus
}
#endif
