#ifndef __FLASH__H_
#define __FLASH__H_
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
void my_flash_write_uint32_t(void);
void my_flash_read_uint32_t(void);
extern uint32_t int_data[2];
extern float float_data[4];



#endif