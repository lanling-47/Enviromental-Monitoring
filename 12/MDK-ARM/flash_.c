
// @author     古月杰
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
#include "lorawan_node_driver.h"
#include "usart.h"
#include "dma.h"
#include "rtc.h"
#include "gpio.h"
#include "tim.h"
#include "i2c.h"
#include "hdc1000.h"
#include "opt3001.h"
#include "MPL3115.h"
#include "mma8451.h"
#include "XPT2046.h"
#include "flash_.h"
#define FLASH_SAVE_ADDR   (0x08080000)
#define FLASH_SAVE_ADDR1  (0x08090000)
void Flash_Read(uint32_t address, uint32_t *data, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++) {
        data[i] = *(__IO uint32_t *)(address + (i * 8)); // 以字为单位读取Flash
			//debug_printf("read_data = %d\r\n",data[i]);
    }
}

// 掉电存储函数
void WriteFlash(uint32_t addr,uint32_t *Data,uint32_t L)
{
	uint32_t i=0;
	/* 1/4解锁FLASH*/
	HAL_FLASH_Unlock();
	/* 2/4擦除FLASH*/
	/*初始化FLASH_EraseInitTypeDef*/
	/*擦除方式页擦除FLASH_TYPEERASE_PAGES，块擦除FLASH_TYPEERASE_MASSERASE*/
	/*擦除页数*/
	/*擦除地址*/
	FLASH_EraseInitTypeDef FlashSet;
	FlashSet.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashSet.Page = addr;
	FlashSet.NbPages = 2;
	/*设置PageError，调用擦除函数*/
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&FlashSet, &PageError);
	/* 3/4对FLASH烧写*/
	for(i=0;i<L;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr+8*i, Data[i]);
		//debug_printf("write_data = %d\r\n",Data[i]);
	}
	/* 4/4锁住FLASH*/
	HAL_FLASH_Lock();
}

void my_flash_write_uint32_t(void)
{
	// 将数据转换为uint32_t数组以便写入Flash
	uint32_t flash_data[sizeof(int_data) / sizeof(uint32_t)];
	memcpy(flash_data, &int_data, sizeof(int_data));
	
	// 将数据写入Flash
	WriteFlash(FLASH_SAVE_ADDR, flash_data, sizeof(int_data) / sizeof(uint32_t));
}
float fff;//这是用来测试的
void my_flash_read_uint32_t(void)
{
	// 将读取到的数据转换回原始结构体
	// 从Flash中读取数据
	uint32_t flash_data_read[sizeof(int_data) / sizeof(uint32_t)];
	Flash_Read(FLASH_SAVE_ADDR, flash_data_read, sizeof(int_data) / sizeof(uint32_t));
	memcpy(&int_data, flash_data_read, sizeof(int_data));
	// 现在，data_read.float_data 和 data_read.int_data 中分别包含从Flash中读取的float和int数据
	//读取完以后的数据可以从这里拿出！！！！
	//fff=data_read.float_data[0];
	//debug_printf("store data = %d %d\n",int_data[0],int_data[1]);
}


