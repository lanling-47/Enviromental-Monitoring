#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
/**
  * @brief  Initializes Memory.
  * @param  None
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
flash_status_t Flash_If_Init(void)
{
  /* Unlock the internal flash */
  HAL_FLASH_Unlock();
  return FLASH_OK;
}

/**
  * @brief  De-Initializes Memory.
  * @param  None
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
flash_status_t Flash_If_DeInit(void)
{
  /* Lock the internal flash */
  HAL_FLASH_Lock();
  return FLASH_OK;
}

/**
  * @brief  Erases sector.
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
flash_status_t Flash_If_Erase(uint32_t Add)
{
  uint32_t PageError = 0;
  /* Variable contains Flash operation status */
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef eraseinitstruct;

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

   /* Get the number of sector to erase from 1st sector*/
  eraseinitstruct.Banks = GetBank(Add);
  eraseinitstruct.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseinitstruct.Page = GetPage(Add);
  eraseinitstruct.NbPages = 1;
  status = HAL_FLASHEx_Erase(&eraseinitstruct, &PageError);

  if (status != HAL_OK)
  {
    return FLASH_ERR;
  }
  return FLASH_OK;
}

/**
  * @brief  Writes Data into Memory.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
flash_status_t Flash_If_Write(uint8_t *src, uint32_t dest_addr, uint32_t Len)
{
  uint32_t i = 0;

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  for(i = 0; i < Len; i += 8)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by byte */
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)(dest_addr+i), *(uint64_t*)(src+i)) == HAL_OK)
    {
     /* Check the written value */
      if(*(uint64_t *)(src + i) != *(uint64_t*)(dest_addr+i))
      {
        /* Flash content doesn't match SRAM content */

        return FLASH_CHECK_ERR;
      }

    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return FLASH_ERR;
    }
  }
  return FLASH_OK;
}

/**
  * @brief  Reads Data into Memory.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval return FLASH_OK.
  */
flash_status_t Flash_If_Read(uint8_t* buff, uint32_t dest_addr, uint32_t Len)
{

    uint32_t i;
    for(i = 0; i < Len; i++){
            buff[i] = *(__IO uint8_t*)(dest_addr + i);
    }
  /* Return a valid address to avoid HardFault */
  return FLASH_OK;
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}