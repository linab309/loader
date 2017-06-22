/**
  ******************************************************************************
  * @file    FLASH/Data_Program/main.c 
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    13-April-2012
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"



/** @addtogroup STM32L1xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup FLASH_Data_Program
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x080803FF
#define DATA_EEPROM_PAGE_SIZE      0x8
#define DATA_32                    0x12345678
#define FAST_DATA_32               0x55667799

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef FLASHStatus = HAL_OK;
__IO TestStatus DataMemoryProgramStatus = PASSED;
uint32_t Address = 0;
  
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */


void stm_write_eerpom(uint16_t address,uint32_t data)
{
    HAL_FLASHEx_DATAEEPROM_Unlock();
    /* Clear all pending flags */      
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR );
    Address = DATA_EEPROM_START_ADDR;
    Address += (4*address);    
    while(HAL_OK != HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, Address,data))
    {
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
               | FLASH_FLAG_SIZERR );
    }
    Address = DATA_EEPROM_START_ADDR;
    //HAL_FLASHEx_DATAEEPROM_Lock();	
}


void stm_fast_write_eerpom(uint8_t address,uint32_t data)
{
   
    HAL_FLASHEx_DATAEEPROM_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR );
    Address += (4*address);    
    while(HAL_OK != HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_FASTWORD, Address,data))
    {
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		               | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
    }

    HAL_FLASHEx_DATAEEPROM_Lock();	

	
}
void stm_read_eerpom(uint32_t address,uint32_t *data)
{
    Address = DATA_EEPROM_START_ADDR;
    Address += (4*address);
    *data = *(__IO uint32_t*)Address;
}

uint8_t stm_EraseWord_Eeprom(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */

  uint32_t NbrOfPage = 0, j = 0;
  
  HAL_FLASHEx_DATAEEPROM_Unlock();

  /*  Data EEPROM Fast Word program of FAST_DATA_32 at addresses defined by 
      DATA_EEPROM_START_ADDR and DATA_EEPROM_END_ADDR */  
  Address = DATA_EEPROM_START_ADDR;

  NbrOfPage = ((DATA_EEPROM_END_ADDR - Address) + 1 ) >> 2; 
  
  /* Erase the Data EEPROM Memory pages by Word (32-bit) */
  for(j = 0; j < NbrOfPage; j++)
  {
    FLASHStatus = HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEERASEDATA_WORD,Address + (4 * j));
  }
   
  /* Check the correctness of written data */
  while(Address < DATA_EEPROM_END_ADDR)
  {
    if(*(__IO uint32_t*)Address != 0x0)
    {
      DataMemoryProgramStatus = FAILED;
    }
    Address = Address + 4;
  }
   
   Address = DATA_EEPROM_START_ADDR;
  HAL_FLASHEx_DATAEEPROM_Lock();
  return DataMemoryProgramStatus;
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

