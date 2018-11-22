/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USER_Path[4];  /* USER logical drive path */

/* USER CODE BEGIN Variables */
#include "stm32l1xx_nucleo.h"


FATFS SD_FatFs;
uint8_t  My_Fs_Init(FATFS *SD_FatFs);

/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USER_Path);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  if(My_Fs_Init(&SD_FatFs) == 1)
  {

  }  
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes Memory.
  * @param  None
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
uint16_t Flash_If_Init(void)
{ 
  /* Unlock the internal flash */  
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);  
  return 0;
}

/**
  * @brief  De-Initializes Memory.
  * @param  None
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
uint16_t Flash_If_DeInit(void)
{ 
  /* Lock the internal flash */
  HAL_FLASH_Lock();
  return 0;
}

/**
  * @brief  Erases sector.
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
uint16_t Flash_If_Erase(void)
{
  uint32_t NbOfPages = 0;
  uint32_t PageError = 0;
  /* Variable contains Flash operation status */
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef eraseinitstruct;

   /* Get the number of sector to erase from 1st sector*/
  NbOfPages = (USBD_DFU_APP_END_ADD - USBD_DFU_APP_DEFAULT_ADD) / FLASH_PAGE_SIZE;
  eraseinitstruct.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseinitstruct.PageAddress = USBD_DFU_APP_DEFAULT_ADD;
  eraseinitstruct.NbPages = NbOfPages;
  status = HAL_FLASHEx_Erase(&eraseinitstruct, &PageError);

  if (status != HAL_OK)
  {
    return 1;
  }
  return 0;
}

/**
  * @brief  Writes Data into Memory.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval 0 if operation is successeful, MAL_FAIL else.
  */
uint16_t Flash_If_Write(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  uint32_t i = 0;
  
  for(i = 0; i < Len; i+=4)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by byte */ 
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dest+i), *(uint32_t*)(src+i)) == HAL_OK)
    {
     /* Check the written value */
      if(*(uint32_t *)(src + i) != *(uint32_t*)(dest+i))
      {
        /* Flash content doesn't match SRAM content */
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dest+i), *(uint32_t*)(src+i)) == HAL_OK)
        {
            if(*(uint32_t *)(src + i) != *(uint32_t*)(dest+i))
               return 2;                
        }
        

      }
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return 1;
    }
  }
  return 0;
}

/**
  * @brief  Reads Data into Memory.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the physical address where data should be read.
  */
uint8_t *Flash_If_Read(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  uint32_t i = 0;
  uint8_t *psrc = src;
  
  for(i = 0; i < Len; i++)
  {
    dest[i] = *psrc++;
  }
  /* Return a valid address to avoid HardFault */
  return (uint8_t*)(dest); 
}




uint8_t  My_Fs_Init(FATFS *SD_FatFs)
{

	uint8_t ret = 0;



	/* Check the mounted device */
    if(retUSER == 0)
    {
    	if(f_mount(SD_FatFs, (TCHAR const*)USER_Path, 0) != FR_OK)
    	{
    	    //printf("BSP_SD_INIT_FAILED \r\n");
    	}  
    	else
    	{
    	    //printf("\r\n mkdir_init \r\n");  

        }
    }
    else
    {
        FATFS_UnLinkDriver(USER_Path);
    }

    return ret;
  
}

#define FLASH_PAGE_TO_BE_PROTECTED (OB_WRP1_PAGES32TO47 | OB_WRP1_PAGES48TO63)  

uint8_t update_frameware(void)
{
    FRESULT fr;
    FIL update_config_fp;
    UINT br = 0;
    uint32_t SectorAddress = USBD_DFU_APP_DEFAULT_ADD;
    uint8_t update_filebuffer[512];
    //uint32_t *ramsource = NULL;
    uint32_t status = 0;
    uint8_t led_flag = 0;
    uint8_t rewrite_cnt = 0;

    /*Variable used to handle the Options Bytes*/
   // static FLASH_OBProgramInitTypeDef OptionsBytesStruct;
    //printf("f_open  P-1.BIN  \r\n");

    if(f_open(&update_config_fp,(TCHAR const*)"P-1.BIN",FA_READ) == FR_OK)
    {
        sound_toggle_simple(3,50,50);   
        Flash_If_Init();
        /* Unlock the Options Bytes *************************************************/

        
        //HAL_FLASH_OB_Unlock();  
#if 0        
        /* Get pages write protection status ****************************************/
        HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
        
        
        if((OptionsBytesStruct.WRPSector0To31 & FLASH_PAGE_TO_BE_PROTECTED) == FLASH_PAGE_TO_BE_PROTECTED)
        {
          /* Restore write protected pages */
          OptionsBytesStruct.OptionType   = OPTIONBYTE_WRP;
          OptionsBytesStruct.WRPState     = OB_WRPSTATE_DISABLE;
          OptionsBytesStruct.WRPSector0To31 = FLASH_PAGE_TO_BE_PROTECTED;
          if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
          {
              ;
          }
        
          /* Generate System Reset to load the new option byte values ***************/
          HAL_FLASH_OB_Launch();
        }
        /* Lock the Options Bytes *************************************************/
        HAL_FLASH_OB_Lock();    
#endif        
        //if ((OptionsBytesStruct.WRPSector0To31 & FLASH_PAGE_TO_BE_PROTECTED) == 0x00)
        {        
            Flash_If_Erase();
        }
        
        while(1)        
        {

            if(led_flag == 0)
            {
                BSP_LED_On(LED_SD);  
                led_flag++;
            }
            else if(led_flag == 1)
            {
                BSP_LED_On(LED_GPS);  
                led_flag++;
            }

            else if(led_flag == 2)            
            {
                BSP_LED_On(LED_SURPORT);  
                led_flag++;
            }

            else if(led_flag == 3)
            {
                BSP_LED_Off(LED_SD);
                BSP_LED_Off(LED_GPS);
                BSP_LED_Off(LED_SURPORT);
                led_flag = 0;
            }

            fr = f_read(&update_config_fp,update_filebuffer,512,&br);
            //ramsource = (uint32_t*)update_filebuffer;
            if(br != 0)
            {
                status = Flash_If_Write(update_filebuffer,(uint8_t *)SectorAddress,br);
                //printf("file status :%d, %d \r\n",status,br);  
                while(status != 0)
                {
                    status = Flash_If_Write(update_filebuffer,(uint8_t *)SectorAddress,br);
                    rewrite_cnt++;
                    if(rewrite_cnt > 200)
                    {
                        rewrite_cnt = 0;
                        break;
                            
                    }
                }
                
                if(br < 512)
                {
                   //printf("file end  \r\n");  
                   f_close(&update_config_fp);
                   //printf("file end  \r\n");                 
                   Flash_If_DeInit();
                   break;

                }
                SectorAddress += 512;
                
            }
            else
            {
               f_close(&update_config_fp);
               Flash_If_DeInit();
               break;
            }
        }

        return 0;
    }
    else
        return 1;

}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
