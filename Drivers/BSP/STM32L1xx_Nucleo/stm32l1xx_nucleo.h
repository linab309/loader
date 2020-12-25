/**
  ******************************************************************************
  * @file    stm32l1xx_nucleo.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    04-March-2016
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32L1XX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8 TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L1XX_NUCLEO_H
#define __STM32L1XX_NUCLEO_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32L1XX_NUCLEO
  * @{
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

   
/** @defgroup STM32L1XX_NUCLEO_Exported_Types Exported Types
  * @{
  */
typedef enum 
{
  LED_GREEN = 0,
  LED_RED ,
  LED_SD ,
  LED_GPS ,
  LED_BULE,
  LED_SURPORT,
  LED_GPS_2,

  LED_MORE = LED_SURPORT
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER   = 0,
  BUTTON_WAKEUP = 1,

  /* Alias */
  BUTTON_KEY  = BUTTON_WAKEUP
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 



typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;


/**
  * @}
  */ 

/** @defgroup STM32L1XX_NUCLEO_Exported_Constants Exported Constants
  * @{
  */ 

/** 
  * @brief  Define for STM32L1xx_NUCLEO board  
  */ 
#if !defined (USE_STM32L1xx_NUCLEO)
 #define USE_STM32L1xx_NUCLEO
#endif
  
/** @defgroup STM32L1XX_NUCLEO_LED LED Constants
  * @{
  */
#define LEDn                             7

#define LED_GREEN_PIN                         GPIO_PIN_0
#define LED_GREEN_GPIO_PORT                   GPIOB
#define LED_GREEN_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_GREEN_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()  


#define LED_RED_PIN                         GPIO_PIN_1
#define LED_RED_GPIO_PORT                   GPIOB
#define LED_RED_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_RED_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()  

#define LED_SD_PIN                         GPIO_PIN_3
#define LED_SD_GPIO_PORT                   GPIOB
#define LED_SD_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_SD_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()  

#define LED_GPS_PIN                         GPIO_PIN_4
#define LED_GPS_GPIO_PORT                   GPIOB
#define LED_GPS_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_GPS_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()  


#define LED_BULE_PIN                         GPIO_PIN_5
#define LED_BULE_GPIO_PORT                   GPIOB
#define LED_BULE_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_BULE_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()  

#define LED_SURPORT_PIN                         GPIO_PIN_6
#define LED_SURPORT_GPIO_PORT                   GPIOB
#define LED_SURPORT_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_SURPORT_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()  

#define LED_GPS_2_PIN                           GPIO_PIN_8
#define LED_GPS_2_PORT                          GPIOB
#define LED_GPS_2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_GPS_2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()  






#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) LED_GREEN_GPIO_CLK_ENABLE();} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED_GREEN_GPIO_CLK_DISABLE() : 0)


/**
  * @}
  */ 

/** @defgroup STM32L1XX_NUCLEO_BUTTON BUTTON Constants
  * @{
  */  
#define BUTTONn                          2  

/**
  * @brief User push-button
 */
#define USER_BUTTON_PIN                  GPIO_PIN_15
#define USER_BUTTON_GPIO_PORT            GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn            EXTI15_10_IRQn
/* Aliases */
#define KEY_BUTTON_PIN                   USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT             USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()     USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()    USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn             USER_BUTTON_EXTI_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == 0) USER_BUTTON_GPIO_CLK_ENABLE();} while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0)


/**
  * @brief User push-button
 */
#define WAKEUP_BUTTON_PIN                  GPIO_PIN_13
#define WAKEUP_BUTTON_GPIO_PORT            GPIOC
#define WAKEUP_BUTTON_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define WAKEUP_BUTTON_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOC_CLK_DISABLE()
#define WAKEUP_BUTTON_EXTI_IRQn            EXTI15_10_IRQn
/* Aliases */
#define WAKEUP_KEY_BUTTON_PIN                   WAKEUP_BUTTON_PIN
#define WAKEUP_KEY_BUTTON_GPIO_PORT             WAKEUP_BUTTON_GPIO_PORT
#define WAKEUP_KEY_BUTTON_GPIO_CLK_ENABLE()     WAKEUP_BUTTON_GPIO_CLK_ENABLE()
#define WAKEUP_KEY_BUTTON_GPIO_CLK_DISABLE()    WAKEUP_BUTTON_GPIO_CLK_DISABLE()
#define WAKEUP_KEY_BUTTON_EXTI_IRQn             WAKEUP_BUTTON_EXTI_IRQn

#define WAKEUP_BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == 1) WAKEUP_BUTTON_GPIO_CLK_ENABLE();} while(0)
#define WAKEUP_BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 1) ? WAKEUP_BUTTON_GPIO_CLK_DISABLE() : 0)

/**
  * @}
  */



/**
  * @}
  */
    
/** @addtogroup STM32L1XX_NUCLEO_BUS BUS Constants
  * @{
  */
/*###################### SPI1 ###################################*/
#define NUCLEO_SPIx                                 SPI1
#define NUCLEO_SPIx_CLK_ENABLE()                    __HAL_RCC_SPI1_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                          GPIO_AF5_SPI1
#define NUCLEO_SPIx_SCK_GPIO_PORT                   GPIOA
#define NUCLEO_SPIx_SCK_PIN                         GPIO_PIN_5
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                    GPIO_AF5_SPI1
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT             GPIOA
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                        GPIO_PIN_6
#define NUCLEO_SPIx_MOSI_PIN                        GPIO_PIN_7
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define NUCLEO_SPIx_TIMEOUT_MAX                   1000





/**
  * @brief  SD Control Interface pins (shield D4)
  */
#define SD_CS_PIN                                 GPIO_PIN_3
#define SD_CS_GPIO_PORT                           GPIOA
#define SD_CS_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOA_CLK_DISABLE()

/**
  * @brief  SD Control Lines management
  */  
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)     



#define BATT_CHARG_OK 0x05
#define BATT_EMPTY 0x00
#define BATT_HIGH 0x03
#define BATT_LOW 0x01
#define BATT_MID 0x02

#define BATT_CHARGING 0x04




/**
  * @}
  */

/**
  * @}
  */
    
/** @addtogroup STM32L1XX_NUCLEO_Exported_Functions
  * @{
  */
uint32_t        BSP_GetVersion(void);
/** @addtogroup STM32L1XX_NUCLEO_LED_Functions
  * @{
  */ 

void            BSP_LED_Init(Led_TypeDef Led);
void            BSP_LED_DeInit(Led_TypeDef Led);
void            BSP_LED_On(Led_TypeDef Led);
void            BSP_LED_Off(Led_TypeDef Led);
void            BSP_LED_Toggle(Led_TypeDef Led);



/**
  * @}
  */

/** @addtogroup STM32L1XX_NUCLEO_BUTTON_Functions
  * @{
  */

void            BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void            BSP_PB_DeInit(Button_TypeDef Button);
uint32_t        BSP_PB_GetState(Button_TypeDef Button);


/**
  * @}
  */




/**
  * @}
  */ 

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __STM32L1XX_NUCLEO_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
