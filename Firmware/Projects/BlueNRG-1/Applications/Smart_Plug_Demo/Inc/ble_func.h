/**
  ******************************************************************************
  * @file    ble_func.h 
  * @author  System Lab Team
* @version V1.11.000
* @date    25-07-2019
  * @brief   User definded BLE function declarations and defines
  *
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/*Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ble_func_H
#define __ble_func_H

#ifdef __cplusplus
extern "C" {
#endif
  
#include "ble_func.h"
#include "meter.h"
#include "string.h"
  
  typedef struct
  {
    uint8_t Hours;   
    uint8_t Minutes;
  }Time_t;
  
  typedef struct
  {
    Time_t On_T  ;
    Time_t Off_T ;
    uint8_t Repeat_week;  
  }Schedule;
  
#ifdef __cplusplus
}
#endif

/* Private function prototypes -----------------------------------------------*/
void BLE_Profile(void);
void Send_PacketToApp(uint8_t packet_type);
void pushTo_Bufferarray(uint32_t data,uint8_t *data_enc,uint8_t offset);
void sendData(uint8_t *data,uint8_t length);

#endif
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
