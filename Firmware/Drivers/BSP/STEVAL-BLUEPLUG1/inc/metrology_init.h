/**
  ******************************************************************************
  * @file    metrology_init.h 
  * @author  System Lab Team
  * @version V1.0.0
  * @date    August-2017
  * @brief   User definded function definitions
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

#ifndef __USER_FUNC_H
#define __USER_FUNC_H

#include "BlueNRG1.h"
#include "BlueNRG1_conf.h"
#include "BLUEPLUG1_Config.h"
#include "stpm_metrology.h"
#include "metrology.h"
#include "metrology_hal.h"
#include "handler_metrology.h"
#include "metro_Task.h"
#include "meter.h"
#include "metering.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "app_state.h"
#include "chat.h"
#include "app_state.h"
#include "BLUEPLUG1_Config.h"
#include <stdio.h>
#include "ble_func.h"
#include "BlueNRG1_mft.h"
#include "BlueNRG1_rtc.h"
#include "BlueNRG1_uart.h"


#define RTC_PERIOD_500ms        (0x8000 - 1)    //0x8000 = System Freq/2
#define ONE_SEC                 8000
#define BLUE_LED_PIN            GPIO_Pin_14
//#define SYSCLK_FREQ             16000000      /*System clock frequency = 16MHz*/
#define PACKET_TYPE_3           3
#define PACKET_TYPE_4           4
#define ZCR_PIN                 GPIO_Pin_6              
#define STPM_SCS_PIN            GPIO_Pin_1
#define BUTTON_PIN              GPIO_Pin_12
#define STPM_SYN_PIN            GPIO_Pin_2
#define OPTO_PWM_PIN            GPIO_Pin_0
#define POWER_PWM_LED_PIN       GPIO_Pin_3

void GPIO_Initialization(void);
void MFT_Configuration(void);
void RTC_Timer_Configuration(void);
int printf(const char *_Restrict, ...);
void Metrology_Init(void);

#endif

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
