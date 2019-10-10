/**
******************************************************************************
* @file    hal.c
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Hardware Abstraction Layer
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
* Initial BlueNRG-Mesh is built over Motorola’s Mesh over Bluetooth Low Energy 
* (MoBLE) technology. The present solution is developed and maintained for both 
* Mesh library and Applications solely by STMicroelectronics.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_common.h"
#include "mesh_cfg.h"
#include "appli_mesh.h"
#include "PWM_handlers.h"
#include "PWM_config.h"
#include "BlueNRG1_gpio.h"
#ifdef STEVAL_BLUEMIC1
#include "inertial_app.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  SetLed sets the state of led
* @param  int state
* @retval void
*/ 
void SetLed(int state)
{
  if (state != 0)
  {
#ifdef CUSTOM_BOARD_PWM_SELECTION
    SdkEvalLedOn(LED3);
#elif defined(STEVAL_BLUEMIC_1_BOARD_PWM_SELECTION)
    Modify_PWM(PWM1, PWM_TIME_PERIOD);
#else
    Modify_PWM(PWM4, PWM_TIME_PERIOD);
#endif    
    
  }
  else
  {
#ifdef CUSTOM_BOARD_PWM_SELECTION
    SdkEvalLedOff(LED3);
#elif defined(STEVAL_BLUEMIC_1_BOARD_PWM_SELECTION)
    Modify_PWM(PWM1, 1);
#else
    Modify_PWM(PWM4, 1);
#endif    
  }
}

/**
* @brief  GetButtonState 
* @param  void
* @retval BUTTON_STATE returns the state of the button
*/ 
BUTTON_STATE GetButton2State(void)
{
#ifdef STEVAL_BLUEMIC1
   return SdkEvalPushButtonGetState(BUTTON_1);
#else 
  return SdkEvalPushButtonGetState(BUTTON_2);
#endif
}

/**
* @brief  Change State of Yellow led
* @param  none
* @retval void
*/ 
void StateYellowLed(MOBLEUINT8 const *data)
{
  if(*(data+1) == 0x00)
    SdkEvalLedOff(LED1);
  else if(*(data+1) == 0xFF)
    SdkEvalLedOn(LED1);
}

/**
* @brief  Change State of Blue led
* @param  none
* @retval void
*/ 
void StateBlueLed(MOBLEUINT8 const *data)
{
  if(*(data+2) == 0x00)
    SdkEvalLedOff(LED3);
  else if(*(data+2) == 0xFF)
    SdkEvalLedOn(LED3);
}

/**
* @brief  Change Status of Red led
* @param  none
* @retval void
*/ 
void StateRedLed(MOBLEUINT8 const *data)
{
  if(*data == 0x00)
    SdkEvalLedOff(LED2);
  else if(*data == 0xFF)
    SdkEvalLedOn(LED2);
}

/**
* @brief  GetButtonState 
* @param  void
* @retval BUTTON_STATE returns the state of the button
*/ 
BUTTON_STATE GetButtonState(void)
{
  return SdkEvalPushButtonGetState(BUTTON_1);
}

/**
* @brief  InitButton Initializes the Button
* @param  void
* @retval void
*/ 
static void InitButton(void)
{
  SdkEvalPushButtonInit(BUTTON_1);
}

/**
* @brief  InitDevice Initializes the device
* @param  void
* @retval void
*/
void InitDevice(void)
{
  SystemInit();/* Device Initialization */
  SdkEvalIdentification();
  NVIC->ICPR[0] = 0xFFFFFFFF;/* Clear pending interrupt on cortex */
#ifdef ENABLE_USART /* ENABLE_USART */
  SdkEvalComIOConfig(SdkEvalComIOProcessInputData);
#endif /* ENABLE_USART */
  Clock_Init();
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_PKA, ENABLE);
  
  SdkEvalLedInit(LED1);
  SdkEvalLedInit(LED2); 
/* when RGB board is selcted then blue LEd is used as GPIO. for STEVAL board blue
   Led is used as PWM.
*/  
#ifdef CUSTOM_BOARD_PWM_SELECTION
  SdkEvalLedInit(LED3);
#endif 
  
  /* Configures Button pin as input */
  InitButton();
  SdkEvalPushButtonIrq(BUTTON_1, IRQ_ON_BOTH_EDGE);
    
  /* Enables the BUTTON Clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
  
  /* Enable the GPIO as input interupt */
#ifdef SAVE_MODEL_STATE_POWER_FAILURE_DETECTION       
  GPIO_InitNVICPowerOff();
#endif 
   
#ifdef ENABLE_OCCUPANCY_SENSOR

  TRACE_I(TF_INIT,"Ensure GPIO_0 connected with sensor \r\n");
  GPIO_InitSensorPin();
#endif
  
  /* Clear GPIO pending interrupt on SWD_CLK */
  GPIO_ClearITPendingBit(SWD_CLK_PIN);
#ifdef STEVAL_BLUEMIC1   
  INERTIAL_APP_Init();

  INERTIAL_double_tap(1); 
#endif 
  SystemSleepCmd(ENABLE);
  
}

/**
* @brief function to initialise the GPIO interrupt for Power down 
* @param  void
* @retval void
*/
void GPIO_InitNVICPowerOff(void)
{
  GPIO_EXTIConfigType GPIO_ExtIntType;
  GPIO_InitType GPIO_InitConfig;
  NVIC_InitType NVIC_InitConfig;

   GPIO_ExtIntType.GPIO_Pin      = GPIO_Pin_12;
   GPIO_ExtIntType.GPIO_IrqSense = GPIO_IrqSense_Edge;
   GPIO_ExtIntType.GPIO_Event    = GPIO_Event_Low;
   
   GPIO_InitConfig.GPIO_Pin     = GPIO_Pin_12;
   GPIO_InitConfig.GPIO_Mode    = GPIO_Input;
   GPIO_InitConfig.GPIO_Pull    = DISABLE;
   GPIO_InitConfig.GPIO_HighPwr = DISABLE;
   
   NVIC_InitConfig.NVIC_IRQChannel = GPIO_IRQn;
   NVIC_InitConfig.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
   NVIC_InitConfig.NVIC_IRQChannelCmd = ENABLE;
   
   GPIO_Init(&GPIO_InitConfig);
   NVIC_Init(&NVIC_InitConfig);
   GPIO_EXTIConfig(&GPIO_ExtIntType); 
   GPIO_EXTICmd(GPIO_Pin_12 , ENABLE);
}

/**
* @brief function to inintiallise the GPIO interrupt fot sensor interrupt.
* @param  void
* @retval void
*/
void GPIO_InitSensorPin(void)
{
  GPIO_EXTIConfigType GPIO_ExtIntType;
  GPIO_InitType GPIO_InitConfig;
  NVIC_InitType NVIC_InitConfig;

   GPIO_ExtIntType.GPIO_Pin      = GPIO_Pin_0;
   GPIO_ExtIntType.GPIO_IrqSense = GPIO_IrqSense_Edge;
   GPIO_ExtIntType.GPIO_Event    = GPIO_Event_High;
   
   GPIO_InitConfig.GPIO_Pin     = GPIO_Pin_0;
   GPIO_InitConfig.GPIO_Mode    = GPIO_Input;
   GPIO_InitConfig.GPIO_Pull    = ENABLE;
   GPIO_InitConfig.GPIO_HighPwr = DISABLE;
   
   NVIC_InitConfig.NVIC_IRQChannel = GPIO_IRQn;
   NVIC_InitConfig.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
   NVIC_InitConfig.NVIC_IRQChannelCmd = ENABLE;
   
   GPIO_Init(&GPIO_InitConfig);
   NVIC_Init(&NVIC_InitConfig);
   GPIO_EXTIConfig(&GPIO_ExtIntType); 
   GPIO_EXTICmd(GPIO_Pin_0 , ENABLE);
}

/**
* @brief  ShouldSleepFunc sleep mode fuction   
* @param  void
* @retval void
*/
void ShouldSleepFunc(void)
{
  __WFI();
}
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
