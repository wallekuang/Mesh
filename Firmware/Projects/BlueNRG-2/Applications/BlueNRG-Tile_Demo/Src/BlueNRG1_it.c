/**
******************************************************************************
* @file    BlueNRG1_it.c
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
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
#include "BlueNRG1_it.h"
#include "bluenrg1_stack.h"
#include "clock.h"
#include "appli_mesh.h"
#include "PWM_handlers.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#if USER_DEFINED_PLATFORM == USER_EVAL_PLATFORM
  #include "LSM6DSO.h"
#else
  #include "lsm6ds3_hal.h"
#endif
#include <stdio.h>
#include "BlueNRG_x_device.h"
#include "crash_handler.h"
#include "miscutil.h"
#include "mesh_cfg.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#if USER_DEFINED_PLATFORM == USER_EVAL_PLATFORM
volatile uint8_t AXEL_Event_Detected = 0;
#endif
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief  This function handles NMI exception.
*/
void NMI_Handler(void)
{
  HAL_CrashHandler(__get_MSP(), NMI_SIGNATURE);
  TRACE_M(TF_HANDLER,"%s\r\n", __func__);
}

#if !defined(__IAR_SYSTEMS_ICC__)
void hardfaultGetContext(unsigned long* stackedContextPtr) __asm("label_hardfaultGetContext");
#endif

/*!
*  \fn void hardfaultGetContext(unsigned long* stackedContextPtr)
*  \brief Copies system stacked context into function local variables. \n
*  This function is called from asm-coded Interrupt Service Routine associated to 
*  HARD_FAULT exception
*  \param stackedContextPtr : Address of stack containing stacked processor context.
*/
void hardfaultGetContext(unsigned long* stackedContextPtr)
{
  volatile unsigned long stacked_r0;
  volatile unsigned long stacked_r1;
  volatile unsigned long stacked_r2;
  volatile unsigned long stacked_r3;
  volatile unsigned long stacked_r12;
  volatile unsigned long stacked_lr;
  volatile unsigned long stacked_pc;
  volatile unsigned long stacked_psr;
  volatile unsigned long _CFSR;
  volatile unsigned long _HFSR;
  volatile unsigned long _DFSR;
  volatile unsigned long _AFSR;
  volatile unsigned long _BFAR;
  volatile unsigned long _MMAR;
  
  TRACE_M(TF_HANDLER,"HardFault %p\r\n", stackedContextPtr);
  
  
  if (stackedContextPtr)
  {
    
    stacked_r0  = stackedContextPtr[0];
    stacked_r1  = stackedContextPtr[1];
    stacked_r2  = stackedContextPtr[2];
    stacked_r3  = stackedContextPtr[3];
    stacked_r12 = stackedContextPtr[4];
    stacked_lr  = stackedContextPtr[5];
    stacked_pc  = stackedContextPtr[6];
    stacked_psr = stackedContextPtr[7];
    
    /* Configurable Fault Status Register */
    
    /* Consists of MMSR, BFSR and UFSR */
    _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;
    
    /* Hard Fault Status Register */
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
    
    /* Debug Fault Status Register */
    _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
    
    /* Auxiliary Fault Status Register */
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
    
    /* Read the Fault Address Registers. These may not contain valid values.
       Check BFARVALID/MMARVALID to see if they are valid values
       MemManage Fault Address Register */
    _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    /* Bus Fault Address Register */
    _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;
    /* printf("HardFault: SP %p PC %lx r0 %lx r1 %lx r2 %lx r3 %lx\r\n", 
    stackedContextPtr, stacked_pc, stacked_r0, stacked_r1, stacked_r2, stacked_r3); */
    TRACE_M(TF_HANDLER,"HardFault");       
    
    /* The following code avoids compiler warning [-Wunused-but-set-variable]*/
    stackedContextPtr[0] = stacked_r0;
    stackedContextPtr[1] = stacked_r1;
    stackedContextPtr[2] = stacked_r2;
    stackedContextPtr[3] = stacked_r3;
    stackedContextPtr[4] = stacked_r12;
    stackedContextPtr[5] = stacked_lr;
    stackedContextPtr[6] = stacked_pc;
    stackedContextPtr[7] = stacked_psr;
    (*((volatile unsigned long *)(0xE000ED28))) = _CFSR;
    (*((volatile unsigned long *)(0xE000ED2C))) = _HFSR;
    (*((volatile unsigned long *)(0xE000ED30))) = _DFSR;
    (*((volatile unsigned long *)(0xE000ED3C))) = _AFSR;
    (*((volatile unsigned long *)(0xE000ED34))) = _MMAR;
    (*((volatile unsigned long *)(0xE000ED38))) = _BFAR;
  }
  while (1)
  {}
}

/**
* @brief HARD_FAULT interrupt service routine.
* @param  None
* @retval None
*/
#if defined(__GNUC__)
void __attribute__((naked, interrupt)) HardFault_Handler(void)
{
  __asm__ volatile  (
                     "     MOVS   R0, #4             \n" /* Determine if processor uses PSP or MSP by checking bit.4 at LR register. */
                       "   MOV    R1, LR             \n"
                         "   TST    R0, R1             \n"
                           "   BEQ    _IS_MSP              \n" /* Jump to '_MSP' if processor uses MSP stack.                  */
                             "   MRS    R0, PSP              \n" /* Prepare PSP content as parameter to the calling function below.  */
                               "   BL     label_hardfaultGetContext    \n" /* Call 'hardfaultGetContext' passing PSP content as stackedContextPtr value. */
                                 "_IS_MSP:                   \n"
                                   "   MRS    R0, MSP              \n" /* Prepare MSP content as parameter to the calling function below.   */
                                     "   BL     label_hardfaultGetContext    \n" /* Call 'hardfaultGetContext' passing MSP content as stackedContextPtr value. */
                                       ::  );
}
#else
void HardFault_Handler(void)
{
  HAL_CrashHandler(__get_MSP(), HARD_FAULT_SIGNATURE);
  TRACE_M(TF_HANDLER,"HardFault\r\n");
  
  while(1) {}
}

#endif

/**
* @brief This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  TRACE_M(TF_HANDLER,"%s\r\n", __func__);
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/

void BusFault_Handler(void)
{
  TRACE_M(TF_HANDLER,"%s\r\n", __func__);
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  TRACE_M(TF_HANDLER,"%s\r\n", __func__);
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}
/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
  TRACE_M(TF_HANDLER,"%s\r\n", __func__);
  while (1)
  {}
}
/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
  TRACE_M(TF_HANDLER,"%s\r\n", __func__);
  while (1)
  {}
}

/**
* @brief  This function handles PendSV_Handler exception.
* @param  None
* @retval None
*/

void PendSV_Handler(void)
{
  while (1)
  {}
}
/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
  SysCount_Handler();
}
/**
* @brief  This function handles GPIO.
* @param  None
* @retval None
*/
void GPIO_Handler(void)
{
  if(SdkEvalPushButtonGetITPendingBit(BUTTON_1) == SET) {
    
    /* Clear BUTTON_1 pending interrupt */
    SdkEvalPushButtonClearITPendingBit(BUTTON_1);
  }
  
  if(GPIO_GetITPendingBit(SWD_CLK_PIN) == SET) {
    
    /* Clear GPIO pending interrupt */
    GPIO_ClearITPendingBit(SWD_CLK_PIN);
  }
  
#if USER_DEFINED_PLATFORM == USER_EVAL_PLATFORM
  /* Check if GPIO pin 13 interrupt event occured */
  // Interrupt from the common line
  if (GPIO_GetITPendingBit(SDK_EVAL_IRQ_SENSOR_PIN) == SET) {


#ifdef ENABLE_SENSOR_MODEL_SERVER  //for sensor testing    
      GPIO_ClearITPendingBit(SDK_EVAL_IRQ_SENSOR_PIN);
      GPIO_EXTICmd(SDK_EVAL_IRQ_SENSOR_PIN, DISABLE);
      AXEL_Event_Detected = 1;
//      MEMSCallback();
//      GPIO_EXTICmd(SDK_EVAL_IRQ_SENSOR_PIN, ENABLE);
#endif
  }
#endif
}

#if 0
/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void UART_Handler(void)
{
}
#endif

/**
* @brief  This function handles BLE request.
* @param  None
* @retval None
*/
void Blue_Handler(void)
{
  RAL_Isr();
}
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

/**
* @brief  This function handles MFT1B interrupt request.
* @param  None
* @retval None
*/
void MFT1B_Handler(void)
{
  PWM2_handler();
}

/**
* @brief  This function handles MFT2B interrupt request.
* @param  None
* @retval None
*/
void MFT2B_Handler(void)
{
  PWM3_PWM4_handler();
}

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/


