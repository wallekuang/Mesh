/**
******************************************************************************
* @file    user_func.c 
* @author  System Lab
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

/* Includes ------------------------------------------------------------------*/

#include "metrology_init.h"

/* Private variables ---------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/**
* @brief  Meirology Initialization
* @param  None
* @retval None
*/
void Metrology_Init(void)
{
  METRO_Init();
  Meter_Init();
}

/**
* @brief  GPIO initialization
* @param  void
* @retval None
*/
void GPIO_Initialization(void){
  
  GPIO_InitType GPIO_InitStructure;
  
  /* Enables the GPIO Clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
  
  GPIO_EXTIConfigType GPIO_EXTIStructure;
//  NVIC_InitType NVIC_InitStructure;
//  
//  /* Set the GPIO interrupt priority and enable it */
//  NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
  
  /* Configures EXTI line */
  GPIO_EXTIStructure.GPIO_Pin = ZCR_PIN;
  GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
  GPIO_EXTIStructure.GPIO_Event = IRQ_ON_BOTH_EDGE;
  GPIO_EXTIConfig(&GPIO_EXTIStructure);
  
  /* Clear pending interrupt */
  GPIO_ClearITPendingBit(ZCR_PIN);
  
  /* Enable the interrupt */
  GPIO_EXTICmd(ZCR_PIN, ENABLE);
  
//  /* Initialize LED2 pin */
//  GPIO_InitStructure.GPIO_Pin = BLUE_LED_PIN ;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
//  GPIO_InitStructure.GPIO_Pull = DISABLE;
//  GPIO_InitStructure.GPIO_HighPwr = ENABLE; 
//  GPIO_Init(&GPIO_InitStructure);
  
  /* Initalize STPM chip select pin */
  GPIO_InitStructure.GPIO_Pin = STPM_SCS_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  UART_RXTimeoutConfig(0x007000);
  
//  /* Initalize Interrupt button pin */
//  GPIO_EXTIStructure.GPIO_Pin = BUTTON_PIN;
//  GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
//  GPIO_EXTIStructure.GPIO_Event = IRQ_ON_RISING_EDGE;
//  GPIO_EXTIConfig(&GPIO_EXTIStructure);
//  
//  GPIO_ClearITPendingBit(BUTTON_PIN);
//  
//  /* Enable the interrupt */
//  GPIO_EXTICmd(BUTTON_PIN, ENABLE);
  
  /* Initialize STPM synchronization pin */
  /*for defected modules assign GPIO_pin_3*/
  GPIO_InitStructure.GPIO_Pin = STPM_SYN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init(&GPIO_InitStructure);
  
  /* Initialize Optocoupler PWM pin*/
  GPIO_InitStructure.GPIO_Pin = OPTO_PWM_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init(&GPIO_InitStructure);
  
//  /* Initialize LED1 pin */
//  /*for defected modules assign GPIO_pin_2*/
//  GPIO_InitStructure.GPIO_Pin = POWER_PWM_LED_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
//  GPIO_Init(&GPIO_InitStructure);
}

/**
* @brief  MFT interrupt configuration
* @param  void
* @retval None
*/
void MFT_Configuration(void)
{
  MFT_InitType timer_init;
  NVIC_InitType NVIC_InitStructure;
  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_MTFX1, ENABLE);
  
  /* Init MFT3 in mode 3 */
  /* Clock 1 is from prescaled clock */
  /* Clock 2 is external clock */
  /* Counter 1 is preset and reloaded to 5000 (timer period = TnCRA * 2 = 10 ms) */
  /* Counter 2 is preset and reloaded to 5 (timer period = TnCRA * 2 * (5 * 2) = 100 ms */
  timer_init.MFT_Mode = MFT_MODE_3;
  timer_init.MFT_Clock1 = MFT_PRESCALED_CLK;
  timer_init.MFT_Prescaler = 8 - 1;
  timer_init.MFT_Clock2 = MFT_EXTERNAL_EVENT;
  timer_init.MFT_CRA = 1200 - 1;
  timer_init.MFT_CRB = 5 - 1;
  MFT_Init(MFT1, &timer_init);
  
  /* Set counter for timer1 and timer2 */
  MFT_SetCounter(MFT1, 1200 - 1, 5 - 1);
  
  /* Enable MFT Interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = MFT1A_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  MFT_EnableIT(MFT1, MFT_IT_TNA , ENABLE);
}


/**
* @brief  RTC timer Configuration
* @param  void
* @retval None
*/
void RTC_Timer_Configuration(void)
{
  RTC_InitType RTC_Init_struct;
  NVIC_InitType NVIC_InitStructure;
  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_RTC, ENABLE);
  
  /* < Periodic RTC mode */
  RTC_Init_struct.RTC_operatingMode = RTC_TIMER_PERIODIC;
  /* < Pattern size set to 1 */
  RTC_Init_struct.RTC_PATTERN_SIZE = 1 - 1;     
  /* < Enable 0.5s timer period */
  RTC_Init_struct.RTC_TLR1 = RTC_PERIOD_500ms;
  /* < RTC_TLR1 selected for time generation */
  RTC_Init_struct.RTC_PATTERN1 = 0x00;                          
  RTC_Init(&RTC_Init_struct);
  
  /* Enable RTC Timer interrupt*/
  RTC_IT_Config(RTC_IT_TIMER, ENABLE);
  RTC_IT_Clear(RTC_IT_TIMER);
  
  /** Delay between two write in RTC0->TCR register has to be
  *  at least 3 x 32k cycle + 2 CPU cycle. For that reason it
  *  is neccessary to add the delay. 
  */
  for (volatile uint16_t i=0; i<300; i++) {
    __asm("NOP");
  }
  
  /* Set the RTC_IRQn interrupt priority and enable it */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable RTC */
  RTC_Cmd(ENABLE);
}

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
