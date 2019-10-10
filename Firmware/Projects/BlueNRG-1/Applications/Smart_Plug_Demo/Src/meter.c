/**
******************************************************************************
* @file    meter.c 
* @author  System Lab
* @version V1.11.000
* @date    25-07-2019
* @brief   Meter initialization and measurement of energy parameters.
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
#include "meter.h"

/* Private typedef -----------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

extern uint32_t Phase1_ActiveFEnergy;
extern uint32_t Phase1_Channel1RMSVoltage;
extern uint32_t Phase1_Channel1RMSCurrent;
extern uint16_t Pulse_Peroid;

/* Private defines -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/**
* @brief Measures electrical parameters from STPM32
* @retval void
*/
void Meter(void)
{
  /* Latch data registers */
  Metro_Set_Latch_device_type(EXT1, LATCH_SYN_SCS); 
  /* Read 18 register rows for device 1 */
  PhaseRead_Block(EXT1, READ_ADDRESS, REG_READ);
  Measure_Param(EXT1, CHANNEL_1);
}

/**
* @brief STPM32 initialization
* @retval void
*/
void Meter_Init()
{
  Comm_Test(EXT1);
  PhaseRead_Block(EXT1, START_ADDRESS, ALL_REG);
  STPM_Init(EXT1, CHANNEL_1);
}

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
