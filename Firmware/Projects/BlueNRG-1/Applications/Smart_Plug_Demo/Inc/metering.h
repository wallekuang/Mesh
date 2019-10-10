/**
  ******************************************************************************
  * @file    metering.h 
  * @author  System Lab Team
* @version V1.11.000
* @date    25-07-2019
  * @brief   Contains energy parameter calculation function prototypes.
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

#ifndef __metering_H
#define __metering_H

/* Includes ------------------------------------------------------------------*/ 
#include "metrology.h"
#include "metrology_hal.h"
#include "stpm_metrology.h"

/* Private defines ------------------------------------------------------------*/
#define ALL_REG                 70      /* Total no. of registers in STPM*/
#define READ_ONLY               49      /* Total no. of read only registers in STPM*/
#define START_ADDRESS           0x00    /*Initial address of STPM IC*/
#define READ_ADDRESS            0x48    /*Initial address of the registers to be read*/
#define REG_READ                18      /*Total registers to be read during one read cycle*/

#define SAGSWELLTIMEOUT_ZERO    0       /*Bit 0 to 3*/
#define SAGSWELLTIMEOUT_MAX     15      /*Bit 0 to 3*/   
#define SAGSWELLTIMEOUT_CLEAR   1       /*Bit 4*/
    
#define VREF_1V8                2       /*Bit 6,7,8*/

#define LED_SPEEDDIVISION       4       /*bit 24-27*/

#define VOLTSAGTIMERTH_10ms     1248    /*Bit 0 to 13*/

#define POWER_ARRAY_LENGTH      10

#define CURRENT_CAL_FACTOR      1713//2407
#define VOLTAGE_CAL_FACTOR      1750//1763
#define POWER_OFFSET_VAL        294


/* External variables --------------------------------------------------------*/

extern METRO_Device_Config_t *p_Metro_Device_Config;

/* Function prototypes -------------------------------------------------------*/
void STPM_Init(METRO_NB_Device_t Device_Id, METRO_Channel_t Metro_Channel);
void Measure_Param(METRO_NB_Device_t Device_Id, METRO_Channel_t Metro_Channel);
uint32_t Phase_ReadEnergy(METRO_Channel_t Metro_Channel,METRO_NB_Device_t Device_Id, METRO_Energy_selection_t Metro_Energy_Selection);
uint32_t Phase_ReadPower(METRO_Channel_t Metro_Channel,METRO_NB_Device_t Device_Id, METRO_Power_selection_t Metro_Power_Selection);
void Comm_Test( METRO_NB_Device_t Device_Id) ;
void PhaseRead_Block( METRO_NB_Device_t Device_Id, uint32_t Offset_Adress, uint8_t Nb_of_32b_Reg);
void PhaseWrite_Block( METRO_NB_Device_t Device_Id, uint8_t Offset_Adress, uint8_t Nb_of_32b_Reg);

#endif

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/


