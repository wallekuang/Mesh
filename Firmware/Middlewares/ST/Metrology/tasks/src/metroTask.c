/**
*   @file      metroTask.c
*   @author    STMicroelectronics
*   @version   V1.0
*   @date      17 May 2016
*   @brief     This source code includes Metrology legal task related functions
*   @note      (C) COPYRIGHT 2013 STMicroelectronics
*
* @attention
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*/

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/
#include "metro_Task.h"
#include "metrology.h"
#include "handler_metrology.h"
#include "string.h"

/** @addtogroup LEGAL
* @{
*/

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/

#define FACTOR_POWER_ON_ENERGY      (858)   // (3600 * 16000000 / 0x4000000) = 858.3...
uint8_t StopRead=0;


/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/
metroData_t metroData;
METRO_Device_Config_t Tab_METRO_Global_Devices_Config[NB_MAX_DEVICE];

extern METRO_Device_Config_t Tab_METRO_internal_Devices_Config[NB_MAX_DEVICE];

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/

/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/
char text[100];

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/


/*******************************************************************************
*
*                       IMPLEMENTATION: Private functions
*
*******************************************************************************/

/**
* @brief  This function implements the Metrology init
* @param  None
* @retval None
*/
void METRO_Init()
{
  
#ifdef EEPROM_WORKING
  MET_Conf();
  
  /* initialization device type and number of channel */
  Metro_Setup(metroData.nvm->config[0],metroData.nvm->config[1]);
#else
  /* Basic setup of all STPM ICs
  Phase 1 STPM32 : Channel_1 is affected
  Phase 2 STPM32 : Channel_2 is affected
  Phase 3 STPM33 : Channel_3,Channel_4 are affected
  */
  Metro_Setup(HOST_CONFIG_DATA, METRO_CONFIG_DATA);
#endif
  /* initialization device communication port */ 
  Metro_com_port_device();
  
  /* Enable for STPM device */
  Metro_power_up_device();
  
  /* initialization steps for STPM device */
  Metro_Init();
  
#ifdef UART_XFER_STPM3X /* UART MODE */   
  /* Change UART speed for STPM communication between Host and EXT1*/
  Metro_UartSpeed(USART_SPEED); 
#endif
  
#ifdef EEPROM_WORKING
  MET_RestoreConfigFromNVM();
  //NB_MAX_DEVICE =1;
  /* Initialize the factors for the computation */
  for(int i=0;i<CHANNEL_2;i++)
  {
    Metro_Set_Hardware_Factors( (METRO_Channel_t)(CHANNEL_1+i), metroData.nvm->powerFact[i], metroData.nvm->powerFact[i]/ FACTOR_POWER_ON_ENERGY,metroData.nvm->voltageFact[i],metroData.nvm->currentFact[i]/4);
  }
#else
  for(int i=1;i<NB_MAX_CHANNEL;i++)
  {
    Metro_Set_Hardware_Factors(CHANNEL_1,POWER_LSB_FACTOR,ENERGY_LSB_FACTOR,VOLTAGE_LSB_FACTOR,CURRENT_LSB_FACTOR);
  }
#endif
  
  for (int i=EXT1;i<(NB_MAX_DEVICE);i++)
  {
    if(Tab_METRO_internal_Devices_Config[i].device != 0)
    {
      /* Set default latch device type inside Metro struct for Ext chips */
      Metro_Register_Latch_device_Config_type((METRO_NB_Device_t)i, LATCH_SYN_SCS);
    }
  }
}

/**
* @brief  This function implements the Metrology latch device
*         Set the HW latch for next update
* @param  None
* @retval None
*/
void METRO_Latch_Measures()
{
  METRO_NB_Device_t i;
  
  for (i=EXT1;i<(NB_MAX_DEVICE);i++)
  {
    if(Tab_METRO_internal_Devices_Config[i].device != 0)
    {
      Metro_Set_Latch_device_type(i,LATCH_SYN_SCS);
    }
  }
}

/**
* @brief  This function implements the Metrology get DSP data inside device
* @param  None
* @retval None
*/
void METRO_Get_Measures()
{
  METRO_NB_Device_t i;
  
  for (i=EXT1;i<(NB_MAX_DEVICE);i++)
  {
    if(Tab_METRO_internal_Devices_Config[i].device != 0)
    {
      Metro_Get_Data_device(i);
    }
  }
  
}

/**
* @brief  This function updates the Metro measurements values
* @param  None
* @retval None
*/
void METRO_UpdateData(void)
{
  metroData.powerActive    = 0;
  metroData.powerReactive  = 0;
  metroData.powerApparent  = 0;
  
  for(int i=0;i<metroData.nbPhase;i++)
  {
    metroData.rawEnergyExt[METRO_DATA_ACTIVE  ][METRO_PHASE_1+i] = Metro_Read_energy((METRO_Channel_t)(CHANNEL_1+i), E_W_ACTIVE);
    metroData.rawEnergyExt[METRO_DATA_REACTIVE  ][METRO_PHASE_1+i] = Metro_Read_energy((METRO_Channel_t)(CHANNEL_1+i), E_REACTIVE);
    metroData.rawEnergyExt[METRO_DATA_APPARENT  ][METRO_PHASE_1+i] = Metro_Read_energy((METRO_Channel_t)(CHANNEL_1+i), E_APPARENT);
    
    metroData.chanPower[METRO_DATA_ACTIVE  ][METRO_PHASE_1+i] = Metro_Read_Power((METRO_Channel_t)(CHANNEL_1+i), W_ACTIVE);
    metroData.chanPower[METRO_DATA_REACTIVE][METRO_PHASE_1+i] = Metro_Read_Power((METRO_Channel_t)(CHANNEL_1+i), REACTIVE);
    metroData.chanPower[METRO_DATA_APPARENT][METRO_PHASE_1+i] = Metro_Read_Power((METRO_Channel_t)(CHANNEL_1+i), APPARENT_RMS);
    metroData.powerActive    += metroData.chanPower[METRO_DATA_ACTIVE  ][METRO_PHASE_1+i];
    metroData.powerReactive  += metroData.chanPower[METRO_DATA_REACTIVE][METRO_PHASE_1+i];
    metroData.powerApparent  += metroData.chanPower[METRO_DATA_APPARENT][METRO_PHASE_1+i];
  }
  
}

/**
* @}
*/

/* End Of File */
