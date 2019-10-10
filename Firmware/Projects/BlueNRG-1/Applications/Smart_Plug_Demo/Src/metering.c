/**
******************************************************************************
* @file    metering.c 
* @author  System Lab
* @version V1.11.000
* @date    25-07-2019
* @brief   Energy parameter calculation functions.
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

#include "metering.h"

/*Private variables ----------------------------------------------------------*/

uint16_t Pulse_Period;

uint32_t Phase1_ActiveFEnergy;
uint32_t Phase1_ApparentEnergy;
uint32_t Phase1_ReactiveEnergy;

uint32_t Cumm_ActiveFEnergy;
uint32_t Cumm_ApparentEnergy;
uint32_t Cumm_ReactiveEnergy;

uint32_t Phase1_ActiveFPower;
uint32_t Phase1_ApparentPower;
uint32_t Phase1_ReactivePower;  

uint32_t Phase1_Channel1RMSVoltage;
uint32_t Phase1_Channel1RMSCurrent;

uint32_t Phase1_Channel1InstVoltage;
uint32_t Phase1_Channel1InstCurrent;

uint32_t Phase1_MaximumFDemand=0;
uint32_t Cumm_MaximumFDemand=0;
uint32_t Phase1_MaximumSDemand=0;
uint32_t Cumm_MaximumSDemand=0;
uint32_t Phase1_PowerFactor;
uint32_t AveragePowerFactor;

uint32_t LCD_Date;
uint32_t LCD_Time;

uint32_t Phase1_PowerArray[POWER_ARRAY_LENGTH];
uint32_t Phase2_PowerArray[POWER_ARRAY_LENGTH];
uint32_t Phase3_PowerArray[POWER_ARRAY_LENGTH];

uint32_t Phase1_ArrayConst=0;

uint32_t Phase1_ReadFlag=0; 

/* External variables --------------------------------------------------------*/

extern uint8_t Phase1_PowerDown;


/* Functions -----------------------------------------------------------------*/

/**
* @brief Test the communication of STPM3x chip.
* @param Device_Id: METRO_NB_Device_t(Device Id of external chip).
* @retval void
*/
void    Comm_Test(METRO_NB_Device_t Device_Id)  
{
  /* variable to recieve the data*/
  uint32_t testData = 0;
  
  /* read a single register from STPM chip*/
  Metro_Read_Block_From_Device(Device_Id, 0x00, 1, &testData);
  
  /*Set power down for those phase that are not working*/
  if (testData > 0)
  {
    switch(Device_Id)
    {
    case EXT1:
      {
        Phase1_PowerDown = 0;
        break;
      }
    }
  }
}

/**
* @brief read a block of data from STPM3x chip.
* @param Device_Id: METRO_NB_Device_t(Device Id of external chip).
* @param Offset_Adress : uint32_t offset address of the first register to be read  
* @param Nb_of_32b_Reg : uint8_t no. of 32 bit registers to be read
* @retval void
*/

extern
void PhaseRead_Block( METRO_NB_Device_t Device_Id, uint32_t Offset_Adress,  
                     uint8_t Nb_of_32b_Reg)
{
  uint32_t *tmp_addr;
  /*address of the first register*/
  tmp_addr = &((p_Metro_Device_Config+Device_Id)->metro_stpm_reg.DSPCTRL1);
  
  /*address of the required register*/
  tmp_addr = tmp_addr + (Offset_Adress/2);
  Metro_Read_Block_From_Device(Device_Id,(uint8_t)Offset_Adress,Nb_of_32b_Reg,
                                tmp_addr);
}


/**
* @brief write a block of data to STPM3x chip.
* @param Device_Id: METRO_NB_Device_t(Device Id of external chip).
* @param Offset_Adress : uint32_t offset address of the first register to write 
* @param Nb_of_32b_Reg : uint8_t no. of 32 bit registers to be write
* @retval void
*/
void PhaseWrite_Block(METRO_NB_Device_t Device_Id, uint8_t Offset_Adress,
                      uint8_t Nb_of_32b_Reg)
{
  uint32_t *tmp_addr;
  /*address of the first register*/
  tmp_addr = &((p_Metro_Device_Config+Device_Id)->metro_stpm_reg.DSPCTRL1);
  
  /*address of the required register*/
  tmp_addr = tmp_addr + (Offset_Adress/2);
  Metro_Write_Block_to_Device(Device_Id,Offset_Adress,Nb_of_32b_Reg,tmp_addr);
}


/**
* @brief Initialize the STPM3x chip.
* @param Device_Id: METRO_NB_Device_t(Device Id of external chip).
* @param Metro_Channel : METRO_Channel_t(Channel for that external chip).
* @retval void
*/
void STPM_Init(METRO_NB_Device_t Device_Id, METRO_Channel_t Metro_Channel)
{   
  /*Set all the basic parameters in DSPCTRL1,DSPCTRL2,DSPCTRL3,DSPCTRL4,
  DFECTRL1,DFECTRL2*/

  /*Sag Swell settings*/
  Metro_Set_SAG_and_SWELL_Clear_Timeout(Metro_Channel, SAGSWELLTIMEOUT_MAX);  
  Metro_Set_SAG_Config(Metro_Channel, 0, VOLTSAGTIMERTH_10ms);
  
  /*Vref settings*/
  Metro_Set_Vref(Metro_Channel, INT_VREF);
  Metro_Set_Temperature_Compensation(Metro_Channel, 4);
  
  /*HP_Filter and coil Integrator settings*/
  Metro_Set_Current_HP_Filter(Metro_Channel, DEVICE_DISABLE);
  Metro_Set_Voltage_HP_Filter(Metro_Channel, DEVICE_DISABLE);
  Metro_Set_Coil_integrator(Metro_Channel, DEVICE_DISABLE);
  
  /*LED1 and LED2 settings*/
  Metro_Set_Led_On_Off(Device_Id,RED_LED_BOARD,DEVICE_ENABLE);
  Metro_Set_Led_On_Off(Device_Id,GREEN_LED_BOARD,DEVICE_ENABLE);
  Metro_Set_Led_Power_Config(Device_Id,RED_LED_BOARD,LED_F_ACTIVE);
  Metro_Set_Led_Power_Config(Device_Id,GREEN_LED_BOARD,LED_REACTIVE);
  Metro_Set_Led_Speed_divisor(Device_Id, RED_LED_BOARD,LED_SPEEDDIVISION);
  Metro_Set_Led_Speed_divisor(Device_Id, GREEN_LED_BOARD,LED_SPEEDDIVISION);
  Metro_Set_Led_Channel_Config(Device_Id,RED_LED_BOARD,PRIMARY);
  
  /*Latch settings of STPM3x device*/
  Metro_Set_Latch_device_type(Device_Id, LATCH_SYN_SCS);
  
  /* Set Current Gain */
  Metro_Set_Current_gain(Metro_Channel, X8);
  
  /* Calibrate Current */
  Metro_HAL_Set_C_Calibration(EXT1, INT_CHANNEL_1, CURRENT_CAL_FACTOR);
  /* Calibrate Voltage */
  Metro_HAL_Set_V_Calibration(EXT1, INT_CHANNEL_1, VOLTAGE_CAL_FACTOR);
  /* Calibrate Power Offset */
  Metro_HAL_Set_Power_Offset_Compensation(EXT1, INT_CHANNEL_1, F_ACTIVE, 419);
  Metro_HAL_Set_Power_Offset_Compensation(EXT1, INT_CHANNEL_1, W_ACTIVE, 112);
  Metro_HAL_Set_Power_Offset_Compensation(EXT1, INT_CHANNEL_1, APPARENT_RMS, 257);
  Metro_HAL_Set_Power_Offset_Compensation(EXT1, INT_CHANNEL_1, REACTIVE, 109);
  /* Calibrate Phase offset */
  Metro_HAL_Set_Phase_C_Calibration(EXT1, INT_CHANNEL_1, 239);
  Metro_HAL_Set_Phase_V_Calibration(EXT1, INT_CHANNEL_1, 1);
}

/**
* @brief Measure the parameters from read values of registers.
* @param Device_Id: METRO_NB_Device_t(Device Id of external chip).
* @param Metro_Channel : METRO_Channel_t(Channel for that external chip).
* @retval void
*/
void Measure_Param(METRO_NB_Device_t Device_Id, METRO_Channel_t Metro_Channel)
{
  
  /* Measure parameters of external chip 1*/
  if ((Device_Id == EXT1) && (Phase1_PowerDown == 0))
  {
    Phase1_ActiveFEnergy = Phase_ReadEnergy(Metro_Channel, Device_Id, E_F_ACTIVE);
    Phase1_ActiveFPower = Phase_ReadPower(Metro_Channel, Device_Id, F_ACTIVE);
    Phase1_ApparentEnergy = Phase_ReadEnergy(Metro_Channel, Device_Id, E_APPARENT);
    Phase1_ApparentPower = Phase_ReadPower(Metro_Channel, Device_Id, APPARENT_RMS);
    Phase1_ReactiveEnergy = Phase_ReadEnergy(Metro_Channel, Device_Id, E_REACTIVE);
    Phase1_ReactivePower = Phase_ReadPower(Metro_Channel, Device_Id, REACTIVE);
    Metro_Read_RMS(CHANNEL_1, &Phase1_Channel1RMSVoltage,
                   &Phase1_Channel1RMSCurrent, 1);
    Pulse_Period = Metro_Read_Period(Metro_Channel);
    if (Phase1_ActiveFPower > Phase1_MaximumFDemand)
    {
      Phase1_MaximumFDemand = Phase1_ActiveFPower;
    }
    
    if (Phase1_ApparentPower > Phase1_MaximumSDemand)
    {
      Phase1_MaximumSDemand = Phase1_ApparentPower;
    }
    
    /* Power Factor */
    Phase1_PowerFactor = (Phase1_ActiveFPower*100)/Phase1_ApparentPower;
    
    Phase1_ReadFlag++;
  }
}

/**
* @brief Read the energy from STPM3x chip.
* @param Device_Id: METRO_NB_Device_t(Device Id of external chip).
* @param Metro_Channel : METRO_Channel_t(Channel for that external chip).
* @param Metro_Energy_Selection : METRO_Energy_selection_t(Type of Energy to be read).
* @retval uint32_t : read energy
*/
int32_t a_tempEnergy = 0;
int32_t r_tempEnergy = 0;
int32_t s_tempEnergy = 0;
uint32_t Phase_ReadEnergy(METRO_Channel_t Metro_Channel,
                          METRO_NB_Device_t Device_Id, 
                          METRO_Energy_selection_t Metro_Energy_Selection)
{
  uint32_t tempEnergy;
  
  tempEnergy=Metro_Read_energy(Metro_Channel,Metro_Energy_Selection);
if(Metro_Energy_Selection == E_F_ACTIVE){
  a_tempEnergy = tempEnergy;
}
 if(Metro_Energy_Selection == E_REACTIVE){
  r_tempEnergy = tempEnergy;
}
  if(Metro_Energy_Selection == E_APPARENT){
  s_tempEnergy = tempEnergy;
}
 
  if (((tempEnergy) & 0x80000000) > 0)
  {
    tempEnergy = ~tempEnergy;
    tempEnergy = tempEnergy + 1;
  }
  return (uint32_t)tempEnergy;
}

/**
* @brief Read the power from STPM3x chip.
* @param Device_Id: METRO_NB_Device_t(Device Id of external chip).
* @param Metro_Channel : METRO_Channel_t(Channel for that external chip).
* @param Metro_Power_Selection : METRO_Power_selection_t(Type of Power to be read).
* @retval void
*/
uint32_t Phase_ReadPower(METRO_Channel_t Metro_Channel,
                         METRO_NB_Device_t Device_Id, 
                         METRO_Power_selection_t Metro_Power_Selection)
{
  int32_t tempPower;
  
  uint8_t loopVariable;
  
  tempPower = Metro_Read_Power(Metro_Channel, Metro_Power_Selection);
  
  if (((tempPower) & 0x80000000) > 0)
  {
    tempPower = ~tempPower;
    tempPower = tempPower + 1;
  }
  
  if (Metro_Power_Selection == F_ACTIVE)
  {
    switch (Device_Id)
    {
    case EXT1:
      if (( (0.9 * (Phase1_ActiveFPower)) < tempPower) && 
          (tempPower < (1.1*(Phase1_ActiveFPower))))
      {
        Phase1_PowerArray[Phase1_ArrayConst] = tempPower;
        Phase1_ArrayConst++;
        tempPower = 0;
        for(loopVariable = 0; loopVariable < Phase1_ArrayConst; loopVariable++)
        {
          tempPower = tempPower + Phase1_PowerArray[loopVariable];
        }
        tempPower = tempPower/Phase1_ArrayConst;
        
        if (Phase1_ArrayConst == POWER_ARRAY_LENGTH)
        {
          Phase1_ArrayConst = 0;
        }
      }
      else
      {
        Phase1_ArrayConst = 0;
        Phase1_PowerArray[Phase1_ArrayConst] = tempPower;
      }
      break;    
    }
  }
  return (uint32_t)tempPower;
}

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
