/**
******************************************************************************
* @file    appli_generic.c
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Application interface for Generic Mesh Models 
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
#include "hal_common.h"
#include "types.h"
#include "appli_generic.h"
#include "appli_light.h"
#include "common.h"
#include "mesh_cfg_usr.h"
#include "appli_nvm.h"
/** @addtogroup BlueNRG_Mesh
*  @{
*/

/** @addtogroup models_BlueNRG2
*  @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

MOBLEUINT8 RestoreFlag;
extern MOBLEUINT16 IntensityValue;
extern MOBLEUINT8 IntensityFlag;
extern MOBLEUINT8 PowerOnOff_flag;
extern Appli_LightPwmValue_t Appli_LightPwmValue;

Appli_Generic_OnOffSet AppliOnOffSet;
Appli_Generic_LevelSet AppliLevelSet;
Appli_Generic_PowerOnOffSet AppliPowerOnSet;
Appli_Generic_DefaultTransitionSet AppliDefaultTransitionSet;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef ENABLE_GENERIC_MODEL_SERVER_ONOFF
/**
* @brief  Appli_Generic_OnOff_Set: This function is callback for Application
*          when Generic OnOff message is received
* @param  pGeneric_OnOffParam: Pointer to the parameters received for message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_OnOff_Set(Generic_OnOffStatus_t* pGeneric_OnOffParam, 
                                     MOBLEUINT8 OptionalValid)
{ 
   AppliOnOffSet.Present_OnOff = pGeneric_OnOffParam->Present_OnOff_State;
   AppliOnOffSet.Present_OnOffValue = pGeneric_OnOffParam->Present_OnOff_Value;
   

/* This condition is applicable when user want to on off the light with some default 
    transition value, or optionalValid =IN_TRANSITION ,  transition is in progress.
*/
  if((OptionalValid == DEFAULT_TRANSITION) || (OptionalValid == IN_TRANSITION))
  {
    Appli_LightPwmValue.IntensityValue = AppliOnOffSet.Present_OnOffValue;
    Light_UpdateLedValue(LOAD_STATE ,Appli_LightPwmValue);
  }  
  else
    {
    if((AppliOnOffSet.Present_OnOff == APPLI_LED_ON) && (OptionalValid == NO_TRANSITION))
    { 
      Appli_LightPwmValue.PwmRedValue = 0;
      Appli_LightPwmValue.PwmGreenValue = PWM_VALUE_OFF;
      Appli_LightPwmValue.PwmBlueValue = PWM_VALUE_OFF;  
      Appli_LightPwmValue.IntensityValue = PWM_VALUE_OFF;
      Light_UpdateLedValue(LOAD_STATE , Appli_LightPwmValue);
    }
    else
    {
      Appli_LightPwmValue.PwmRedValue = PWM_TIME_PERIOD;
      Appli_LightPwmValue.PwmGreenValue = PWM_TIME_PERIOD;
      Appli_LightPwmValue.PwmBlueValue = PWM_TIME_PERIOD;
      Appli_LightPwmValue.IntensityValue = PWM_TIME_PERIOD;
      Light_UpdateLedValue(RESET_STATE , Appli_LightPwmValue);
    } 
    }
            
  TRACE_M(TF_SERIAL_CTRL,"#8202%02hx!\n\r",AppliOnOffSet.Present_OnOff);
  
  /* set the flag value for NVM store */
  RestoreFlag = GENERIC_ON_OFF_NVM_FLAG;
  
  AppliNvm_SaveMessageParam();

    return MOBLE_RESULT_SUCCESS;
}
#endif   

#ifdef ENABLE_GENERIC_MODEL_SERVER_LEVEL
/**
* @brief  Appli_Generic_Level_Set: This function is callback for Application
*          when Generic Level message is received
* @param  plevelParam: Pointer to the parameters message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_Level_Set(Generic_LevelStatus_t* plevelParam, 
                                     MOBLEUINT8 OptionalValid)
{
  MOBLEUINT16 duty;
  static MOBLEUINT16 previousIntensity = 0;
  
  AppliLevelSet.Present_Level16= plevelParam->Present_Level16;   
  if(AppliLevelSet.Present_Level16 <= 0)
  {
    AppliLevelSet.Present_Level16 = 0;
  }
    
  IntensityValue =  AppliLevelSet.Present_Level16;
  
  if(((IntensityValue > previousIntensity) && (IntensityValue <PWM_TIME_PERIOD)) ||
     IntensityValue == INTENSITY_LEVEL_ZERO)
  {
    IntensityFlag = FALSE;
  }
  else
  {
    IntensityFlag = TRUE;
  }
  previousIntensity = IntensityValue;
  
  duty = PwmValueMapping(AppliLevelSet.Present_Level16 , 0x7FFF ,0); 
  Appli_LightPwmValue.IntensityValue = duty;
  Light_UpdateLedValue(LOAD_STATE , Appli_LightPwmValue);
      
  TRACE_M(TF_SERIAL_CTRL,"#8206%04hx!\n\r",AppliLevelSet.Present_Level16);
      
  /* set the flag value for NVM store */
  RestoreFlag = GENERIC_LEVEL_NVM_FLAG;
  
  AppliNvm_SaveMessageParam();

  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Appli_Generic_LevelDelta_Set: This function is callback for Application
*          when Generic Level Delta message is received
* @param  pdeltalevelParam: Pointer to the parameters message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_LevelDelta_Set(Generic_LevelStatus_t* pdeltalevelParam, 
                                     MOBLEUINT8 OptionalValid)
{
      
  AppliLevelSet.Present_Level16 = pdeltalevelParam->Present_Level16;
  
  /* For demo, if Level is more than 50, switch ON the LED */
  if (AppliLevelSet.Present_Level16 >= 50)
  {
    Appli_LightPwmValue.IntensityValue = PWM_TIME_PERIOD;
    Light_UpdateLedValue(LOAD_STATE , Appli_LightPwmValue);
  }
  else
  {
    Light_UpdateLedValue(RESET_STATE , Appli_LightPwmValue);
  }
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Appli_Generic_LevelMove_Set: This function is callback for Application
*          when Generic Level Move message is received
* @param  pdeltaMoveParam: Pointer to the parameters message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_LevelMove_Set(Generic_LevelStatus_t* pdeltaMoveParam, 
                                               MOBLEUINT8 OptionalValid)
{
     
    if(OptionalValid == 1)
    {
      AppliLevelSet.Present_Level16= pdeltaMoveParam->Present_Level16;   
    }
  
    return MOBLE_RESULT_SUCCESS;
}
#endif   /* ENABLE_GENERIC_MODEL_SERVER_LEVEL */


#ifdef ENABLE_GENERIC_MODEL_SERVER_POWER_ONOFF
/**
* @brief  Appli_Generic_PowerOnOff_Set: This function is callback for Application
*           when Generic Power on off set message is received
* @param  pPowerOnOffParam: Pointer to the parameters message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_PowerOnOff_Set(Generic_PowerOnOffParam_t* pPowerOnOffParam, 
                                         MOBLEUINT8 OptionalValid)
{ 
  AppliPowerOnSet.PowerOnState = pPowerOnOffParam->PowerOnOffState;
  
  /* set the flag value for NVM store */
  RestoreFlag = GENERIC_ON_OFF_NVM_FLAG;

  AppliNvm_SaveMessageParam();
    
  return MOBLE_RESULT_SUCCESS;
}
#endif  /* ENABLE_GENERIC_MODEL_SERVER_POWER_ONOFF */


#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME
/**
* @brief  Appli_Generic_DefaultTransitionTime_Set: This function is callback for Application
*          when Generic Power on off set message is received
* @param  pDefaultTimeParam: Pointer to the parameters message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_DefaultTransitionTime_Set(Generic_DefaultTransitionParam_t* pDefaultTimeParam, 
                                         MOBLEUINT8 OptionalValid)
{
  
  AppliDefaultTransitionSet.DefaultTransitionTime = pDefaultTimeParam->DefaultTransitionTime;
  
  return MOBLE_RESULT_SUCCESS;
}
#endif   /* ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME */


/**
* @brief  Appli_Generic_GetOnOffState: This function is callback for Application
*          when Generic on off status message is to be provided
* @param  pOnOff_status: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_GetOnOffStatus(MOBLEUINT8* pOnOff_Status)                                        
{
  
   *pOnOff_Status = AppliOnOffSet.Present_OnOff;
  return MOBLE_RESULT_SUCCESS; 
}

/**
* @brief  Appli_Generic_GetOnOffValue: This function is callback for Application
          to get the PWM value for the generic on off
* @param  pOnOff_Value: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_GetOnOffValue(MOBLEUINT8* pOnOff_Value)                                        
{
  
  *pOnOff_Value = AppliOnOffSet.Present_OnOffValue;
  *(pOnOff_Value+1) = AppliOnOffSet.Present_OnOffValue >> 8;
  return MOBLE_RESULT_SUCCESS; 
}

/**
* @brief  Appli_Generic_GetLevelStatus: This function is callback for Application
           when Generic Level status message is to be provided
* @param  pLevel_status: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_GetLevelStatus(MOBLEUINT8* pLevel_Status) 
{ 
   *pLevel_Status = AppliLevelSet.Present_Level16;
   *(pLevel_Status+1) = AppliLevelSet.Present_Level16 >> 8;
  
  return MOBLE_RESULT_SUCCESS; 
}
                                        
/**
* @brief  Appli_Generic_GetPowerOnOffStatus: This function is callback for Application
*          when Generic Get Power status message is to be provided
* @param  pLevel_status: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_GetPowerOnOffStatus(MOBLEUINT8* pLevel_Status) 
{ 
  *pLevel_Status = AppliPowerOnSet.PowerOnState;
  
  return MOBLE_RESULT_SUCCESS; 
}

/**
* @brief  Appli_Generic_GetDefaultTransitionStatus: This function is callback for 
*           Application when Generic Level status message is to be provided
* @param  pTransition_Status: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Generic_GetDefaultTransitionStatus(MOBLEUINT8* pTransition_Status) 
{ 
  *pTransition_Status = AppliDefaultTransitionSet.DefaultTransitionTime;
  
  return MOBLE_RESULT_SUCCESS; 
}

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

