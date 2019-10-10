/**
******************************************************************************
* @file    appli_light_lc.c
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Application interface for light LC Mesh Models 
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
#include "appli_light_lc.h"

/** @addtogroup BlueNRG_Mesh
*  @{
*/

/** @addtogroup models_BlueNRG2
*  @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variable ----------------------------------------------------------*/

Appli_LightLC_Set_t Appli_LightLC_set;
Appli_Light_LC_PropertySet_t Appli_LightLC_PropertySet; 

MOBLEUINT16 AmbientLuxLevel;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Appli_Light_LCMode_Set: This function is callback for Application
when Light LC mode Set message is received
* @param  pLight_LC_Param: Pointer to the parameters received for message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_LightLC_Mode_Set(Light_LC_Param_t* pLight_LC_Param,
                                                     MOBLEUINT8 OptionalValid)
{
  Appli_LightLC_set.LC_mode = pLight_LC_Param->LC_mode;
  return MOBLE_RESULT_SUCCESS;
}
  
/**
* @brief  Appli_LightLC_OM_Set: This function is callback for Application
when Light LC mode Occupancy Model Set message is received
* @param  pLight_LC_Param: Pointer to the parameters received for message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_LightLC_OM_Set(Light_LC_Param_t* pLight_LC_Param,
                                                     MOBLEUINT8 OptionalValid)
{
  Appli_LightLC_set.LC_OM = pLight_LC_Param->LC_OM;
  return MOBLE_RESULT_SUCCESS;
}  

/**
* @brief  Appli_LightLC_OnOff_Set: This function is callback for Application
when Light LC On Off Set message is received
* @param  pLight_LC_Param: Pointer to the parameters received for message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_LightLC_OnOff_Set(Light_LC_Param_t* pLight_LC_Param,
                                                     MOBLEUINT8 OptionalValid)
{
  Appli_LightLC_set.Light_OnOffState = pLight_LC_Param->Light_OnOff;
  return MOBLE_RESULT_SUCCESS;
}  

/**
* @brief Get_AmbientLuxLevelOutput: This function is given to calculate the Ambient 
         Lux level output from the ambient sensor.
* @param void:
* @retval MOBLEUINT16:    
**/
MOBLEUINT16 Appli_LightLC_Get_AmbientLuxLevelOutput(void)
{
  /* Use AmbientLuxLevel global variable to store the value from ambient lux sensor
     and then use this value to copmpare the lux level output from state machine.
   */
  
  return AmbientLuxLevel;
}

/**
* @brief Light_LC_LuxLevelPIRegulator: This function will calculate all the parameter
         Kid,kpu,kiu,kpd and return the value Light Lightness Linear.           
* @param void:
* @retval MOBLEUINT16:    
**/
MOBLEUINT16 Appli_Light_LC_PIRegulatorOutput(MOBLEUINT16 tableLuxLevel,MOBLEUINT16 ambientLuxLevel)
{
  MOBLEUINT16 luxLevel = 0;
  /* User can write their code for the calculation */
  
  
  return luxLevel;
}

/**
* @brief  Appli_LightLC_Get_ModeStatus: This function is callback for Application
to get the application values in middleware used for transition change.
* @param  lcModeState: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_LightLC_Get_ModeStatus(MOBLEUINT8* plcModeState)
{  
  *(plcModeState) = Appli_LightLC_set.LC_mode;
  
  return MOBLE_RESULT_SUCCESS;
}  

/**
* @brief  Appli_LightLC_Get_OMModeStatus: This function is callback for Application
to get the application values in middleware used for transition change.
* @param  lcOM_ModeState: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_LightLC_Get_OMModeStatus(MOBLEUINT8* plcOM_ModeState)
{  
  *(plcOM_ModeState) = Appli_LightLC_set.LC_OM;
  
  return MOBLE_RESULT_SUCCESS;
}  

/**
* @brief  Appli_LightLC_Get_OnOffStatus: This function is callback for Application
to get the application values in middleware used for transition change.
* @param  lcOnOffState: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_LightLC_Get_OnOffStatus(MOBLEUINT8* plcOnOffState)
{  
  *(plcOnOffState) = Appli_LightLC_set.Light_OnOffState;
  
  return MOBLE_RESULT_SUCCESS;
}  


/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

