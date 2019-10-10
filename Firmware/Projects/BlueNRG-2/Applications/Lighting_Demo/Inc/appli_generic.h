/**
******************************************************************************
* @file    appli_generic.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APPLI_GENERIC_H
#define __APPLI_GENERIC_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "generic.h"
#include "mesh_cfg.h"


/* Exported macro ------------------------------------------------------------*/
/* user configuration for Battery status */
#define BATTERY_ABSENT                 0X00
#define BATTERY_PRESENT_REMOVABLE      0X01
#define BATTERY_PRESENT_NON_REMOVABLE  0X02
#define BATTERY_UNKNOWN                0X03
#define BATTERY_CRITICAL_LOW_LEVEL     0X00
#define BATTERY_LOW_LEVEL              0X01
#define BATTERY_GOOD_LEVEL             0X02
#define BATTERY_LEVEL_UNKNOWN         0X03
#define BATTERY_NOT_CHARGEABLE         0X00
#define BATTERY_NOT_CHARGING           0X01
#define BATTERY_IS_CHARGING            0X02
#define BATTERY_CHARGING_UNKNOWN       0X03
#define BATTERY_SERVICE_RFU            0X00
#define BATTERY_REQUIRE_NO_SERVICE      0X01
#define BATTERY_REQUIRE_SERVICE        0X02
#define BATTERY_SERVICE_UNKNOWN        0X03   

/* Exported variables  ------------------------------------------------------- */
/* Application Variable-------------------------------------------------------*/
#pragma pack(1)
typedef struct
{
  MOBLEUINT8 Present_OnOff;
  MOBLEUINT16 Present_OnOffValue;
}Appli_Generic_OnOffSet;

#pragma pack(1)
typedef struct
{
  MOBLEINT16 Present_Level16; 
}Appli_Generic_LevelSet;

#pragma pack(1)
typedef struct
{
  MOBLEINT16 PowerOnState; 
}Appli_Generic_PowerOnOffSet;

#pragma pack(1)
typedef struct
{
  MOBLEINT16 DefaultTransitionTime; 
}Appli_Generic_DefaultTransitionSet;

#pragma pack(1)
typedef struct 
{
  MOBLEUINT8 Is_BatteryPresent;
  MOBLEUINT8 Is_Chargeable;
  MOBLEUINT8 Is_Serviceable;
}Appli_BatteryUserflag_param_t;

/* Exported Functions Prototypes ---------------------------------------------*/

MOBLE_RESULT Appli_Generic_OnOff_Set(Generic_OnOffStatus_t*, MOBLEUINT8);
MOBLE_RESULT Appli_Generic_Level_Set(Generic_LevelStatus_t*, MOBLEUINT8);
MOBLE_RESULT Appli_Generic_LevelDelta_Set(Generic_LevelStatus_t*, MOBLEUINT8 );
MOBLE_RESULT Appli_Generic_LevelMove_Set(Generic_LevelStatus_t* pdeltaMoveParam, 
                                                MOBLEUINT8 OptionalValid);
MOBLE_RESULT Appli_Generic_Level_Status(MOBLEUINT8* level_status, 
                                                MOBLEUINT32 *plength);
MOBLE_RESULT Appli_Generic_PowerOnOff_Set(Generic_PowerOnOffParam_t* pPowerOnOffParam, 
                                                MOBLEUINT8 OptionalValid);  
MOBLE_RESULT Appli_Generic_DefaultTransitionTime_Set(Generic_DefaultTransitionParam_t* pDefaultTimeParam, 
                                                MOBLEUINT8 OptionalValid);

MOBLE_RESULT Appli_Generic_GetOnOffStatus(MOBLEUINT8* pOnOff_Status);
MOBLE_RESULT Appli_Generic_GetOnOffValue(MOBLEUINT8* pOnOff_Value);
MOBLE_RESULT Appli_Generic_GetLevelStatus(MOBLEUINT8* pLevel_Status);
MOBLE_RESULT Appli_Generic_GetPowerOnOffStatus(MOBLEUINT8* pLevel_Status);
MOBLE_RESULT Appli_Generic_GetDefaultTransitionStatus(MOBLEUINT8* pTransition_Status) ;
 

#endif /* __APPLI_GENERIC_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

