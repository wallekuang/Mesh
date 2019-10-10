/**
******************************************************************************
* @file    appli_sensor.h
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Application interface for Light Mesh Model
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
#ifndef __APPLI_SENSOR_H
#define __APPLI_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"

/* Exported macro ------------------------------------------------------------*/

#define CONTROLLER_WAIT_TIME            1000

/* Exported variables  -------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
/* Application variables------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Sensor Cadence set */
#pragma pack(1)
typedef struct
{
  MOBLEUINT16 Prop_ID;
  MOBLEUINT16 PositiveTolerance;
  MOBLEUINT16 NegativeTolerance;
  MOBLEUINT8 SamplingFunction;
  MOBLEUINT8 MeasurementPeriod;
  MOBLEUINT8 UpdateInterval; 
}Appli_Sensor_DescriptorStatus_t;

/* Sensor Setting set */
#pragma pack(1)
typedef struct 
{
  MOBLEUINT16 Property_ID; 
  MOBLEUINT16 Sensor_Setting_ID; 
  MOBLEUINT8 Sensor_Setting_Access;
  MOBLEUINT16 Sensor_Setting_Value;
}Appli_Sensor_SettingSet_t;


/* structure of flags used for publishing data */
#pragma pack(1)
typedef struct 
{
  MOBLEBOOL CadenceDurationFlag ;
  MOBLEBOOL DeltaDataFlag ;
}PublishingDataFlag_t;

/* structure for the cadence set */
#pragma pack(1)
typedef struct 
{
  MOBLEUINT16 Property_ID; 
  MOBLEUINT8 FastCadenceDevisor;
  MOBLEUINT8 StatusTriggerType; 
  MOBLEUINT8 triggerDeltaDown;
  MOBLEUINT8 triggerDeltaUp;
  MOBLEUINT8 StatusMinInterval;
  float FastCadenceLow;
  float FastCadenceHigh;
}Sensor_CadenceSet_t;

#pragma pack(4)
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LSM6DSO_Axes_t;

MOBLE_RESULT Appli_Sensor_Cadence_Set(Sensor_CadenceParam_t* pCadence_param, MOBLEUINT16 property_ID,
                                                                           MOBLEUINT32 length); 
MOBLE_RESULT Appli_Sensor_Data_Status(MOBLEUINT8* sensor_Data , MOBLEUINT32* pLength, 
                                              MOBLEUINT16 prop_ID , MOBLEUINT32 length);
MOBLE_RESULT Appli_Sensor_Descriptor_Status(MOBLEUINT8* sensor_Discriptor , 
                                                           MOBLEUINT32* pLength);
MOBLE_RESULT Appli_Sensor_Setting_Set(Sensor_SettingParam_t* pSensor_SettingParam,
                                                           MOBLEUINT8 OptionalValid);                                      

void Sensor_Publication_Process(float* , MODEL_Property_IDTableParam_t*);
void SensorDataPublish(MOBLEUINT32 * , MOBLEUINT16*);
void Read_Sensor_Data(float *);
MOBLE_RESULT Check_Property_ID(const MODEL_Property_IDTableParam_t prop_ID_Table[] 
                                                         , MOBLEUINT16 prop_ID);

MOBLE_RESULT Appli_Sensor_GetSettingStatus(MOBLEUINT8* pSetting_Status);
MOBLE_RESULT Appli_Sensor_GetSetting_IDStatus(MOBLEUINT8* pSetting_Status);
MOBLE_RESULT Appli_Sensor_Init(void); 
void Sensor_Process(void);
void Sensor_LC_Light_Publish(void);
void Init_Pressure_Temperature_Sensor(void);
void Init_Humidity_Sensor(void);
void Init_Accelerometer_Gyroscope(void);
#endif /* __APPLI_SENSOR_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

