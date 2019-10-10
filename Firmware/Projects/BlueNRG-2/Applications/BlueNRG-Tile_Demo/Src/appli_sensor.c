/**
******************************************************************************
* @file    appli_sensor.c
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Application interface for Sensor Mesh Models 
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
#include "sensors.h"
#include "light_lc.h"
#include "appli_sensor.h"
#include "mesh_cfg.h"
#include "PWM_handlers.h"
#include "PWM_config.h"
#include "LPS22HH.h"
#include "LSM6DSO.h"
#include "LIS2MDL.h"
#include "HTS221.h"
#include <string.h>
#include "common.h"
#include "math.h"
#include "LPS22HH_hal.h"
#include "LSM6DSO_hal.h"



/** @addtogroup BlueNRG_Mesh
*  @{
*/

/** @addtogroup models_BlueNRG2
*  @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

extern void Vendor_Publish(MOBLE_ADDRESS publishAddress);
extern MOBLEUINT8 NumberOfElements;
/**
 * @brief  PRESSURE init structure definition
 */

/* Application variables of sensor model definition */
#ifdef ENABLE_SENSOR_MODEL_SERVER

Appli_Sensor_DescriptorStatus_t Appli_Sensor_DescriptorStatus;
Appli_Sensor_SettingSet_t Appli_Sensor_SettingSet;

/* By Default value used for cadence set for testing. */
Sensor_CadenceSet_t Sensor_CadenceSet[NUMBER_OF_SENSOR] = {
                                  {0x0071 , 0x2 , 2 , 2 ,2 ,0 ,0X05 , 0x64},
                                  {0x2A6D , 0x2 , 1 , 1 , 1, 0, 0X258 , 0x3ED}
                                              };
#endif
#if USER_DEFINED_PLATFORM != USER_EVAL_PLATFORM
  /* Temperature and Pressure init structure*/
  PRESSURE_DrvTypeDef* xLPS25HBDrv = &LPS25HBDrv;
#endif

MODEL_Property_IDTableParam_t Property_ID_Table[NUMBER_OF_SENSOR] = {
                                                                      {TEMPERATURE_PID},
                                                                      {PRESSURE_PID}
                                                                    }; 
MOBLEUINT8 Occupancy_Flag = FALSE;


#if USER_DEFINED_PLATFORM == USER_EVAL_PLATFORM
  lsm6dso_pin_int1_route_t int1_route;
  lsm6dso_all_sources_t all_source;
#endif
  
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef ENABLE_SENSOR_MODEL_SERVER

/**
* @brief  Appli_Sensor_Cadence_Set: This function is callback for Application
when sensor cadence Set message is received
* @param  pCadence_param: Pointer to the parameters received for message
* @param  property_ID: Property is of sensor coming in data packet
* @param  length: Length of data coming in packet.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Sensor_Cadence_Set(Sensor_CadenceParam_t* pCadence_param, MOBLEUINT16 property_ID, MOBLEUINT32 length)                                    
{  
  if(property_ID == (MOBLEUINT16)TEMPERATURE_PID)
  {
    Sensor_CadenceSet[0].Property_ID = pCadence_param->Property_ID;
    Sensor_CadenceSet[0].FastCadenceDevisor = pCadence_param->FastCadenceDevisor ;
    Sensor_CadenceSet[0].StatusTriggerType = pCadence_param->StatusTriggerType ;
    Sensor_CadenceSet[0].triggerDeltaDown = pCadence_param->triggerDeltaDown;
    Sensor_CadenceSet[0].triggerDeltaUp = pCadence_param->triggerDeltaUp;
    Sensor_CadenceSet[0].StatusMinInterval = pCadence_param->StatusMinInterval;
    Sensor_CadenceSet[0].FastCadenceLow = pCadence_param->FastCadenceLow;
    Sensor_CadenceSet[0].FastCadenceHigh = pCadence_param->FastCadenceHigh;
  }
  else if(property_ID == (MOBLEUINT16)PRESSURE_PID)
  {
    
    Sensor_CadenceSet[1].Property_ID = pCadence_param->Property_ID;
    Sensor_CadenceSet[1].FastCadenceDevisor = pCadence_param->FastCadenceDevisor ;
    Sensor_CadenceSet[1].StatusTriggerType = pCadence_param->StatusTriggerType ;
    Sensor_CadenceSet[1].triggerDeltaDown = pCadence_param->triggerDeltaDown;
    Sensor_CadenceSet[1].triggerDeltaUp = pCadence_param->triggerDeltaUp;
    Sensor_CadenceSet[1].StatusMinInterval = pCadence_param->StatusMinInterval;
    Sensor_CadenceSet[1].FastCadenceLow = pCadence_param->FastCadenceLow;
    Sensor_CadenceSet[1].FastCadenceHigh = pCadence_param->FastCadenceHigh;
  }
  else
  {
    /* No Comments */
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Appli_Sensor_Setting_Set: This function is callback for Application
when sensor setting Set message is received
* @param  pSensor_SettingParam: Pointer to the parameters received for message
* @param  OptionalValid: Flag to inform about the validity of optional parameters 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Sensor_Setting_Set(Sensor_SettingParam_t* pSensor_SettingParam,
                                      MOBLEUINT8 OptionalValid)
{  
  Appli_Sensor_SettingSet.Property_ID = pSensor_SettingParam->Property_ID;
  Appli_Sensor_SettingSet.Sensor_Setting_ID = pSensor_SettingParam->Sensor_Setting_ID;
  Appli_Sensor_SettingSet.Sensor_Setting_Access = pSensor_SettingParam->Sensor_Setting_Access;
  Appli_Sensor_SettingSet.Sensor_Setting_Value = pSensor_SettingParam->Sensor_Setting_Value;
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Appli_Sensor_Data_Status: This function is callback for Application
when sensor get message is received
* @param  sensor_Data: Pointer to the parameters to be send in message
* @param  pLength: Length of the parameters to be sent in response
* @param  prop_ID: Property is of sensor coming in data packet
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Sensor_Data_Status(MOBLEUINT8* sensor_Data , MOBLEUINT32* pLength, 
                                      MOBLEUINT16 prop_ID , MOBLEUINT32 length)
{
   MOBLEUINT8 data_Length = 0x03;
   
  static axis1bit32_t data_raw_pressure;
  static axis3bit16_t data_raw_Acceleration,data_raw_angular_velocity,data_raw_magnetic_field;

  float humidity,temperature;
  LSM6DSO_Axes_t acceleration,angular_velocity,magnetic_field;
  uint8_t index= 0;
  float pressure = 0.0;
  
  /* Get Temperature data */
  HTS221_Get_Temperature_In_Float(0,&temperature);
  //returned temperature value that must be divided by 10 to get the value in ['C]
  temperature = temperature / 10;
  
  /* Get Humidit data */
  HTS221_Get_Humidity_Value_In_Float(0,&humidity);
  //returned humidity value that must be divided by 10 to get the value in [%].
  humidity = humidity / 10; 

  /* Get pressure data */
   memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
   lps22hh_pressure_raw_get(0, data_raw_pressure.u8bit);
   pressure = LPS22HH_FROM_LSB_TO_hPa(data_raw_pressure.i32bit);
    
  /* Get Acceleration data */
  float acc_sensitivity = 61.0f / 1000.0f;
  lsm6dso_acceleration_raw_get(0,data_raw_Acceleration.u8bit);
  acceleration.x = (int32_t)((float)((float)data_raw_Acceleration.i16bit[0] * acc_sensitivity));
  acceleration.y = (int32_t)((float)((float)data_raw_Acceleration.i16bit[1] * acc_sensitivity));
  acceleration.z = (int32_t)((float)((float)data_raw_Acceleration.i16bit[2] * acc_sensitivity));

  /* Get angular velocity data */

  lsm6dso_angular_rate_raw_get(0,data_raw_angular_velocity.u8bit);
  float gyro_sensitivity = 70.0f;
  angular_velocity.x = (int32_t)((float)((float)data_raw_angular_velocity.i16bit[0] * gyro_sensitivity));
  angular_velocity.y = (int32_t)((float)((float)data_raw_angular_velocity.i16bit[1] * gyro_sensitivity));
  angular_velocity.z = (int32_t)((float)((float)data_raw_angular_velocity.i16bit[2] * gyro_sensitivity));

 /* Get magnetic field data */
  lis2mdl_magnetic_raw_get(0,data_raw_magnetic_field.u8bit);
  float magno_sensitivity = 1.500f;
  magnetic_field.x = (int32_t)((float)((float)data_raw_magnetic_field.i16bit[0] * magno_sensitivity));
  magnetic_field.y = (int32_t)((float)((float)data_raw_magnetic_field.i16bit[1] * magno_sensitivity));
  magnetic_field.z = (int32_t)((float)((float)data_raw_magnetic_field.i16bit[2] * magno_sensitivity));  
      
 switch(prop_ID)
   {
 case TEMPERATURE_PID: 
     data_Length = 0x03;             
     *(sensor_Data + index) = ((TEMPERATURE_PID & 0x07) << 5) | (data_Length <<1) ; 
     index++;
     *(sensor_Data+index) = (TEMPERATURE_PID >> 3) & 0xFF; 
     index++;
     memcpy(&sensor_Data[index],(void*)&temperature,4);
     index = index + 4;             
     break;
      
 case PRESSURE_PID: 
      
      /* Format B for Pressure sensor */
     data_Length = 0x03;             
     *(sensor_Data + index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data + index) = (MOBLEUINT8)PRESSURE_PID ;
     index++;
     *(sensor_Data + index) = (MOBLEUINT8)(PRESSURE_PID >> 8);
     index++;
      
     memcpy(&sensor_Data[index],(void*)&pressure,4);
     index = index + 4;             
     break;
      
 case HUMIDITY_PID: 
     /* Format B for Humidity sensor */
     data_Length = 0x03;             
     *(sensor_Data+index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)HUMIDITY_PID ;
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)(HUMIDITY_PID >> 8);
     index++;
     
     memcpy(&sensor_Data[index],(void*)&humidity,4);
     index = index + 4;             
     break;
   
 case MAGNETO_METER_PID: 
     /* Format B for Magnetic sensor */
     data_Length = 11;             
     *(sensor_Data+index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)MAGNETO_METER_PID ;
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)(MAGNETO_METER_PID >> 8);
     index++;
     memcpy(&sensor_Data[index],(void*)&magnetic_field.x,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&magnetic_field.y,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&magnetic_field.z,4);  
     index = index + 4;             
     break;
   
 case ACCELERO_METER_PID: 
     /* Format B for Acc sensor */
     data_Length = 11;   
     *(sensor_Data+index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)ACCELERO_METER_PID ;
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)(ACCELERO_METER_PID >> 8);
     index++;
     memcpy(&sensor_Data[index],(void*)&acceleration.x,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&acceleration.y,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&acceleration.z,4);  
     index = index + 4;             
     break;
   
 case GYROSCOPE_PID: 
     
     /* Format B for Gyro sensor */
     data_Length = 11;   
     *(sensor_Data+index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)GYROSCOPE_PID ;
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)(GYROSCOPE_PID >> 8);
     index++;
     memcpy(&sensor_Data[index],(void*)&angular_velocity.x,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&angular_velocity.y,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&angular_velocity.z,4);  
     index = index + 4;             
     break;
   
 default:
   {
      /*(prop_Id_Temp & 0x07) << 5) | (Len <<1) Format A 
        Property calculation is done like above line
      */
     *(sensor_Data + index) = ((TEMPERATURE_PID & 0x07) << 5) | (data_Length <<1) ; 
     index++;
     *(sensor_Data+index) = (TEMPERATURE_PID >> 3) & 0xFF; 
     index++;
     memcpy(&sensor_Data[index],(void*)&temperature,4);
     index = index + 4;
     
      /* Format B for Pressure sensor */
     *(sensor_Data + index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data + index) = (MOBLEUINT8)PRESSURE_PID ;
     index++;
     *(sensor_Data + index) = (MOBLEUINT8)(PRESSURE_PID >> 8);
     index++;
     
     memcpy(&sensor_Data[index],(void*)&pressure,4);
     index = index + 4;
     
     
     /* Format B for Humidity sensor */
     *(sensor_Data+index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)HUMIDITY_PID ;
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)(HUMIDITY_PID >> 8);
     index++;
     
     memcpy(&sensor_Data[index],(void*)&humidity,4);
     index = index + 4;
     
     data_Length = 11;
     /* Format B for Acc sensor */
     *(sensor_Data+index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)ACCELERO_METER_PID ;
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)(ACCELERO_METER_PID >> 8);
     index++;
     memcpy(&sensor_Data[index],(void*)&acceleration.x,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&acceleration.y,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&acceleration.z,4);  
     index = index + 4;

      
     /* Format B for Gyro sensor */
     *(sensor_Data+index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)GYROSCOPE_PID ;
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)(GYROSCOPE_PID >> 8);
     index++;
     memcpy(&sensor_Data[index],(void*)&angular_velocity.x,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&angular_velocity.y,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&angular_velocity.z,4);  
     index = index + 4;
     
     
     /* Format B for Magnetic sensor */
     *(sensor_Data+index) = ((data_Length <<1) | 0x01); 
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)MAGNETO_METER_PID ;
     index++;
     *(sensor_Data+index) = (MOBLEUINT8)(MAGNETO_METER_PID >> 8);
     index++;
     memcpy(&sensor_Data[index],(void*)&magnetic_field.x,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&magnetic_field.y,4);  
     index = index + 4;
     
     memcpy(&sensor_Data[index],(void*)&magnetic_field.z,4);  
     index = index + 4;
     break;
   }
  }
  //*pLength  =13;    
  *pLength  = index;    
      
  return MOBLE_RESULT_SUCCESS;

}

/**
* @brief  Appli_Sensor_Descriptor_Status: This function is callback for Application
when sensor descriptor get message is received
* @param  sensor_Descriptor: Pointer to the parameters to be send in message
* @param  pLength: Length of the parameters to be sent in response
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Sensor_Descriptor_Status(MOBLEUINT8* sensor_Descriptor , MOBLEUINT32* pLength)
{
  Appli_Sensor_DescriptorStatus_t Appli_Sensor_DescriptorStatus1[] = {{PRESSURE_PID,0xABC,0xDEF,0x03,0x04,0x05},
                                                                       {TEMPERATURE_PID,0xc56,0xd78,0x06,0x07,0x08}};
    MOBLEUINT32 tolerance;
    tolerance = Appli_Sensor_DescriptorStatus1[0].NegativeTolerance;
    tolerance = (tolerance << 12 ) | Appli_Sensor_DescriptorStatus1[0].PositiveTolerance;
    
  *(sensor_Descriptor) = Appli_Sensor_DescriptorStatus1[0].Prop_ID;
  *(sensor_Descriptor+1) = Appli_Sensor_DescriptorStatus1[0].Prop_ID >> 8;
  *(sensor_Descriptor+2) = tolerance;
  *(sensor_Descriptor+3) = tolerance >> 8;
  *(sensor_Descriptor+4) = tolerance >> 16;
  *(sensor_Descriptor+5) = Appli_Sensor_DescriptorStatus1[0].SamplingFunction;
  *(sensor_Descriptor+6) = Appli_Sensor_DescriptorStatus1[0].MeasurementPeriod;
  *(sensor_Descriptor+7) = Appli_Sensor_DescriptorStatus1[0].UpdateInterval;
  
   tolerance = Appli_Sensor_DescriptorStatus1[1].NegativeTolerance;
   tolerance = (tolerance << 12 ) | Appli_Sensor_DescriptorStatus1[1].PositiveTolerance ;
    
  *(sensor_Descriptor+8) = Appli_Sensor_DescriptorStatus1[1].Prop_ID;
  *(sensor_Descriptor+9) = Appli_Sensor_DescriptorStatus1[1].Prop_ID >> 8;
  *(sensor_Descriptor+10) = tolerance;
  *(sensor_Descriptor+11) = tolerance >> 8;
  *(sensor_Descriptor+12) = tolerance >> 16;
  *(sensor_Descriptor+13) = Appli_Sensor_DescriptorStatus1[1].SamplingFunction;
  *(sensor_Descriptor+14) = Appli_Sensor_DescriptorStatus1[1].MeasurementPeriod;
  *(sensor_Descriptor+15) = Appli_Sensor_DescriptorStatus1[1].UpdateInterval;
  
  *pLength = 18;
   
  return MOBLE_RESULT_SUCCESS;
}

#endif

/**
* @brief  Sensor Process function
* @param  Function will continuously monitor the sensors.
Function used for the Publishing, data monitoring..
* @retval void
*/ 
void Sensor_Process(void)
{  
  
#ifdef ENABLE_SENSOR_PUBLICATION    
  float sensorValue[NUMBER_OF_SENSOR];
  if(ProvisionFlag == 1)
  {
    Read_Sensor_Data(&sensorValue[0]);
    Sensor_Publication_Process(&sensorValue[0], &Property_ID_Table[0]);
  }
#endif

/* Occupancy_Flag become True when ever sensor detect occupancy and get interrupt
   and make flag True to run this routine.
*/ 
  if(Occupancy_Flag == TRUE) 
  {
    if(BlueNrg_waitPeriod(CONTROLLER_WAIT_TIME))
    {
/* publishing the command for LC Light occupancy set message in the sensor status 
   message .
*/     
      Sensor_LC_Light_Publish();  
      Occupancy_Flag = FALSE;   
    }
  
  }
   
}

/**
* @brief  Function read the particular sensor value which are called inside.
* @param  pSensorValue: pointer to the sensor data array.       
* @retval void
*/ 
void Read_Sensor_Data(float *pSensorValue)
{
  float temp,press;
  static axis1bit32_t data_raw_pressure;
  HTS221_Get_Temperature_In_Float(0,&temp);
  
  pSensorValue[0] = temp;
  lps22hh_pressure_raw_get(0, data_raw_pressure.u8bit);
  press = LPS22HH_FROM_LSB_TO_hPa(data_raw_pressure.i32bit);
   
  pSensorValue[1] = press;
}

/**
* @brief  Function check for the couupancy in the location and send the status
  message with the ocuppancy value, when the interrupt is detected.
* @param  void     
* @retval void
*/  
void Sensor_LC_Light_Publish(void)
{
  MOBLEUINT8 occupancyData = 0x1;
  MOBLEUINT8 sensor_Data[5];
  MOBLE_ADDRESS srcAdd;
  MOBLE_RESULT result = MOBLE_RESULT_SUCCESS;
  
  sensor_Data[1] = (MOBLEUINT8)(LIGHT_CONTROL_LIGHTNESS_ON_ID << 8);
  sensor_Data[0] = (MOBLEUINT8)LIGHT_CONTROL_LIGHTNESS_ON_ID;
  sensor_Data[2] = occupancyData;
  
  srcAdd = BluenrgMesh_GetAddress();
  
  result = BluenrgMesh_SetRemotePublication(LIGHT_MODEL_SERVER_LC_MODEL_ID, srcAdd ,
                            SENSOR_STATUS , 
                            sensor_Data,3,
                            MOBLE_FALSE, MOBLE_FALSE);
   
  if(result)
  {
    TRACE_M(TF_LIGHT_LC,"Publication Error \r\n");
  }
   
}

#if defined ENABLE_SENSOR_PUBLICATION && defined ENABLE_SENSOR_MODEL_SERVER
/**
* @brief  Sensor Publication Process function
* @param  Function will publish the sensor data according to the given conditions.
* @param  void type function pointer.
* @param  pSensorData: Pointer to the sensor data array.
* @param  pProp_ID: Pointer to the Property id of sensor array.
* @retval void
*/
void Sensor_Publication_Process(float* pSensorData, MODEL_Property_IDTableParam_t* pProp_ID)
{
  static MOBLEUINT8 sensor_Count = 0;
  displayFloatToInt_t out_value;
  MOBLEUINT8 devisorValue;
  MOBLEUINT32 publishTime;
  static MOBLEUINT32 cadenceDurationTick[NUMBER_OF_SENSOR];
  static float previousDataValue[NUMBER_OF_SENSOR];
  static PublishingDataFlag_t PublishingDataFlag[NUMBER_OF_SENSOR] = {MOBLE_FALSE};
  
      floatToInt(pSensorData[sensor_Count], &out_value, 2);
      
      /* Taking the timestamp for the cadence publication and making flag high */
      if(PublishingDataFlag[sensor_Count].CadenceDurationFlag == MOBLE_FALSE)
      {
        cadenceDurationTick[sensor_Count] = Clock_Time();
        PublishingDataFlag[sensor_Count].CadenceDurationFlag = MOBLE_TRUE;          
      }
      /* Taking the sensor value and store it for comparing present sensor value with
      particular difference of increasing or decreasing. and making flag high.
 */
      if(PublishingDataFlag[sensor_Count].DeltaDataFlag == MOBLE_FALSE)
      {
        previousDataValue[sensor_Count] = pSensorData[sensor_Count];
        PublishingDataFlag[sensor_Count].DeltaDataFlag = MOBLE_TRUE;
      }
      /*
      This condition is checking for the difference of present sensor value 
      with prestored sensor value with user defined difference,if this condition 
      is true then it publish the sensor data.And making the delta flag low again.
      */       
      if((pSensorData[sensor_Count] >= (previousDataValue[sensor_Count] + Sensor_CadenceSet[sensor_Count].triggerDeltaUp)) ||
         (pSensorData[sensor_Count] <= (previousDataValue[sensor_Count] - Sensor_CadenceSet[sensor_Count].triggerDeltaDown)))
      {        
        SensorDataPublish((MOBLEUINT32*)&pSensorData[sensor_Count] , &pProp_ID[sensor_Count].Property_ID);
        
        PublishingDataFlag[sensor_Count].DeltaDataFlag = MOBLE_FALSE;
        TRACE_M(TF_SENSOR,"previous value data %.3f \r\n",previousDataValue[sensor_Count]);
        TRACE_M(TF_SENSOR,"Delta publication of data %.3f\r\n",*((float*)&pSensorData[sensor_Count]));
        sensor_Count++;
      }
      /*
      This condition is continuously checking the sensor value range, if that 
      value is within the user defined range then publishing duration or rate will
      be divided by user definedcadence devisor value and rate of publishing will 
      become high.And making the cadence flag low again.
      */
      if(((out_value.out_int <= Sensor_CadenceSet[sensor_Count].FastCadenceHigh) && 
          (out_value.out_int >= Sensor_CadenceSet[sensor_Count].FastCadenceLow)) ||
         (Sensor_CadenceSet[sensor_Count].FastCadenceHigh < Sensor_CadenceSet[sensor_Count].FastCadenceLow))
      {
        devisorValue = (MOBLEUINT8)pow(2 ,Sensor_CadenceSet[sensor_Count].FastCadenceDevisor);
        publishTime = SENSOR_PUBLISH_PERIOD/devisorValue;  
        
        if(((Clock_Time()- cadenceDurationTick[sensor_Count]) >= publishTime))
        {                   
          SensorDataPublish((MOBLEUINT32*)&pSensorData[sensor_Count] , &pProp_ID[sensor_Count].Property_ID);                 
          PublishingDataFlag[sensor_Count].CadenceDurationFlag = MOBLE_FALSE;           
          TRACE_M(TF_SENSOR,"Cadence publication of data %.2f \r\n",*((float*)&pSensorData[sensor_Count])); 
          sensor_Count++;
        } 
      }
      else
      {
        publishTime = SENSOR_PUBLISH_PERIOD ;     
        
        if(((Clock_Time()- cadenceDurationTick[sensor_Count]) >= SENSOR_PUBLISH_PERIOD))
        {            
          SensorDataPublish((MOBLEUINT32*)&pSensorData[sensor_Count] , &pProp_ID[sensor_Count].Property_ID);
          
          PublishingDataFlag[sensor_Count].CadenceDurationFlag = MOBLE_FALSE;                
          TRACE_M(TF_SENSOR,"Regular publication of data %.3f \r\n",*((float*)&pSensorData[sensor_Count]));
        }  
      } 
     if(sensor_Count > 1)
     {
       sensor_Count = 0;
     }

}

/**
* @brief  BluenrgMesh Sensor Publication function
* @param  Function will decide the publish address and element.
* @param  pSensor_Value: Pointer to the sensor data array 
* @param  pProp_ID: pointer to the property id of sensor array.
* @retval void
*/
void SensorDataPublish(MOBLEUINT32 *pSensor_Value , MOBLEUINT16* pProp_ID)
{
  MOBLEUINT32 length;
  MOBLEUINT8 sensor_Data[8];
  MOBLE_ADDRESS srcAdd;
  MOBLE_RESULT result = MOBLE_RESULT_SUCCESS;
  
  srcAdd = BluenrgMesh_GetAddress();
  
  switch(*pProp_ID)
  {
  case TEMPERATURE_PID:
    { 
      /*(prop_Id_Temp & 0x07) << 5) | (Len <<1) Format A 
      Property calculation is done like above line
 */
      sensor_Data[0] = ((TEMPERATURE_PID & 0x07) << 5) | (3 <<1) ; 
      sensor_Data[1] = (TEMPERATURE_PID >> 3) & 0xFF; 
      
      memcpy(&sensor_Data[2],(void*)&pSensor_Value[0],4);
      length  =6;  
      break;
    }
  case PRESSURE_PID:
    {
      /* Format B for Pressure sensor */
      sensor_Data[0] = ((0x03 <<1) | 0x01); 
      sensor_Data[1] = (MOBLEUINT8)PRESSURE_PID ;
      sensor_Data[2] = (MOBLEUINT8)(PRESSURE_PID >> 8);
      
      memcpy(&sensor_Data[3],(void*)&pSensor_Value[0],4);  
      length  =7;     
      break;
    }
  default:
    break;
  }
  

  result = BluenrgMesh_SetRemotePublication(SENSOR_SERVER_MODEL_ID, srcAdd,
                            SENSOR_STATUS , 
                            sensor_Data,length,
                            MOBLE_FALSE, MOBLE_FALSE);
  
  if(result)
  {
    TRACE_M(TF_SENSOR,"Publication Error \r\n");
  }
  
}

#endif

#ifdef ENABLE_SENSOR_MODEL_SERVER
/**
* @brief  Appli_Sensor_GetSettingStatus: This function is callback for Application
when sensor setting numbers status message is to be provided
* @param  pSetting_Status: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Sensor_GetSettingStatus(MOBLEUINT8* pSetting_Status)                                        
{
  
  *pSetting_Status = Appli_Sensor_SettingSet.Property_ID;
  *(pSetting_Status+1) = Appli_Sensor_SettingSet.Property_ID >> 8;
  *(pSetting_Status+2) = Appli_Sensor_SettingSet.Sensor_Setting_ID; 
  *(pSetting_Status+3) = Appli_Sensor_SettingSet.Sensor_Setting_ID >> 8;
  
  return MOBLE_RESULT_SUCCESS; 
}

/**
* @brief  Appli_Sensor_GetSetting_IDStatus: This function is callback for Application
when sensor setting numbers and row value status message is to be provided
* @param  pSetting_Status: Pointer to the status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Sensor_GetSetting_IDStatus(MOBLEUINT8* pSetting_Status)                                        
{
  
  *pSetting_Status = Appli_Sensor_SettingSet.Property_ID;
  *(pSetting_Status+1) = Appli_Sensor_SettingSet.Property_ID >> 8;
  *(pSetting_Status+2) = Appli_Sensor_SettingSet.Sensor_Setting_ID; 
  *(pSetting_Status+3) = Appli_Sensor_SettingSet.Sensor_Setting_ID >> 8; 
  *(pSetting_Status+4) = Appli_Sensor_SettingSet.Sensor_Setting_Access; 
  *(pSetting_Status+5) = Appli_Sensor_SettingSet.Sensor_Setting_Value;
  *(pSetting_Status+6) = Appli_Sensor_SettingSet.Sensor_Setting_Value >> 8;
  
  return MOBLE_RESULT_SUCCESS; 
}

#endif

/**
* @brief  Check_Property_ID: This function is used for checking the Property id 
of sensor available in table.
* @param  prop_ID_Table: address of the property id table array.
* @param  prop_ID:received property id of sensor.
* @retval MOBLE_RESULT
*/
MOBLE_RESULT Check_Property_ID(const MODEL_Property_IDTableParam_t prop_ID_Table[] 
                               , MOBLEUINT16 prop_ID)
{
  
  for(int i=0;i<NUMBER_OF_SENSOR;i++)
  {
    if(prop_ID_Table[i].Property_ID != prop_ID)
    {       
      return MOBLE_RESULT_FALSE;
    }
  }     
  
  return MOBLE_RESULT_SUCCESS;
  
}

/**
* @brief  Appli_Sensor_Init: This function is callback for Initialisation of 
Application interface
* @void  No input parameter 
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_Sensor_Init(void)                                        
{
  GPIO_EXTIConfigType GPIO_EXTIStructure;
  GPIO_InitType GPIO_InitStructure;  
#if LSM6DSO_ENABLED
  GPIO_EXTIStructure.GPIO_Pin = SDK_EVAL_IRQ_SENSOR_PIN;
  GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Level;
  GPIO_EXTIStructure.GPIO_Event = GPIO_Event_High;
  GPIO_EXTIConfig( &GPIO_EXTIStructure);
  
  /* Disable interrupt on SENSOR INT pin */
  GPIO_EXTICmd(SDK_EVAL_IRQ_SENSOR_PIN, DISABLE);
  
  /* Clear GPIO pending interrupt on SWD_CLK and SDK_EVAL_IRQ_SENSOR_PIN */
  GPIO_ClearITPendingBit(SWD_CLK_PIN | SDK_EVAL_IRQ_SENSOR_PIN);

#else
  /* Clear GPIO pending interrupt on SWD_CLK */
  GPIO_ClearITPendingBit(SWD_CLK_PIN);
#endif
  
  /* Enable interrupt on WD_CLK*/
  GPIO_EXTICmd(SWD_CLK_PIN, ENABLE);

#if USER_DEFINED_PLATFORM == USER_EVAL_PLATFORM
  /* Configure I2C @ 400 kHz */
  SdkEvalI2CInit(400000);
  
  /** Init Structure */
  GPIO_StructInit(&GPIO_InitStructure);
  /** Configure GPIO_Pin_7 for Proximity Sensor XSHUT */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init(&GPIO_InitStructure);
  
  /* Turn OFF Proximity Sensor */
  GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);

  /* Configure Sensor Common INT pin */
  GPIO_InitStructure.GPIO_Mode = GPIO_Input;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_InitStructure.GPIO_Pin = SDK_EVAL_IRQ_SENSOR_PIN;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  
#ifdef CUBE_EXT_LED
  GPIO_InitStructure.GPIO_Pin = SDK_EVAL_UART_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  
  /* Turn On the external LED for the CUBE */
  GPIO_WriteBit(SDK_EVAL_UART_TX_PIN, Bit_RESET);
#endif
  
#ifdef LPS22HH_ENABLED
Init_Pressure_Temperature_Sensor();
#endif

#ifdef HTS221_ENABLED
  Init_Humidity_Sensor();
#endif
  
#ifdef LSM6DSO_ENABLED
Init_Accelerometer_Gyroscope();
/* Enable interrupt on SENSOR INT pin */
GPIO_EXTICmd(SDK_EVAL_IRQ_SENSOR_PIN, ENABLE);
#endif

#ifdef VL53L1_ENABLED
  Init_Proximity_Sensor();
  /* Enable interrupt on SENSOR INT pin */
  GPIO_EXTICmd(SDK_EVAL_IRQ_SENSOR_PIN, ENABLE);
  lis2mdl_operating_mode_set(0, LIS2MDL_POWER_DOWN);
#endif
  
#endif  
  return MOBLE_RESULT_SUCCESS; 
}

#ifdef HTS221_ENABLED
/**
 * @brief Init HTS221 temperature sensor.
 * @param  None
 * @retval None
 */
void Init_Humidity_Sensor(void)
{
    HTS221_Set_AvgH(0, HTS221_AVGH_4);
    HTS221_Set_AvgT(0, HTS221_AVGT_2);
    HTS221_Set_BduMode(0, HTS221_ENABLE);
    HTS221_Set_Odr(0, HTS221_ODR_1HZ);
    HTS221_Set_HeaterState(0, HTS221_DISABLE);

    HTS221_Set_IrqOutputType(0, HTS221_OPENDRAIN);
    HTS221_Set_IrqActiveLevel(0, HTS221_LOW_LVL);

    HTS221_Set_PowerDownMode(0, HTS221_RESET);
    HTS221_Activate(0);
}
#endif /* HTS221_ENABLED */

#ifdef LPS22HH_ENABLED
/**
 * @brief  Init LPS22HB pressure and temperature sensor.
 * @param  None
 * @retval None
 */
void Init_Pressure_Temperature_Sensor(void) 
{
  uint8_t who_am_I_8 = 0x00;
  
  lps22hh_device_id_get(0, &who_am_I_8); // 0xB3
  lps22hh_i3c_interface_set(0, LPS22HH_I3C_DISABLE);
  lps22hh_block_data_update_set(0, PROPERTY_ENABLE);
  lps22hh_data_rate_set(0, LPS22HH_10_Hz_LOW_NOISE);
  lps22hh_pin_mode_set(0, LPS22HH_OPEN_DRAIN);
  lps22hh_pin_polarity_set(0, LPS22HH_ACTIVE_LOW);
  lps22hh_int_notification_set(0, LPS22HH_INT_LATCHED);
  lps22hh_int_pd_set(0, LPS22HH_PULL_DOWN_DISCONNECT);
}
#endif /* LPS22HH_ENABLED */



#ifdef LSM6DS3_ENABLE
IMU_6AXES_DrvTypeDef *Imu6AxesDrv = NULL;
LSM6DS3_DrvExtTypeDef *Imu6AxesDrvExt = NULL;
IMU_6AXES_StatusTypeDef AccStatus;
/**
* @brief  Accelerometer Initialization fuction   
* @param  void
* @retval void
*/
void Init_Accelerometer(void)
{
  /* LSM6DS3 library setting */
  IMU_6AXES_InitTypeDef InitStructure;
  AxesRaw_t axes; 
  uint8_t tmp1 = 0x00;
  
  Imu6AxesDrv = &LSM6DS3Drv;
  Imu6AxesDrvExt = &LSM6DS3Drv_ext_internal;
  InitStructure.G_FullScale      = 125.0f;
  InitStructure.G_OutputDataRate = 13.0f;
  InitStructure.G_X_Axis         = 1;
  InitStructure.G_Y_Axis         = 1;
  InitStructure.G_Z_Axis         = 1;
  InitStructure.X_FullScale      = 2.0f;
  InitStructure.X_OutputDataRate = 13.0f;
  InitStructure.X_X_Axis         = 1;
  InitStructure.X_Y_Axis         = 1;
  InitStructure.X_Z_Axis         = 1;  

  /* LSM6DS3 initiliazation */
  AccStatus = Imu6AxesDrv->Init(&InitStructure);

  /* Disable all mems IRQs */
  LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_INT1_CTRL, 1);
  
  /* Clear first previous data */
  Imu6AxesDrv->Get_X_Axes((int32_t *)&axes);
  Imu6AxesDrv->Get_G_Axes((int32_t *)&axes);
}

IMU_6AXES_StatusTypeDef GetAccAxesRaw(AxesRaw_t * acceleration_data, AxesRaw_t * gyro_data)
{
  IMU_6AXES_StatusTypeDef status = IMU_6AXES_OK;

  status = Imu6AxesDrv->Get_X_Axes((int32_t *)acceleration_data);
  status |= Imu6AxesDrv->Get_G_Axes((int32_t *)gyro_data);

  return status;
}
#endif



#ifdef LSM6DSO_ENABLED

/**
 * @brief  Configure the sensors low-power mode.
 * @param  None
 * @retval None
 */
void SensorsLowPower(void) 
{    
//    lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_OFF);
//    lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_OFF);
//    lps22hh_data_rate_set(0, LPS22HH_POWER_DOWN);

    lis2mdl_operating_mode_set(0, LIS2MDL_POWER_DOWN);
    HTS221_Set_PowerDownMode(0,HTS221_SET);
}

/**
 * @brief  This function disables all the HW's Features
 * @param  None
 * @retval None
 */
void DisableHWFeatures(void) 
{
  lsm6dso_pin_int1_route_get(0, &int1_route);
  int1_route.reg.md1_cfg.int1_6d = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_double_tap = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_emb_func = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_ff = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_single_tap = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_sleep_change = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_wu = PROPERTY_DISABLE;
  lsm6dso_pin_int1_route_set(0, &int1_route);
}

/**
 * @brief  This function enables the HW's Wake Up Detection
 * @param  None
 * @retval None
 */
void EnableHWWakeUp(void) 
{
  lsm6dso_xl_hp_path_internal_set(0, LSM6DSO_USE_HPF); //LSM6DSO_USE_SLOPE);
  lsm6dso_wkup_threshold_set(0, 0x04);
  lsm6dso_wkup_dur_set(0, 0x1B);
  lsm6dso_pin_int1_route_get(0, &int1_route);
  int1_route.reg.md1_cfg.int1_wu = PROPERTY_ENABLE;
  lsm6dso_pin_int1_route_set(0, &int1_route);
}


/**
 * @brief  This function enables the HW's Free Fall Detection
 * @param  None
 * @retval None
 */
void EnableHWFreeFall(void) 
{
  lsm6dso_ff_dur_set(0, 0x02);
  lsm6dso_ff_threshold_set(0, LSM6DSO_FF_TSH_406mg);
  lsm6dso_pin_int1_route_get(0, &int1_route);
  int1_route.reg.md1_cfg.int1_ff = PROPERTY_ENABLE;
  lsm6dso_pin_int1_route_set(0, &int1_route);
}

/**
 * @brief  This function enables the HW's Single Tap Detection
 * @param  None
 * @retval None
 */
void EnableHWSingleTap(void) 
{
  lsm6dso_tap_detection_on_z_set(0, PROPERTY_ENABLE);
  lsm6dso_tap_detection_on_y_set(0, PROPERTY_DISABLE); //PROPERTY_ENABLE);
  lsm6dso_tap_detection_on_x_set(0, PROPERTY_DISABLE); //PROPERTY_ENABLE);
  lsm6dso_tap_threshold_x_set(0, 0x0C); //0x04);
  lsm6dso_tap_threshold_y_set(0, 0x0C); //0x04);
  lsm6dso_tap_threshold_z_set(0, 0x08); //0x04);
  lsm6dso_tap_dur_set(0, 0x01);
  lsm6dso_tap_quiet_set(0, 0x01);
  lsm6dso_tap_shock_set(0, 0x08);
  lsm6dso_tap_mode_set(0, LSM6DSO_ONLY_SINGLE);
  lsm6dso_pin_int1_route_get(0, &int1_route);
  //	int1_route.reg.md1_cfg.int1_wu = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_double_tap = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_single_tap = PROPERTY_ENABLE;
  lsm6dso_pin_int1_route_set(0, &int1_route);
  
  lsm6dso_act_mode_set(0, LSM6DSO_XL_AND_GY_NOT_AFFECTED);
}

/**
 * @brief  This function enables the HW's Double Tap Detection
 * @param  None
 * @retval None
 */
void EnableHWDoubleTap(void)
{
  lsm6dso_tap_detection_on_z_set(0, PROPERTY_ENABLE);
  lsm6dso_tap_detection_on_y_set(0, PROPERTY_DISABLE); //PROPERTY_ENABLE);
  lsm6dso_tap_detection_on_x_set(0, PROPERTY_DISABLE); //PROPERTY_ENABLE);
  lsm6dso_tap_threshold_x_set(0, 0x0C); //0x04);
  lsm6dso_tap_threshold_y_set(0, 0x0C); //0x04);
  lsm6dso_tap_threshold_z_set(0, 0x08); //0x04);
  lsm6dso_tap_dur_set(0, 0x01);
  lsm6dso_tap_quiet_set(0, 0x01);
  lsm6dso_tap_shock_set(0, 0x08);
  lsm6dso_tap_mode_set(0, LSM6DSO_BOTH_SINGLE_DOUBLE);
  lsm6dso_pin_int1_route_get(0, &int1_route);
  //	int1_route.reg.md1_cfg.int1_wu = PROPERTY_DISABLE;
  int1_route.reg.md1_cfg.int1_double_tap = PROPERTY_ENABLE;    
  //	int1_route.reg.md1_cfg.int1_single_tap = PROPERTY_ENABLE;
  lsm6dso_pin_int1_route_set(0, &int1_route);
  
  lsm6dso_act_mode_set(0, LSM6DSO_XL_12Hz5_GY_PD); 
  //LSM6DSO_XL_AND_GY_NOT_AFFECTED=0, LSM6DSO_XL_12Hz5_GY_NOT_AFFECTED=1, LSM6DSO_XL_12Hz5_GY_SLEEP=2, LSM6DSO_XL_12Hz5_GY_PD=3
}

/**
 * @brief  Init LSM6DSL accelerometer/gyroscope.
 * @param  None
 * @retval None
 */
void Init_Accelerometer_Gyroscope(void) 
{
  static uint8_t rst;
  
  lsm6dso_i3c_disable_set(0, LSM6DSO_I3C_DISABLE);
  
  rst = lsm6dso_reset_set(0, PROPERTY_ENABLE);
  do {
    lsm6dso_reset_get(0, &rst);
  } while (rst);
  
  lsm6dso_pin_mode_set(0, LSM6DSO_PUSH_PULL);
  lsm6dso_pin_polarity_set(0, LSM6DSO_ACTIVE_HIGH);
  lsm6dso_all_on_int1_set(0, PROPERTY_ENABLE);
  lsm6dso_int_notification_set(0, LSM6DSO_ALL_INT_LATCHED);
  
  lsm6dso_block_data_update_set(0, PROPERTY_ENABLE);
  lsm6dso_xl_power_mode_set(0, LSM6DSO_LOW_NORMAL_POWER_MD);
  lsm6dso_gy_power_mode_set(0, LSM6DSO_GY_NORMAL);
  lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_104Hz); //LSM6DSO_XL_ODR_52Hz);
  //lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_OFF); //LSM6DSO_GY_ODR_52Hz);        
  lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_52Hz); 
  
  lsm6dso_xl_full_scale_set(0, LSM6DSO_4g); //LSM6DSO_2g);
  lsm6dso_gy_full_scale_set(0, LSM6DSO_2000dps);
  
  lsm6dso_auto_increment_set(0, PROPERTY_ENABLE);
  
  Init_Humidity_Sensor(); //Needed to set the HTS221 INT pin to OpenDrain (Default is PushPull and prevents the Axel use the INT pin)
  //        Init_Magnetometer();
  SensorsLowPower();
  
  DisableHWFeatures();
  EnableHWFreeFall();
  EnableHWWakeUp();
  EnableHWSingleTap();
  // EnableHWDoubleTap();
}

/**
 * @brief  Read and Reset the Latched interrupts from MEMS
 * @param  None
 * @retval None
 */ 
void MEMS_LatchIrqReset(void)
{
  lsm6dso_all_sources_get(0, &all_source);
}

/**
 * @brief  Check the HW Feature Events when there is a interrupt from MEMS
 * @param  None
 * @retval None
 */ 

void MEMSCallback(void)
{
  MOBLE_ADDRESS srcAdd;
  
  /*Select the Element Number for which publication address is required*/
  
  srcAdd = BluenrgMesh_GetAddress();  
  
  lsm6dso_all_sources_get(0, &all_source);
  
  /* Check if the interrupt is due to Free Fall */
  if (all_source.reg.all_int_src.ff_ia)
  {
    //Publish OnOff Message to Group 3: Cube N3;
    Vendor_Publish(srcAdd); //White LED
    lsm6dso_all_sources_get(0, &all_source);
    lsm6dso_all_sources_get(0, &all_source);
    lsm6dso_all_sources_get(0, &all_source);
  }
  
  /* Check if the interrupt is due to Double Tap */
  else if (all_source.reg.all_int_src.double_tap)
  {    
    //Publish OnOff Message to Group 2: Cube N2;
    Vendor_Publish(srcAdd);
  }
  
  /* Check if the interrupt is due to Single Tap */
  else if (all_source.reg.all_int_src.single_tap)
  {    
    //Publish OnOff Message to Group 2: Cube N1;   
    Vendor_Publish(srcAdd);
  }
  
  /* Check if the interrupt is due to Wake Up */
  else if (all_source.reg.all_int_src.wu_ia)
  {  
    //Publish OnOff Message to Group 1: Cube N1;
    Vendor_Publish(srcAdd);
  }
}

#endif /* LSM6DSO_ENABLED */
/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

