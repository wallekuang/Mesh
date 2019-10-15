/**
******************************************************************************
* @file    models_if.c
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Mesh Models interface file of the application
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
#include "appli_mesh.h"
#include "vendor.h"
#include "light.h"
#include "sensors.h"
#include "generic.h"
#include "time_scene.h"
#include "common.h"
#include "appli_generic.h"
#include "appli_vendor.h"
#include "appli_light.h"
#include "appli_sensor.h"
#include "appli_nvm.h"
#include "bluenrg1_api.h"
#include "PWM_config.h"
#include "PWM_handlers.h"
#include "appli_light_lc.h"
#include "light_lc.h"
/** @addtogroup BlueNRG_Mesh
 *  @{
 */

/** @addtogroup models_BlueNRG1
 *  @{
 */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  MOBLE_ADDRESS peer;
  MOBLE_ADDRESS dst;
  MOBLEUINT8 command;
  MOBLEUINT8 data[DATA_BUFFER_LENGTH]; /* 8-Bytes response packet */
  MOBLEUINT32 length;
} APPLI_SEND_RESPONSE_MODULE;

#pragma pack(1)
typedef struct
{
  MOBLEUINT8 packet_count;
  MOBLEUINT32 send_time;
  APPLI_SEND_RESPONSE_MODULE* head;
  MOBLEUINT8 head_index;
  APPLI_SEND_RESPONSE_MODULE packet[MAX_PENDING_PACKETS_QUE_SIZE];
} APPLI_PENDING_PACKETS;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
   

/* Private variables ---------------------------------------------------------*/
APPLI_PENDING_PACKETS Appli_PendingPackets = {0};

ALIGN(4)
const MOBLE_VENDOR_CB_MAP vendor_cb = 
{
  Vendor_WriteLocalDataCb,
  Vendor_ReadLocalDataCb,
  Vendor_OnResponseDataCb
};

ALIGN(4)
const Appli_Vendor_cb_t VendorAppli_cb = 
{
  /*Vendor Commads*/
  Appli_Vendor_LEDControl,
  Appli_Vendor_DeviceInfo,
  Appli_Vendor_Test,
  Appli_LedCtrl,
  Appli_GetTestValue 
};

ALIGN(4)   
const Appli_Generic_cb_t GenericAppli_cb = 
{ 
  /* Generic OnOff callbacks */
  Appli_Generic_OnOff_Set,
 
 /* Generic Level callbacks */
  Appli_Generic_Level_Set,
  Appli_Generic_LevelDelta_Set,
  Appli_Generic_LevelMove_Set,
  
  /* Generic Power on off callbacks */
  Appli_Generic_PowerOnOff_Set,
    
  /* Generic Default transition time callbacks */  
  Appli_Generic_DefaultTransitionTime_Set
};

ALIGN(4)
const Appli_Generic_State_cb_t Appli_GenericState_cb =
{
  
 /* Generic Get On Off status */
  Appli_Generic_GetOnOffStatus, 
  Appli_Generic_GetOnOffValue,
 /* Generic Get level status */
  Appli_Generic_GetLevelStatus,
 /* Generic Get Power on off status */
  Appli_Generic_GetPowerOnOffStatus, 
 /* Generic Get Default transition time status */
  Appli_Generic_GetDefaultTransitionStatus
};

ALIGN(4)
const Appli_Light_GetStatus_cb_t Appli_Light_GetStatus_cb = 
{
  Appli_Light_GetLightnessStatus,
  Appli_Light_GetLightnessLinearStatus,
  Appli_Light_GetLightnessDefaultStatus,
  Appli_Light_GetLightnessRangeStatus, 
  Appli_Light_GetCtlLightStatus,
  Appli_Light_GetCtlTemperatureStatus,
  Appli_Light_GetCtlTemperatureRange,
  Appli_Light_GetCtlDefaultStatus,  
  Appli_Light_GetHslStatus,
  Appli_Light_GetHslHueStatus,
  Appli_Light_GetHslSaturationStatus,
  Appli_Light_GetHslHueRange,
  Appli_Light_GetHslSatRange
};


ALIGN(4)
const Appli_Light_cb_t LightAppli_cb = 
{ 
  /* Light Lightness callbacks */
  Appli_Light_Lightness_Set,
  Appli_Light_Lightness_Linear_Set,
  Appli_Light_Lightness_Default_Set,
  Appli_Light_Lightness_Range_Set,
  
  Appli_Light_Ctl_Set,
  Appli_Light_CtlTemperature_Set,
  Appli_Light_CtlTemperature_Range_Set,
  Appli_Light_CtlDefault_Set,
  
  Appli_Light_Hsl_Set,
  Appli_Light_HslHue_Set,
  Appli_Light_HslSaturation_Set,
  Appli_Light_HslDefault_Set,
  Appli_Light_HslRange_Set
};


ALIGN(4)
const Appli_Light_Ctrl_cb_t LightLCAppli_cb = 
{ 
  /* Light LC mode set callbacks */
  Appli_LightLC_Mode_Set,
  Appli_LightLC_OM_Set,
  Appli_LightLC_OnOff_Set,
};

ALIGN(4)
const Appli_LightLC_GetStatus_cb_t Appli_LightLC_GetStatus_cb = 
{
  Appli_LightLC_Get_ModeStatus,
  Appli_LightLC_Get_OMModeStatus,
  Appli_LightLC_Get_OnOffStatus,
  Appli_LightLC_Get_AmbientLuxLevelOutput,
  Appli_Light_LC_PIRegulatorOutput,
};

#ifdef ENABLE_SENSOR_MODEL_SERVER

ALIGN(4)
const Appli_Sensor_cb_t SensorAppli_cb = 
{
  /* Sensor Model callbacks */
  Appli_Sensor_Cadence_Set,
  Appli_Sensor_Data_Status,
  Appli_Sensor_Descriptor_Status ,
  Appli_Sensor_Setting_Set
};

ALIGN(4)
const Appli_Sensor_GetStatus_cb_t Appli_Sensor_GetStatus_cb = 
{
  Appli_Sensor_GetSettingStatus,
  Appli_Sensor_GetSetting_IDStatus
};

#endif


ALIGN(4)
const MODEL_SIG_cb_t Model_SIG_cb[] = 
{
  {
    GenericModelServer_GetOpcodeTableCb,
    GenericModelServer_GetStatusRequestCb,
    GenericModelServer_ProcessMessageCb
  },
#ifdef ENABLE_LIGHT_MODEL_SERVER
  {
    LightModelServer_GetOpcodeTableCb,
    LightModelServer_GetStatusRequestCb,
    LightModelServer_ProcessMessageCb
  },
#endif
#if defined(ENABLE_SENSOR_MODEL_SERVER) || defined(ENABLE_SENSOR_MODEL_SERVER_SETUP)
  {
    SensorModelServer_GetOpcodeTableCb,
    SensorModelServer_GetStatusRequestCb,
    SensorModelServer_ProcessMessageCb
  },
#endif
#if defined(ENABLE_TIME_MODEL_SERVER) || defined(ENABLE_SCENE_MODEL_SERVER)
  {
    Time_SceneModelServer_GetOpcodeTableCb,
    Time_SceneModelServer_GetStatusRequestCb,
    Time_SceneModelServer_ProcessMessageCb
  },
#endif
#if defined(ENABLE_LIGHT_MODEL_SERVER_LC) || defined(ENABLE_LIGHT_MODEL_SERVER_LC_SETUP)
  {
    Light_LC_ModelServer_GetOpcodeTableCb,
    Light_LC_ModelServer_GetStatusRequestCb,
    Light_LC_ModelServer_ProcessMessageCb
  },
#endif
  { 0, 0,0 }
};

ALIGN(4)
const APPLI_SAVE_MODEL_STATE_CB SaveModelState_cb = AppliNvm_SaveModelState;

#define MODEL_SIG_COUNT ( ( sizeof(Model_SIG_cb)/sizeof(Model_SIG_cb[0]) - 1 ))

ALIGN(4)
const MODEL_Vendor_cb_t Model_Vendor_cb[] = 
{
  {
    VendorModel_PID1_GetOpcodeTableCb,
    VendorModel_PID1_GetStatusRequestCb,
    VendorModel_PID1_ProcessMessageCb
  },
  { 0, 0,0 }
};

#define MODEL_VENDOR_COUNT ( ( sizeof(Model_Vendor_cb)/sizeof(Model_Vendor_cb[0]) - 1 )) 

extern MOBLEUINT8 NumberOfElements;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void GetApplicationVendorModels(const MODEL_Vendor_cb_t** pModelsTable, MOBLEUINT32* VendorModelscount)
{
  *pModelsTable = Model_Vendor_cb       ;
  *VendorModelscount = MODEL_VENDOR_COUNT;
  
   //TRACE_M(TF_VENDOR,"GetApplicationVendorModels \r\n");
}

/**
* @brief  Initialization Commands for Models
* @param  void
* @retval void
*/    
void BluenrgMesh_ModelsInit(void)
{
  MOBLEUINT8 modelStateLoad_Size;
  MOBLEUINT8 modelStateLoadBuff[APP_NVM_MODEL_SIZE];    
  
  /* Callbacks used by BlueNRG-Mesh Models */
  BluenrgMesh_SetSIGModelsCbMap(Model_SIG_cb, MODEL_SIG_COUNT);
  
  /* Load generic model states from nvm */
  AppliNvm_LoadModelState(modelStateLoadBuff, &modelStateLoad_Size);
  if (modelStateLoad_Size != 0)
  {
    /* update states of generic model */
    Model_RestoreStates(modelStateLoadBuff, modelStateLoad_Size);
  }
  
#if defined ENABLE_SENSOR_MODEL_SERVER && !defined CUSTOM_BOARD_PWM_SELECTION  
  /* Initiallization of sensors */
  Appli_Sensor_Init();
#endif  
}

/**
* @brief  Process Commands for Models
* @param  void
* @retval void
*/    
void BluenrgMesh_ModelsProcess(void)
{
  Generic_Process();
  Lighting_Process();
  Vendor_Process();
/* Define this Macro to enable the publication of sensors data.*/ 
#if defined ENABLE_SENSOR_MODEL_SERVER 
  Sensor_Process();
#endif

#ifdef ENABLE_APPLI_TEST  
  Test_Process();
#endif   
  ModelSave_Process();
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_LC   
  Light_control_Process();
#endif
}


/**
* @brief  Publish Command for Models
* @param  void
* @retval void
*/    
void BluenrgMesh_ModelsCommand(void)
{
  MOBLE_ADDRESS srcAdd = BluenrgMesh_GetAddress();
  
#ifdef VENDOR_MODEL_PUBLISH   
  	//Vendor_Publish(srcAdd);
  	app_control_test();
#else
  Generic_Publish(srcAdd);
#endif  
}

/**
* @brief  Get the Element Number for selected Model 
* @param  dst_peer : Destination Address received
* @retval MOBLEUINT8 : elementIndex
*/ 
MOBLEUINT8 BluenrgMesh_ModelsGetElementNumber(MOBLE_ADDRESS dst_peer)
{
  
  MOBLE_ADDRESS nodeAddress;
  MOBLEUINT8 elementNumber;
  
  nodeAddress = BluenrgMesh_GetAddress();
  elementNumber = ((dst_peer - nodeAddress)+1);
  
  return elementNumber;
}

/**
* @brief  Check Subscription of Elements for Group Address for selected Model 
* @param  dst_peer : Destination Address received
* @param  elementNumber : Number of element to check Subscription
* @retval MOBLE_RESULT status of result
*/ 
MOBLE_RESULT BluenrgMesh_ModelsCheckSubscription(MOBLE_ADDRESS dst_peer, \
                                                        MOBLEUINT8 elementNumber)
{
  MOBLE_RESULT status = MOBLE_RESULT_FAIL;
  MOBLE_ADDRESS subscriptionList[10] = {0};
  MOBLEUINT8 length;
  MOBLEUINT32 modelId = GENERIC_MODEL_SERVER_LEVEL_MODEL_ID;
  BluenrgMesh_GetSubscriptionAddress(subscriptionList,&length,elementNumber, modelId);
  
  
  for(uint8_t list=0; list<length; list++)
  {
      if(dst_peer == subscriptionList[list])
      {
      status = MOBLE_RESULT_SUCCESS;    
      break;
      }
  }
  
  return status;
}

/**
* @brief  Schedule a packet to be sent with randomized send timestamp  
*         If a que is empty, random timestamp is calculated
*         Subsequent packets are sent in sequence
* @param  peer:    Address of the peer
* @param  dst :    Address of the node
* @param  status:  Command status
* @param  data:    Data buffer.
* @param  length:  Length of data in bytes.
* @retval None
*/ 
void BluenrgMesh_ModelsDelayPacket(MOBLE_ADDRESS peer, 
                              MOBLE_ADDRESS dst,
                              MOBLEUINT8 command, 
                              MOBLEUINT8 const * data, 
                              MOBLEUINT32 length)
{
  MOBLEUINT8 random_time[8];
  
  if (Appli_PendingPackets.packet_count == 0)
  {
    Appli_PendingPackets.packet_count = 1;
    hci_le_rand(random_time);
    Appli_PendingPackets.send_time = Clock_Time() + 
                                     DEFAULT_DELAY_PACKET_FROM + 
                                     (random_time[0] + random_time[1]*256)\
                                         %DEFAULT_DELAY_PACKET_RANDOM_TIME;    
    Appli_PendingPackets.head = Appli_PendingPackets.packet;
    Appli_PendingPackets.head_index = 0;	
    TRACE_M(TF_MISC, "Randomized time: %d\n\r", Appli_PendingPackets.send_time - Clock_Time());	
  }
  else 
  {
    Appli_PendingPackets.packet_count += 1;
    Appli_PendingPackets.packet_count = (Appli_PendingPackets.packet_count)%\
                                              (MAX_PENDING_PACKETS_QUE_SIZE+1);

    if (Appli_PendingPackets.head != (Appli_PendingPackets.packet + \
                                      MAX_PENDING_PACKETS_QUE_SIZE - 1))
    {
      Appli_PendingPackets.head = Appli_PendingPackets.head +1;
      Appli_PendingPackets.head_index = Appli_PendingPackets.head_index+1;
    }
    else
    {
      Appli_PendingPackets.head = Appli_PendingPackets.packet;
      Appli_PendingPackets.head_index = 0;
    }
  }  
  
  Appli_PendingPackets.head->peer = peer;
  Appli_PendingPackets.head->dst = dst;
  Appli_PendingPackets.head->command = command;
  Appli_PendingPackets.head->length = length;
  for (MOBLEUINT8 count=0; count<length; count++)
  Appli_PendingPackets.head->data[count] = data[count];
}   
  

/**
* @brief  If send timestamp is reached and que is not empty, send all packets
* @param  None
* @retval None
*/
void BluenrgMesh_ModelsSendDelayedPacket(void)
{
  APPLI_SEND_RESPONSE_MODULE* ptr;
  MOBLEUINT8 temp_index;
  
  if ((Appli_PendingPackets.packet_count != 0) && 
      (Appli_PendingPackets.send_time <= Clock_Time()))
  {
    for (MOBLEUINT8 count=Appli_PendingPackets.packet_count; count!=0; count--)
    {	
    TRACE_M(TF_MISC, "Sending randomized packets. Packet count: %d \n\r",\
    Appli_PendingPackets.packet_count - count + 1);   
      temp_index = ((Appli_PendingPackets.head_index+MAX_PENDING_PACKETS_QUE_SIZE+1)\
                                   -count)%MAX_PENDING_PACKETS_QUE_SIZE;
      ptr = Appli_PendingPackets.packet + temp_index;
      
      VendorModel_SendResponse(VENDOR_STMICRO_CID, 
                               ptr->peer,
                               ptr->dst,
                               ptr->command,
                               ptr->data,
                               ptr->length);
    }
    
    Appli_PendingPackets.packet_count = 0;
  }
}


/**
* @brief  Convert ASCII value into Character
* @param  tempValue : 8bit value for conversion
* @retval MOBLEUINT8 
*/         
MOBLEUINT8 BluenrgMesh_ModelsASCII_To_Char(MOBLEUINT8 tempValue)
{
   tempValue = tempValue - 0x30;
   return tempValue;
} 

__weak void Test_Process(void)
{
}
/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
