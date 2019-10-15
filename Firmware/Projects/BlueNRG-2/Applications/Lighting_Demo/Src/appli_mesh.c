/**
******************************************************************************
* @file    appli_mesh.c
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Application interface for Mesh 
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
#include <string.h>
#include "hal_common.h"
#include "bluenrg_config.h"
#include "models_if.h"
#include "bluenrg_mesh.h"
#include "appli_mesh.h"
#include "generic.h"
#include "serial_if.h"
#include "appli_nvm.h"
#include "gp_timer.h"
#include "mesh_cfg_usr.h"
#include "vendor.h"
#include "app_control.h"

/** @addtogroup BlueNRG_Mesh
*  @{
*/

/** @addtogroup Application_Callbacks_BlueNRG2
*  @{
*/

/* Private define ------------------------------------------------------------*/

/*********** Macros to be modified according to the requirement *************/
#define BOUNCE_THRESHOLD                        20U
#define LONG_PRESS_THRESHOLD                    1000U
#define MANUAL_UNPROVISION_TIMER                3000U
#define FLASH_ERASE_TIME                        100U
#define DISCOVER_TIMER                          10*60*1000 /* 10 minutes */
#define DEFAULT_DELAY_PACKET_FROM               500U
#define DEFAULT_DELAY_PACKET_RANDOM_TIME        500U
#define USER_OUTPUT_OOB_APPLI_PROCESS           0U
#define INPUT_OOB_TIMEOUT                       300U /* input Oob30 Sec timeout*/
#define PBADV_UNPROV_DEV_BEACON_INTERVAL        100U /* 100 ms */
/* Private macro -------------------------------------------------------------*/
#define MAX_APPLI_BUFF_SIZE             8 
#define MAX_PENDING_PACKETS_QUE_SIZE    2
#define DATA_BUFFER_LENGTH              8
#define MAX_NUMB_ELEMENTS               1

/**********************Friendship callbacks macros ****************************/
#define FN_CLEARED_REPEAT_REQUEST   1
#define FN_CLEARED_POLL_TIMEOUT     2
#define FN_CLEARED_FRIEND_CLEAR     3
#define LPN_CLEARED_NO_RESPONSE     1

/* Private variables ---------------------------------------------------------*/

enum ButtonState
{
  BS_OFF,
  BS_DEBOUNCE,
  BS_SHORT_PRESS,
  BS_LONG_PRESS
};

enum ButtonState buttonState = BS_OFF;
tClockTime tBounce = 0;
MOBLEUINT8 Appli_LedState = 0;
MOBLEUINT16 IntensityValue = INTENSITY_LEVEL_ZERO;
MOBLEUINT8 IntensityFlag = FALSE;
MOBLEUINT8 ProvisionFlag = 0;


#ifdef ENABLE_AUTH_TYPE_OUTPUT_OOB
static MOBLEUINT8 PrvngInProcess = 0;
static MOBLEUINT32 OutputOobData = 0;
static MOBLEUINT32 OutputOobBlinkCount = 0;
#endif
#ifdef ENABLE_AUTH_TYPE_INPUT_OOB
MOBLEUINT8 InputOobData[8] = {0};
MOBLEUINT8 inputOOBDataReady = 0;
#endif
/*Number Of Elements selected per Node. Maximum Elements supported = 3*/
MOBLEUINT8 NumberOfElements = APPLICATION_NUMBER_OF_ELEMENTS;

#ifdef ENABLE_AUTH_TYPE_STATIC_OOB
/* 16 octets Static OOB information to be input here. Used during provisioning by Library */
const MOBLEUINT8 StaticOobBuff[STATIC_OOB_SIZE] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
#else
const MOBLEUINT8 StaticOobBuff[] = {0};
#endif
#ifdef PUB_KEY_TYPE_OOB
/* 64 octets Public Key information to be input here. It is only required for Public Key OOB case.
Used during provisioning by Library */
const MOBLEUINT8 PubKeyBuff[64] =  {0xF4, 0x65, 0xE4, 0x3F, 0xF2, 0x3D, 0x3F, 0x1B, 0x9D, 0xC7, 
0xDF, 0xC0, 0x4D, 0xA8, 0x75, 0x81, 0x84, 0xDB, 0xC9, 0x66, 0x20, 0x47, 0x96, 0xEC, 0xCF, 
0x0D, 0x6C, 0xF5, 0xE1, 0x65, 0x00, 0xCC, 0x02, 0x01, 0xD0, 0x48, 0xBC, 0xBB, 0xD8, 0x99, 
0xEE, 0xEF, 0xC4, 0x24, 0x16, 0x4E, 0x33, 0xC2, 0x01, 0xC2, 0xB0, 0x10, 0xCA, 0x6B, 0x4D, 
0x43, 0xA8, 0xA1, 0x55, 0xCA, 0xD8, 0xEC, 0xB2, 0x79};

/* 32 octets Private Key information to be input here. It is only required for Public Key OOB case.
Used during provisioning by Library */
const MOBLEUINT8 PrivKeyBuff[32] =  {0x52, 0x9A, 0xA0, 0x67, 0x0D, 0x72, 0xCD, 0x64, 0x97, 0x50, 
0x2E, 0xD4, 0x73, 0x50, 0x2B, 0x03, 0x7E, 0x88, 0x03, 0xB5, 0xC6, 0x08, 0x29, 0xA5, 0xA3, 
0xCA, 0xA2, 0x19, 0x50, 0x55, 0x30, 0xBA};
#else
const MOBLEUINT8 PubKeyBuff[] = NULL;
const MOBLEUINT8 PrivKeyBuff[] = NULL;
#endif



/* Select Node as Sniffer, Means able to sniff all the packets 
*  0 - Filters are enabled and packets not targeted to node are filtered
*  1 - Filters are disabled and packets not targeted to node are not filtered
*/
MOBLEUINT8 DisableFilter = 0;

#if LOW_POWER_FEATURE
MOBLEINT32 BluenrgMesh_sleepTime;
MOBLEUINT32 SysRefCount;
#endif

/* Timer to control unprovisioned device beacons */
#if PB_ADV_SUPPORTED
struct timer discoverTimer;
tClockTime discoverTimerinterval = DISCOVER_TIMER;
#endif

extern const MOBLEUINT8* _bdaddr[];

/********************* Application configuration **************************/
#if defined(__IAR_SYSTEMS_ICC__) || defined(__CC_ARM)
MOBLEUINT8 bdaddr[8];

extern const char* _bnrgmNvmBase_data[];
const void *bnrgmNvmBase = _bnrgmNvmBase_data;

#ifdef INTERNAL_UNIQUE_NUMBER_MAC
static void Appli_GetMACfromUniqueNumber(void);
#endif /* INTERNAL_UNIQUE_NUMBER_MAC */
#else
#error "Unknown compiler"
#endif /* defined(__IAR_SYSTEMS_ICC__) || defined(__CC_ARM) */

/* Private function prototypes -----------------------------------------------*/
static void Appli_LongButtonPress(void);
static void Appli_ShortButtonPress(void);

/* Private functions ---------------------------------------------------------*/

/************************* Button Control functions ********************/
/**
* @brief  Function calls when a button is pressed for short duration  
* @param  void
* @retval void
*/ 
static void Appli_ShortButtonPress(void)
{
  BluenrgMesh_ModelsCommand();
}

/**
* @brief  Function calls when a button is pressed for Long duration  
* @param  void
* @retval void
*/ 
static void Appli_LongButtonPress(void)
{
  /*Edited for OTA, Macro is available in preprocessor*/   
#ifdef ST_USE_OTA_SERVICE_MANAGER_APPLICATION
  if(GetButton2State() == BUTTON_PRESSED)
    OTA_Jump_To_Service_Manager_Application();
#endif
}

/**
* @brief  Updates the button status  
* @param  int isPressed
* @retval void
*/ 
/**
* @brief  Updates the button status  
* @param  int isPressed
* @retval void
*/ 
static void Appli_UpdateButtonState(int isPressed)
{
  /* Check for button state */
  switch (buttonState)
  {
    /* Case for Button State off */
  case BS_OFF:
    /* if button is pressed */
    if (isPressed)
    {
      buttonState = BS_DEBOUNCE;
      tBounce = Clock_Time();
    }
    break;
    /* Case for Button Debounce */
  case BS_DEBOUNCE:
    if (isPressed)
    {
      /* Debouncing Delay check */
      if (Clock_Time() - tBounce > BOUNCE_THRESHOLD)
      {   if (GetButtonState() == BUTTON_PRESSED)    
        buttonState = BS_SHORT_PRESS;
      }
      else
      {
        break;
      }
    }
    else
    {
      buttonState = BS_OFF;
      break;
    }
    /* Case if Button is pressed for small duration */
  case BS_SHORT_PRESS:
    if (isPressed && ((Clock_Time() - tBounce) > LONG_PRESS_THRESHOLD))
    {
      buttonState = BS_LONG_PRESS;
      Appli_IntensityControlPublishing();
      
    }
    else
    {
      if (!isPressed)
      {
        Appli_ShortButtonPress();
      }
      break;
    }
    /* Case if Button is pressed for long duration */
  case BS_LONG_PRESS:
    /* Second Button handling in BlueNRG-2*/
    if (GetButton2State() == BUTTON_PRESSED)
    {
      SdkEvalLedOn(LED1);
      Appli_LongButtonPress();
    }
    break;
    /* Default case */
  default:
    buttonState = BS_OFF;
    break;
  }
  if (!isPressed)
  {
    buttonState = BS_OFF;
  }
}
/************************* LED Control functions ********************/
/**
* @brief  Controls the state of on board LED
* @param  void
* @retval void
*/ 
void Appli_LedCtrl(void)
{
  SetLed(Appli_LedState);
}

/************* BlueNRG-Mesh library callback functions ***************/

/**
* @brief  Blinks the on board LED  
* @param  none
* @retval MOBLE_RESULT status of result
*/ 
MOBLE_RESULT Appli_LedBlink(void)
{
  /* Switch On the LED */
  Appli_LedState = 1;
  Appli_LedCtrl();
  Clock_Wait(500);
  
  /* Switch Off the LED */
  Appli_LedState = 0;
  Appli_LedCtrl();
  Clock_Wait(500);
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Callback function Sets the state of the bulb 
* @param  MOBLEUINT16 ctrl which sets the lighting state of LED
* @retval MOBLE_RESULT status of result
*/ 
MOBLE_RESULT Appli_LedStateCtrlCb(MOBLEUINT16 ctrl)
{
  /* Switch On the LED */
  if(ctrl > 0)
  {
    Appli_LedState = 1;
  }
  /* Switch Off the LED */
  else
  {
    Appli_LedState = 0;
  }
  
  Appli_LedCtrl();
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  initializes BlueNRG Stack    
* @param  none
* @retval MOBLE_RESULT status of result
*/ 
MOBLE_RESULT Appli_BleStackInitCb()
{
  tBleStatus status; 
  
  /* BTLE Stack initialization */
  status =  BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  
  /* Check if command executed successfully */
  if (status)
    return MOBLE_RESULT_FAIL;
  else
    return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Sets transmission power of RF 
* @param  none
* @retval MOBLE_RESULT status of result
*/ 
MOBLE_RESULT Appli_BleSetTxPowerCb(void)
{
  tBleStatus status;
  /* API to change the Transmission power of BlueNRG Device */
  status = aci_hal_set_tx_power_level(0x01, 0x06); 
  /* Check if command executed successfully */
  if (status)
    return MOBLE_RESULT_FAIL;
  else
    return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  SET UUID value 
* @param  uuid_prefix_data : Pointer of UUID buffer data
* @retval MOBLE_RESULT status of result
*/ 
MOBLE_RESULT Appli_BleSetUUIDCb(MOBLEUINT8 *uuid_prefix_data)
{
  /* UUID is 128 bits (16 bytes) and can guarantee uniqueness across space and time.
     It can be “Time-based ?UUID or “truly-random or pseudo-random numbers?
 
     In this buffer user need to fill 10 bytes of UUID data. 
     Remaining 6 bytes, library fill this data with BDaddress.

    For example :
        F81D4FAE-7DEC-XBC4-Y12F-17D1AD07A961 (16 bytes)
        <any number> |_|  |_|   <BDAddress>

      X = 1 i.e. “Time Base UUID?
      X = 4 i.e. “Random Number?
      Y = 4 i.e. Conforming to the current spec 
    For UUID information refer RFC4122
  */
  
  /* copy UUID prefix data*/
  uint8_t uuid_buff[10] = {0xF8,0x1D,0x4F,0xAE,0x7D,0xEC};/*random value by user */

   uuid_buff[6] = 0x4B;  /* 0x4B => 4 : Random */
   uuid_buff[7] = 'S' ;  /* User define value */
   uuid_buff[8] = 0xA1;  /* 0xA1 => A : Conforming to the current Spec */
   uuid_buff[9] = 'T' ;  /* User define value */  

   memcpy(uuid_prefix_data,uuid_buff,sizeof(uuid_buff));
   memcpy(&uuid_prefix_data[10],bdaddr,6);  /* Copy bdaddr to last 6 bytes of UUID */
   
   return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  SET CID , PID and VID values 
* @param  company_product_info : vendor fill product information in this buffer
* @retval MOBLE_RESULT status of result
*/
MOBLE_RESULT Appli_BleSetProductInfoCB(MOBLEUINT8 *company_product_info)
{
  /* CID : provide 16-bit company identifier assigned by the Bluetooth SIG */
  uint16_t company_id                   = COMPANY_ID;
  
  /* PID : Provide 16-bit vendor-assigned product identifier */
  uint16_t product_id                   = PRODUCT_ID;
  
  /* VID : Provide 16-bit vendor-assigned product version identifier*/
  uint16_t product_version_id           = PRODUCT_VERSION_ID;
  
  memcpy(company_product_info + 0,(void *)&company_id,sizeof(company_id));
  memcpy(company_product_info + 2 ,(void *)&product_id,sizeof(product_id));
  memcpy(company_product_info + 4 ,(void *)&product_version_id,sizeof(product_version_id));
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Call back function called when GATT Connection Created  
* @param  none
* @retval none
*/ 
void Appli_BleGattConnectionCompleteCb(void)
{ 
  ProvisionFlag = 1;
  /* Proxy Node, will be called whenever Gatt connection is established */
  /* Turn ON Red LED*/
#if LOW_POWER_FEATURE
  /* do nothing */
#else
  SdkEvalLedOn(LED2);
#endif
}

/**
* @brief  Call back function called when GATT Disconnection Complete  
* @param  none
* @retval none
*/ 
void Appli_BleGattDisconnectionCompleteCb(void)
{
  ProvisionFlag = 0;
  /* Proxy Node, will be called whenever Gatt disconnected */ 
  /* Turn Off Red LED*/
#if LOW_POWER_FEATURE
  /* do nothing */
#else
  SdkEvalLedOff(LED2);
#endif
}

/**
* @brief Unprovisioned Node Identification
* @param MOBLEUINT8 data 
* @retval none
*/
void Appli_BleUnprovisionedIdentifyCb(MOBLEUINT8 data)
{
#ifdef ENABLE_AUTH_TYPE_OUTPUT_OOB
  PrvngInProcess = 1;  
#endif
  TRACE_M(TF_PROVISION,"Unprovisioned Node Identifier received: %02x\n\r", data);    
}

/**
* @brief  Set Number of Elements in a Node
* @param  none
* @retval Sending the value to BLE Mesh Library
*/ 
MOBLEUINT8 Appli_BleSetNumberOfElementsCb(void)
{
  if(NumberOfElements > MAX_NUMB_ELEMENTS)
  {
    TRACE_M(TF_ELEMENTS,"In version 1.11.000 one Element per node is supported!\r\n"); 
    return MAX_NUMB_ELEMENTS;
  }
  else
  {
  return NumberOfElements;
  }
}

/**
* @brief  Sets the Attention Timer Callback function
* @param  none
* @retval MOBLE_RESULT status of result
*/ 
MOBLE_RESULT Appli_BleAttentionTimerCb(void)
{
/* avoid printf, if low power feature is supported */  
#if (LOW_POWER_FEATURE == 0)
  TRACE_M(TF_MISC, " \r\n");
#endif /* LOW_POWER_FEATURE == 0 */
  return MOBLE_RESULT_SUCCESS;
}
/**
* @brief  Call back function to give Output OOB information 
* @param  MOBLEUINT8* output_oob
* @param  MOBLEUINT8 size
* @retval void
*/ 
void Appli_BleOutputOOBAuthCb(MOBLEUINT8* output_oob, MOBLEUINT8 size)
{
#ifdef ENABLE_AUTH_TYPE_OUTPUT_OOB
  uint32_t ooBData = 0;    
    for (MOBLEUINT8 i = 0; i < size; i++)
    {
        ooBData |= (output_oob[i] << 8*i);
    }
    OutputOobData = ooBData;
    TRACE_M(TF_PROVISION,"Output OOB information for provisioner: %d\n\r", ooBData);   
#endif
}

#ifdef ENABLE_AUTH_TYPE_OUTPUT_OOB
/**
* @brief  Process for both output and Input OOB handling
* @param  void
* @retval void
*/ 
void Appli_OobAuthenticationProcess(void)
{
    if(OutputOobBlinkCount < OutputOobData)
    {
      OutputOobBlinkCount++;
      Appli_LedBlink();
    }
    else
    {
        OutputOobBlinkCount++;
        Clock_Wait(1000);
        if(OutputOobBlinkCount > OutputOobData+3)
        {
            OutputOobBlinkCount = 0;
        }
    }
}
#endif
/**
* @brief  Call back function to provide Input OOB information
* @param  MOBLEUINT8 size
* @retval MOBLEUINT8*  
*/ 

MOBLEUINT8* Appli_BleInputOOBAuthCb(MOBLEUINT8 size)
{
#if defined (ENABLE_AUTH_TYPE_INPUT_OOB) && (ENABLE_SERIAL_INTERFACE)
  MOBLEUINT16 inputTimer = 0; 
  TRACE_M(TF_PROVISION,"Input OOB information for provisioner-Size: %d\n\r", size);   
  while(1)
  {
      Serial_InterfaceProcess();
      /* Check if input is completed or timeout */
      if((inputOOBDataReady != 0x00) | (inputTimer > INPUT_OOB_TIMEOUT))
      {
          inputOOBDataReady = 0;
          break;
      }
      Clock_Wait(100);
      inputTimer++;
  }
  return InputOobData; 
#else
  return NULL;
#endif
}
/**
* @brief  Call back function to provide Input OOB information
* @param  rcvdStringBuff: buffer to store input string 
* @param  rcvdStringSize: length of the input string 
* @retval void  
*/ 
void Appli_BleSerialInputOOBValue(char *rcvdStringBuff, uint16_t rcvdStringSize)
{
#ifdef  ENABLE_AUTH_TYPE_INPUT_OOB
  unsigned long long InputOobDatatemp;
    sscanf(rcvdStringBuff + 5, "%lld", &InputOobDatatemp);
    memmove(&InputOobData, &InputOobDatatemp, sizeof(InputOobDatatemp));
    inputOOBDataReady = 1;
#endif
}

/**
* @brief  Enable Node as Sniffer to Capture all the packets
* @param  none
* @retval Sending the value to BLE Mesh Library
*/
MOBLEUINT8 Appli_BleDisableFilterCb(void)
{
    return DisableFilter;
}

/**
* @brief  Checks and do Manual Unprovision of board  
* @param  void
* @retval void
*/ 
void Appli_CheckForUnprovision(void)
{
  /* Checks if the User button is pressed or not at the startup */
  /* On STEVAL-IDB008V2/ V1/ 7V1, 7V2 / 9V1 : GPIO_Pin_13 is used to check the 
     Button pressed state*/
  
  if (BUTTON_PRESSED == GetButtonState())
  {
    SetLed(1);
    tClockTime t = Clock_Time();
    int interrupted = 0;
    
    /*Wait to check if user is pressing the button persistently*/
    while ((Clock_Time() - t) < MANUAL_UNPROVISION_TIMER)
    {
      if (BUTTON_PRESSED != GetButtonState())
      {
        interrupted = 1;
        break;
      }
    }
    /*Unprovision, show that unprovisioning is completed, and 
    wait until user releases button*/
    if (!interrupted)
    {
      BluenrgMesh_Unprovision();
      AppliNvm_ClearModelState();     
      TRACE_M(TF_PROVISION,"Device is unprovisioned by application \r\n");      
      t = Clock_Time();
      while ((Clock_Time() - t) < FLASH_ERASE_TIME)
      {
        BluenrgMesh_Process();
      }
      
      while (BUTTON_PRESSED == GetButtonState())
      {
        Appli_LedBlink();
      }
    }
    SetLed(0);
  }
}


/**
* @brief  Checks and updates Mac address to generate MAC Address
* @param  void
* @retval MOBLEUINT8 sum return the sum calculated mac
*/ 
int Appli_CheckBdMacAddr(void)
{
  MOBLEUINT8 sum = 239;
  MOBLEUINT8 result = 0;
  
#ifdef EXTERNAL_MAC_ADDR_MGMT
  memcpy(bdaddr, (const MOBLEUINT8 *)_bdaddr , 7);
  bdaddr[7] = (EXTERNAL_MAC_ADDR_MGMT | EXTERNAL_MAC_TYPE);
#endif
  
#ifdef INTERNAL_UNIQUE_NUMBER_MAC
  Appli_GetMACfromUniqueNumber();
  bdaddr[7] = INTERNAL_UNIQUE_NUMBER_MAC;
#endif

  for (int i = 0; i < 6; ++i)
  {
    sum = (sum << 1) + bdaddr[i];
  }

  if (sum == bdaddr[6])
  {
    result = 1;
  }

#if defined(EXTERNAL_MAC_ADDR_MGMT) && defined(EXTERNAL_MAC_IS_PUBLIC_ADDR)
   /* Do nothing for modification of 2 MSb */
#else
  bdaddr[5] |= 0xC0;    /* 2 Msb bits shall be 11b for Static Random Address */
#endif
  
#ifdef GENERATE_STATIC_RANDOM_MAC
  bdaddr[7] = GENERATE_STATIC_RANDOM_MAC;   
                      /* Do nothing for bdaddr, just pass the identification */
  result = 1;         /* This will overwrite the above for loop result, 
                          which is redundant for this case */
#endif  

  return result;
}


/**
* @brief  Reads the unique serial number of the device
* @param  void
* @retval void
*/
#ifdef INTERNAL_UNIQUE_NUMBER_MAC
static void Appli_GetMACfromUniqueNumber(void)
{
#define BNRG1_UNIQUE_NUMBER_ADDR (0x100007F4)
  
  /*
  The unique serial number is a six bytes value stored at address 0x100007F4: 
  it is stored as two words (8 bytes) at address 0x100007F4 and 0x100007F8 
  with unique serial number padded with 0xAA55.
  */
  uint8_t *pdst;
  uint8_t *psrc;
  uint8_t i;
  MOBLEUINT8 sum = 239;
  
  pdst = (uint8_t*) (bdaddr);
  psrc = (uint8_t*)(BNRG1_UNIQUE_NUMBER_ADDR);
  
  for (i=0; i<6; i++)
  { /* Read only 6 Bytes for MAC Address */
    *pdst = *psrc;
    pdst++;
    psrc++;
  }
  
  
  bdaddr[5] &= 0xDF;    /* To ensure all bit are not 1 0r 0*/
  bdaddr[5] |= 0xC0;    /*  Static Device Address: The two most significant 
                            bits of the address shall be equal to 1*/
  
  for (int i = 0; i < 6; ++i)
  {
    sum = (sum << 1) + bdaddr[i];
  }
  bdaddr[6] = sum;
}
#endif


/**
* @brief  provides the information of the power saving mode
* @param  sleepMode curently unused, to be used in future 
* @retval SleepModes returns the mode of the controller
*/
SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
  if (buttonState == BS_OFF)
  {
    return SLEEPMODE_WAKETIMER;
  }
  else
  {
    return SLEEPMODE_RUNNING;
  }
}


/**
* @brief  callback for unprovision the node by provisioner.
* @param  status reserved for future 
* @retval void
*/
void BluenrgMesh_UnprovisionCallback(MOBLEUINT8 reason)
{
  ProvisionFlag = 0; 
  TRACE_M(TF_PROVISION,"Device is unprovisioned by provisioner \n\r");
#if PB_ADV_SUPPORTED
  BluenrgMesh_SetUnprovisionedDevBeaconInterval(PBADV_UNPROV_DEV_BEACON_INTERVAL);
#endif 
  AppliNvm_ClearModelState();
}

/**
* @brief  callback for provision the node by provisioner.
* @param  void
* @retval void
*/
void BluenrgMesh_ProvisionCallback(void)
{
  ProvisionFlag = 1;
#ifdef ENABLE_AUTH_TYPE_OUTPUT_OOB
  PrvngInProcess = 0;
#endif
  TRACE_M(TF_PROVISION,"Device is provisioned by provisioner \r\n");
}

/**
* @brief  callback for friendship established by friend node
* @param  address of corresponding low power node
* @param  receive delay of low power node (unit ms)
* @param  poll timeout of low power node (unit 100ms)
* @param  number of elements of low power node
* @param  previous friend address of low power node (can be invalid address)
* @retval void
*/
void BluenrgMesh_FnFriendshipEstablishedCallback(MOBLE_ADDRESS lpnAddress,
                                                 MOBLEUINT8 lpnReceiveDelay,
                                                 MOBLEUINT32 lpnPollTimeout,
                                                 MOBLEUINT8 lpnNumElements,
                                                 MOBLE_ADDRESS lpnPrevFriendAddress)
{ 
  TRACE_M(TF_LPN_FRND,"Friendship established. Low power node address 0x%.4X \r\n", lpnAddress);
  TRACE_M(TF_LPN_FRND,"Low power node receive delay %dms \r\n", lpnReceiveDelay);
  TRACE_M(TF_LPN_FRND,"Low power node poll timeout %dms \r\n", lpnPollTimeout*100);
  TRACE_M(TF_LPN_FRND,"Low power node number of elements %d \r\n", lpnNumElements);
  if (lpnPrevFriendAddress != MOBLE_ADDRESS_UNASSIGNED)
  {
    TRACE_M(TF_LPN_FRND,"Low power node previous friend address 0x%.4X \r\n", lpnPrevFriendAddress);
  }
}

/**
* @brief  callback for friendship clear by friend node
* @param  reason of friendship clear
*         0: reserved,
*         1: friend request received from existing low power node (friend)
*         2: low power node poll timeout occurred
*         3: friend clear received
* @param  previous friend address of low power node (can be invalid address)
* @retval void
*/
void BluenrgMesh_FnFriendshipClearedCallback(MOBLEUINT8 reason, MOBLE_ADDRESS lpnAddress)
{
  TRACE_M(TF_LPN_FRND,"Friendship cleared. Low power node address 0x%.4X \r\n", lpnAddress);
  
  switch(reason)
  {
  case FN_CLEARED_REPEAT_REQUEST: 
    TRACE_M(TF_LPN_FRND,"Reason: New friend request received from existing low power node \r\n");
    break;
  case FN_CLEARED_POLL_TIMEOUT:
    TRACE_M(TF_LPN_FRND,"Reason: Low power node poll timeout occurred \r\n");
    break;
  case FN_CLEARED_FRIEND_CLEAR:
    TRACE_M(TF_LPN_FRND,"Reason: Friend clear received \r\n");
    break;
  default:
    TRACE_M(TF_LPN_FRND,"Reason: Invalid \r\n");
    break;
  }
}

/**
* @brief  callback for friendship established by low power node
* @param  address of corresponding friend node
* @retval void
*/
void BluenrgMesh_LpnFriendshipEstablishedCallback(MOBLE_ADDRESS fnAddress)
{
  /* Friendship established */
}

/**
* @brief  callback for friendship cleare by low power node
* @param  reason of friendship clear.
*         0: reserved
*         1: No response received from friend node
* @retval void
*/
void BluenrgMesh_LpnFriendshipClearedCallback(MOBLEUINT8 reason, MOBLE_ADDRESS fnAddress)
{ 
  TRACE_M(TF_LPN_FRND,"Friendship cleared. Friend node address 0x%.4x \r\n", fnAddress);
  
  if (reason == LPN_CLEARED_NO_RESPONSE)
  {
    TRACE_M(TF_LPN_FRND,"Reason: Friend node not responding \r\n");
  }
  else
  {
    TRACE_M(TF_LPN_FRND,"Reason: Invalid \r\n");
  } 
}

/** 
* @brief  New neighbor appeared callback in neighbor table.
          If MAC address of neighbor changes it appears as new neighbor.
* @param  MAC address of neighbor.
*         is neighbor provisioned or unprovisioned device.
*         uuid of neighbor. NULL if not available
*         network address of neighbor. MOBLE_ADDRESS_UNASSIGNED if not available
*         last updated rssi value.
* @retval void
*/
void BluenrgMesh_NeighborAppearedCallback(const MOBLEUINT8* bdAddr,
                                          MOBLEBOOL provisioned,
                                          const MOBLEUINT8* uuid,
                                          MOBLE_ADDRESS networkAddress,
                                          MOBLEINT8 rssi)
{
  TRACE_M(TF_NEIGHBOUR,"New neighbor appeared. Neighbor MAC address:");
  
  for (MOBLEUINT8 count=0 ; count<6; count++)
  {
    TRACE_M(TF_NEIGHBOUR,"%.2x ", bdAddr[count]);
  }
  
  if (provisioned == MOBLE_TRUE)
  {
    TRACE_M(TF_NEIGHBOUR,"-> Provisioned node. \n\r");
  }
  else
  {
    TRACE_M(TF_NEIGHBOUR,"-> Unprovisioned device. \n\r");
  }
  
  TRACE_M(TF_NEIGHBOUR,"rssi: %d. ", rssi);
  
  if (networkAddress != MOBLE_ADDRESS_UNASSIGNED)
  {
    TRACE_M(TF_NEIGHBOUR,"Network address: 0x%.4x\n\r", networkAddress);
  }
  else
  {
    TRACE_M(TF_NEIGHBOUR,"Network address not available\n\r");
  }

  for (MOBLEUINT8 i=0; i<16; i++)
  {
    if (uuid[i] == 0x00)
    {
      continue;
    }
    else
    {
      TRACE_M(TF_NEIGHBOUR,"UUID: ");
      
      for (MOBLEUINT8 count=0 ; count<16; count++)
      {
        TRACE_M(TF_NEIGHBOUR,"%.2x ", uuid[count]);
      }
      
      break;
    }
  }
  
  TRACE_M(TF_NEIGHBOUR,"\n\r");
}

/** 
* @brief  Existing neighbor refreshed callback in neighbor table.
          If MAC address of neighbor changes it appears as new neighbor.
* @param  MAC address of neighbor.
*         is neighbor provisioned or unprovisioned device.
*         uuid of neighbor. NULL if not available
*         network address of neighbor. MOBLE_ADDRESS_UNASSIGNED if not available
*         last updated rssi value.
* @retval void
*/
void BluenrgMesh_NeighborRefreshedCallback(const MOBLEUINT8* bdAddr,
                                          MOBLEBOOL provisioned,
                                          const MOBLEUINT8* uuid,
                                          MOBLE_ADDRESS networkAddress,
                                          MOBLEINT8 rssi)
{
  TRACE_M(TF_NEIGHBOUR,"Existing neighbor refreshed. Neighbor MAC address:");
  
  for (MOBLEUINT8 count=0 ; count<6; count++)
  {
    TRACE_M(TF_NEIGHBOUR,"%.2x ", bdAddr[count]);
  }
  
  if (provisioned == MOBLE_TRUE)
  {
    TRACE_M(TF_NEIGHBOUR,"-> Provisioned node. \n\r");
  }
  else
  {
    TRACE_M(TF_NEIGHBOUR,"-> Unprovisioned device. \n\r");
  }
  
  TRACE_M(TF_NEIGHBOUR,"rssi: %d. ", rssi);
  
  if (networkAddress != MOBLE_ADDRESS_UNASSIGNED)
  {
    TRACE_M(TF_NEIGHBOUR,"Network address: 0x%.4x\n\r", networkAddress);
  }
  else
  {
    TRACE_M(TF_NEIGHBOUR,"Network address not available\n\r");
  }
  
  for (MOBLEUINT8 i=0; i<16; i++)
  {
    if (uuid[i] == 0x00)
    {
      continue;
    }
    else
    {
      TRACE_M(TF_NEIGHBOUR,"UUID: ");
      
      for (MOBLEUINT8 count=0 ; count<16; count++)
      {
        TRACE_M(TF_NEIGHBOUR,"%.2x ", uuid[count]);
      }
      
      break;
    }
  }
  
  TRACE_M(TF_NEIGHBOUR,"\n\r");
}

/**
* @brief  Appli_IntensityControl:Function to increase the intensity of led by
*          Publishing the value.
* @param  void
*/
void Appli_IntensityControlPublishing(void)
{
  MOBLEUINT8 generic_Level_Buff[3];
  MOBLE_ADDRESS srcAdd;
  MOBLE_RESULT result = MOBLE_RESULT_SUCCESS;
  
  srcAdd = BluenrgMesh_GetAddress();
  
  if(IntensityFlag == FALSE)
  {
      
    IntensityValue = IntensityValue + (INTENSITY_LEVEL_FULL/5);
    generic_Level_Buff[0] = (MOBLEUINT8)IntensityValue;
    generic_Level_Buff[1] = (MOBLEUINT8)(IntensityValue >> 8) ;
    
    result = BluenrgMesh_SetRemotePublication(GENERIC_MODEL_SERVER_LEVEL_MODEL_ID, srcAdd ,
                                              GENERIC_LEVEL_SET_UNACK , 
                                              generic_Level_Buff,3, MOBLE_FALSE, MOBLE_FALSE);
    if(result)
    {
      TRACE_M(TF_GENERIC,"Publication Error \r\n");
    }
    
    if(IntensityValue >= INTENSITY_LEVEL_FULL)
     {
        IntensityFlag = TRUE;
     }
     
  }
  else
  {
     
    IntensityValue = IntensityValue - (INTENSITY_LEVEL_FULL/5);
    generic_Level_Buff[0] = (MOBLEUINT8)IntensityValue;
    generic_Level_Buff[1] = (MOBLEUINT8)(IntensityValue >> 8) ;
    
    result = BluenrgMesh_SetRemotePublication(GENERIC_MODEL_SERVER_LEVEL_MODEL_ID, srcAdd ,
                                              GENERIC_LEVEL_SET_UNACK , 
                                              generic_Level_Buff,3, MOBLE_FALSE, MOBLE_FALSE);
    
    if(result)
    {
      TRACE_M(TF_GENERIC,"Publication Error \r\n");
    }
    
    if(IntensityValue <= INTENSITY_LEVEL_ZERO)
    {
      IntensityFlag = FALSE;
    }
  
  }
}

/**
* @brief  Low Power mode process 
*         If low power feature is supported, device enables low power mode
* @param  void
* @retval void
*/
void Appli_LowPowerProcess(void)
{
#if LOW_POWER_FEATURE
  BluenrgMesh_sleepTime = (MOBLEINT32)BluenrgMesh_GetSleepDuration();
  
  if (BluenrgMesh_sleepTime > 0)
  {
    /* Timer 0 used for low power */
    HAL_VTimerStart_ms(0, BluenrgMesh_sleepTime);
    
    /* To start virtual timer */
    BTLE_StackTick();

    /* save virtual timer current count */
    SysRefCount = HAL_VTimerGetCurrentTime_sysT32();
    
    /* wakeup either from io or virtual timer */
    BlueNRG_Sleep(SLEEPMODE_WAKETIMER, WAKEUP_IO13, WAKEUP_IOx_LOW);
    
    /* update systick count based on updated virtual timer count */
    Set_Clock_Time(Clock_Time() + 
                   HAL_VTimerDiff_ms_sysT32(HAL_VTimerGetCurrentTime_sysT32(), SysRefCount));
    
    /* Stop Timer 0 */
    HAL_VTimer_Stop(0);
  }
#endif /* LOW_POWER_FEATURE */
}

/**
* @brief  Application processing 
*         This function should be called in main loop
* @param  void
* @retval void
*/
void Appli_Process(void)
{
  Appli_UpdateButtonState(GetButtonState() == BUTTON_PRESSED 
                          || GetButton2State() == BUTTON_PRESSED);

#ifdef ENABLE_SAVE_MODEL_STATE_NVM  
  AppliNvm_Process();
#endif
  
  Appli_LowPowerProcess();
#ifdef ENABLE_AUTH_TYPE_OUTPUT_OOB  
  if(PrvngInProcess)
  {
#if USER_OUTPUT_OOB_APPLI_PROCESS
      Appli_OobAuthenticationProcess();
#endif  
  }
#endif

#if ENABLE_SERIAL_INTERFACE
      Serial_InterfaceProcess();
#endif
#if PB_ADV_SUPPORTED
      if (Timer_Expired(&discoverTimer))
      {
          BluenrgMesh_SetUnprovisionedDevBeaconInterval(0);
      }
#endif

	app_control_tick();

}
/**
* @brief  Initializes the Application  
* @param  void
* @retval void
*/
void Appli_Init(void)
{
#ifdef ENABLE_UNPROVISIONING_BY_POWER_ONOFF_CYCLE  
  /* Hardware Factory Restore implementation */
  AppliNvm_FactorySettingReset();
#endif
  
#if PB_ADV_SUPPORTED
  Timer_Set(&discoverTimer, discoverTimerinterval);
#endif
	app_control_init();
}

/**
* @}
*/

/**
* @}
*/
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
