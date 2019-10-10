/**
******************************************************************************
* @file    main.c
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   main file of the application
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
#include "appli_light.h"
#include "models_if.h"
#include "mesh_cfg.h"
#include "PWM_config.h"
#include "PWM_handlers.h"
#if USER_DEFINED_PLATFORM == USER_EVAL_PLATFORM
  #include "LPS22HH.h"
#else
  #include "LPS25HB.h"
#endif
#include "BlueNRG_x_device.h"
#include "miscutil.h"

/** @addtogroup BlueNRG_Mesh
 *  @{
 */

/** @addtogroup main_BlueNRG2
*  @{
*/
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


const MOBLE_USER_BLE_CB_MAP user_ble_cb =
{  
  Appli_BleStackInitCb,
  Appli_BleSetTxPowerCb,
  Appli_BleGattConnectionCompleteCb,
  Appli_BleGattDisconnectionCompleteCb,
  Appli_BleUnprovisionedIdentifyCb,
  Appli_BleSetUUIDCb,
  Appli_BleSetProductInfoCB,
  Appli_BleSetNumberOfElementsCb,
  Appli_BleDisableFilterCb
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* This structure contains Mesh library Initialisation info data */
const Mesh_Initialization_t BLEMeshlib_Init_params = {
  bdaddr,
  &TrParams,
  &FnParams,
  &LpnParams,
  &NeighborTableParams,
  BLUENRG_MESH_FEATURES,
  BLUENRG_MESH_PRVN_BEARER_INFO,
  &PrvnParams,
  &DynBufferParam
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief This function is the Main routine.
* @param  None
* @retval None
*/
int main(void)
{
  /* Device Initialization */
  InitDevice();

  PWM_Init();
  
  /* PWM Initialization */  
  Appli_Light_PwmInit();

  Get_CrashHandlerInfo();
   
  /* Check for valid Board Address */
  if (!Appli_CheckBdMacAddr())
  {
    TRACE_I(TF_INIT,"Bad BD_MAC ADDR!\r\n");
    /* LED Blinks if BDAddr is not appropriate */
    while (1)
    {
      Appli_LedBlink();
    }
  }
  
  /* Set BLE configuration function callbacks */
  BluenrgMesh_BleHardwareInitCallBack(&user_ble_cb);  
  
  /* Initializes BlueNRG-Mesh Library */
  if (MOBLE_FAILED (BluenrgMesh_Init(&BLEMeshlib_Init_params) ))
  {
    TRACE_I(TF_INIT,"Could not initialize BlueNRG-Mesh library!\r\n");   
    /* LED continuously blinks if library fails to initialize */
    while (1)
    {
      Appli_LedBlink();
    }
  }

  /* Checks if the node is already provisioned or not */
  if (BluenrgMesh_IsUnprovisioned() == MOBLE_TRUE)
  {
      BluenrgMesh_InitUnprovisionedNode(); /* Initalizes Unprovisioned node */
    
    TRACE_I(TF_PROVISION,"Unprovisioned device \r\n");
    
#if PB_ADV_SUPPORTED
    BluenrgMesh_SetUnprovisionedDevBeaconInterval(100);
#endif    
  }
  else
  {
      BluenrgMesh_InitProvisionedNode();  /* Initalizes Provisioned node */
    TRACE_I(TF_PROVISION,"Provisioned node \r\n");
  }

  /* Initializes the Application */
  /* This function also checks for Power OnOff Cycles     
     Define the following Macro "ENABLE_UNPROVISIONING_BY_POWER_ONOFF_CYCLE" 
     to check the Power-OnOff Cycles
    5 Continous cycles of OnOff with Ontime <2 sec will cause unprovisioning
  */
  Appli_Init();
  
  /* Check to manually unprovision the board */
  /* On STEVAL-IDB008V2/ V1/ 7V1, 7V2 / 9V1 : GPIO_Pin_13 is used to check the 
     Button pressed state at Power-On */
  Appli_CheckForUnprovision();

  /* Set attention timer callback */
  BluenrgMesh_SetAttentionTimerCallback(Appli_BleAttentionTimerCb);

  /* Prints the MAC Address of the board */
  TRACE_I(TF_INIT,"BlueNRG-Mesh Lighting Demo v%s\n\r", BLUENRG_MESH_APPLICATION_VERSION); 
  TRACE_I(TF_INIT,"BlueNRG-Mesh Library v%s\n\r", BluenrgMesh_GetLibraryVersion()); 
  TRACE_I(TF_INIT,"BD_MAC Address = [%02x]:[%02x]:[%02x]:[%02x]:[%02x]:[%02x] \n\r",
          bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Models intialization */  
  BluenrgMesh_ModelsInit();
  
  /* Turn on Yellow LED */
#if (LOW_POWER_FEATURE == 1)
  SdkEvalLedOn(LED1);
#endif
  
  /* Main infinite loop */
  while(1)
  {
    BluenrgMesh_Process();
    BluenrgMesh_ModelsProcess(); /* Models Processing */
    Appli_Process();
  }
}

#ifdef USE_FULL_ASSERT /* USE_FULL_ASSERT */
/**
* @brief This function is the assert_failed function.
* @param file
* @param line
* @note  Reports the name of the source file and the source line number
*        where the assert_param error has occurred.
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  while (1)
  {
    SetLed(1);
    Clock_Wait(100);
    SetLed(0);
    Clock_Wait(100);
  }
}
#endif /* USE_FULL_ASSERT */
/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
