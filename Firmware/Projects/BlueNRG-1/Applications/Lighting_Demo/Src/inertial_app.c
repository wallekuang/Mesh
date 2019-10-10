/**
 ******************************************************************************
 * @file    inertial_app.c
 * @author  Central Labs
* @version V1.11.000
* @date    25-07-2019
 * @brief   This file contains definitions for Inertial application.
 *******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
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
 ********************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "inertial_app.h"
#include "BlueNRG1_conf.h"
#include "sleep.h"  
#include "appli_mesh.h"
#include "vendor.h"
#include "bluenrg_mesh.h"
#include "mesh_cfg.h"
/** @addtogroup BLUEMIC_1_APP BLUEMIC_1_APP
 * @{
 */

/** @defgroup BLUEMIC_1_INERTIAL_APP BLUEMIC_1_INERTIAL_APP
 * @{
 */

/** @defgroup INERTIAL_APP_Private_Types INERTIAL_APP_Private_Types
 * @{
 */
//extern MOBLEUINT16 IntensityValue;
extern MOBLEUINT8 IntensityFlag;
extern int X;
/** 
 * @brief Structure containing accelerometer or gyroscope value for each axis.
 */
typedef struct {
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} AxesRaw_t;

/**
  * @}
  */

/** @defgroup INERTIAL_APP_Private_Variables INERTIAL_APP_Private_Variables
 * @{
 */
void *ACCELERO_handle = NULL;
void *GYRO_handle = NULL;
/**
  * @}
  */

/** @defgroup INERTIAL_APP_Private_Constants INERTIAL_APP_Private_Constants
 * @{
 */   

/**
  * @}
  */

/** @defgroup INERTIAL_APP_Exported_Variables INERTIAL_APP_Exported_Variables
 * @{
 */  
volatile uint16_t AccGyroHandle = 0;
/**
  * @}
  */

INERTIAL_APP_Status AccGyro_Update(void);


/** @defgroup INERTIAL_APP_Exported_Functions INERTIAL_APP_Exported_Functions
 * @{
 */  

/**
 * @brief  This function is called to add Inertial characteristics.
 * @param  service_handle: handle of the service
 * @retval INERTIAL_APP_Status: INERTIAL_APP_SUCCESS if the configuration is ok, INERTIAL_APP_ERROR otherwise.
 */
INERTIAL_APP_Status INERTIAL_APP_add_char(uint16_t service_handle)
{      
  /*User Implementation*/
  return INERTIAL_APP_SUCCESS;
}

/**
 * @brief  This function is called to initialize Inertial application.
 * @param  None
 * @retval INERTIAL_APP_Status: INERTIAL_APP_SUCCESS if the configuration is ok, INERTIAL_APP_ERROR otherwise.
 */
INERTIAL_APP_Status INERTIAL_APP_Init(void)
{
  if(BSP_ACCELERO_Init(LSM6DSL_X_0, &ACCELERO_handle) == COMPONENT_ERROR)
  {
    return INERTIAL_APP_ERROR;
  }
  BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
  
  return INERTIAL_APP_SUCCESS;
}


/**
 * @brief  Update acceleration/Gryoscope characteristics value
 * @param  service_handle: handle of the service
 * @retval INERTIAL_APP_Status: INERTIAL_APP_SUCCESS if the configuration is ok, INERTIAL_APP_ERROR otherwise.
 */
INERTIAL_APP_Status INERTIAL_APP_DataUpdate()
{  
 /*User Implementation*/
  return INERTIAL_APP_SUCCESS;
}

/**
  * @}
  */
/*void INERTIAL_single_tap(uint8_t bool){
	if(bool) BSP_ACCELERO_Enable_Single_Tap_Detection_Ext( ACCELERO_handle, INT1_PIN);
	else BSP_ACCELERO_Disable_Single_Tap_Detection_Ext( ACCELERO_handle);
}*/
/**
  * @}
  */
void INERTIAL_double_tap(uint8_t bool){
	if(bool) BSP_ACCELERO_Enable_Double_Tap_Detection_Ext( ACCELERO_handle, INT2_PIN);
	else BSP_ACCELERO_Disable_Double_Tap_Detection_Ext( ACCELERO_handle);
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
