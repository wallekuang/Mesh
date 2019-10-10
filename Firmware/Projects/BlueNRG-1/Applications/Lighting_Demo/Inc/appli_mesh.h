/**
******************************************************************************
* @file    appli_mesh.h
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Header file for the user application file 
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
#ifndef __APPLI_MESH_H
#define __APPLI_MESH_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported variables  ------------------------------------------------------- */

extern MOBLEUINT8 bdaddr[]; 

/* Exported Functions Prototypes ---------------------------------------------*/
MOBLE_RESULT Appli_BleStackInitCb(void);
MOBLE_RESULT Appli_BleSetTxPowerCb(void);
MOBLE_RESULT Appli_BleSetUUIDCb(MOBLEUINT8 *uuid_prefix_data);
MOBLE_RESULT Appli_BleSetProductInfoCB(MOBLEUINT8 *company_product_info);
void Appli_BleUnprovisionedIdentifyCb(MOBLEUINT8 data);
void Appli_BleGattConnectionCompleteCb(void);
void Appli_BleGattDisconnectionCompleteCb(void);
MOBLEUINT8 Appli_BleSetNumberOfElementsCb(void);
MOBLE_RESULT Appli_BleAttentionTimerCb(void);
void Appli_BleOutputOOBAuthCb(MOBLEUINT8* output_oob, MOBLEUINT8 size);
MOBLEUINT8* Appli_BleInputOOBAuthCb(MOBLEUINT8 size);
void Appli_BleSerialInputOOBValue(char *rcvdStringBuff, uint16_t rcvdStringSize);
MOBLEUINT8 Appli_BleDisableFilterCb(void);

void Appli_IntensityControlPublishing(void);

int Appli_CheckBdMacAddr(void);
MOBLE_RESULT Appli_LedBlink(void);
MOBLE_RESULT Appli_LedStateCtrlCb(MOBLEUINT16 ctrl);

void Appli_CheckForUnprovision(void);
void Appli_Process(void);
void Appli_LedCtrl(void);
void Appli_Init(void);
#endif /* __APPLI_MESH_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

