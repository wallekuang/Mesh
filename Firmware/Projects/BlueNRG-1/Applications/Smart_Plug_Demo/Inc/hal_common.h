/**
******************************************************************************
* @file    hal_common.h
* @author  BLE Mesh Team
* @version V1.11.000
* @date    25-07-2019
* @brief   Common functions of HAL file 
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
#ifndef _HAL_H_
#define _HAL_H_

/* Includes ------------------------------------------------------------------*/

#ifdef NUCLEO_L152RE /* NUCLEO_L152RE */
#include "user_if.h" 
#else /* not NUCLEO_L152RE */
#if defined(BLUENRG1_DEVICE) || defined(BLUENRG2_DEVICE) /* BLUENRG1 or BLUENRG2 */
#include "hal.h"   
#include "sleep.h"
#else /* not BLUENRG1 or BLUENRG2 */
#error "Unsupported board"
#endif /* BLUENRG1 or BLUENRG2 */
#endif /* NUCLEO_L152RE */

#include <stdio.h>
#include <stdlib.h>

#include <stdint.h>
#include <stdbool.h>

#include "types.h"

void SetLed(int state);
BUTTON_STATE GetButtonState(void);
#ifndef SMART_PLUG
BUTTON_STATE GetButton2State(void);
#endif
void InitDevice(void);
void GPIO_InitNVICPowerOff(void);
void GPIO_InitSensorPin(void);
void ShouldSleepFunc(void);

#endif /* _HAL_H_ */
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

