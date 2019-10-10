/*
 Copyright (c) 2017, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 Core and is dual licensed,
 either 'STMicroelectronics
 Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

 *******************************************************************************

 'STMicroelectronics Proprietary license'

 *******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document is strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


 *******************************************************************************

 Alternatively, VL53L1 Core may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

 *******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


 *******************************************************************************

 */

#include "VL53L1X_platform.h"
#include <stdio.h>      
#include <stdint.h>
#include <string.h>     
#include "SDK_EVAL_I2C.h"
#include "clock.h"

#if(ENABLE_DEBUG==1)
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

volatile uint32_t SystemClockTick;

uint32_t value;

int8_t VL53L1_WriteMulti(uint8_t dev, uint16_t index, uint8_t *pdata, uint32_t count) 
{
	int8_t status = 0;
	uint8_t i, ReadTemp[8], WriteTemp[8];

//  SdkEvalI2CWrite16(pdata, dev, index, count);

	VL53L1_ReadMulti(dev, index, ReadTemp, count);
	value = 0xff;
  for (i = 0; i < count; i++) 
  {
		WriteTemp[i] = pdata[i];
    if (WriteTemp[i] != ReadTemp[i]) 
    {
			status = i;
		}
	}
  if (status > 0) 
  {
		status = 0;
	}
	return status;
}

int8_t VL53L1_ReadMulti(uint8_t dev, uint16_t index, uint8_t *pdata, uint32_t count) 
{

	int8_t status = 0;
 // SdkEvalI2CRead16(pdata, dev, index, count);
	return status;
}

uint16_t ReadRegister32(uint8_t dev, uint16_t RegisterAdress) 
{
	uint8_t buffer[4];
	VL53L1_ReadMulti(dev, RegisterAdress, buffer, 4);
	return  (uint16_t)((uint32_t) buffer[0] << 24) + ((uint32_t) buffer[1] << 16) + ((uint32_t) buffer[2] << 8) + (uint32_t) buffer[3];
}

uint16_t ReadRegister16(uint8_t dev, uint16_t RegisterAdress) 
{
	uint8_t buffer[2];

	VL53L1_ReadMulti(dev, RegisterAdress, buffer, 2);
	return (uint16_t) (((uint16_t) (buffer[0]) << 8) + (uint16_t) buffer[1]);
}

uint8_t ReadRegister8(uint8_t dev, uint16_t RegisterAdress) 
{

	uint8_t buffer[1];

	 VL53L1_ReadMulti(dev, RegisterAdress, buffer, 1);
	 return buffer[0];

}

void WriteRegister8(uint8_t dev, uint16_t RegisterAdress, uint8_t value) 
{
	uint8_t buffer[1];

	// Split 16-bit word into MS and LS uint8_t
	buffer[0] = (uint8_t) (value);

	VL53L1_WriteMulti(dev, RegisterAdress, buffer, 1);

}

void WriteRegister16(uint8_t dev, uint16_t RegisterAdress, uint16_t value) 
{
	uint8_t buffer[2];

	// Split 16-bit word into MS and LS uint8_t
	buffer[0] = (uint8_t) (value >> 8);
	buffer[1] = (uint8_t) (value & 0x00FF);

	VL53L1_WriteMulti(dev, RegisterAdress, buffer, 2);

}

void WriteRegister32(uint8_t dev, uint16_t RegisterAdress, uint32_t value) 
{
	uint8_t buffer[4];

	// Split 32-bit word into MS ... LS bytes
	buffer[0] = (uint8_t) (value >> 24);
	buffer[1] = (uint8_t) ((value & 0x00FF0000) >> 16);
	buffer[2] = (uint8_t) ((value & 0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (value & 0x000000FF);

	VL53L1_WriteMulti(dev, RegisterAdress, buffer, 4);

}

double StartTime = 0, CurrentTime;
void WaitMs(uint32_t TimeMs) 
{
	StartTime = SystemClockTick;
	CurrentTime = SystemClockTick - StartTime;
	while (CurrentTime < TimeMs) {
		CurrentTime = ((SystemClockTick) - StartTime);
	}
}

