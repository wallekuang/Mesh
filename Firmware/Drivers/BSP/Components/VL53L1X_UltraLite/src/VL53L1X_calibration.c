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
#include <VL53L1X_api.h>
#include <VL53L1X_calibration.h>
#include "VL53L1X_platform.h"

#define ALGO__PART_TO_PART_RANGE_OFFSET_MM	0x001E
#define MM_CONFIG__INNER_OFFSET_MM		0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 		0x0022

int16_t Offset = 0x7FFF;	/* Error code */
uint16_t Xtalk = 0x7FFF;	/* Error code */

int16_t VL53L1X_CalibrateOffset(uint16_t dev, uint16_t TargetDistInMm)
{
	uint8_t i = 0;
	int16_t AverageDistance = 0;

	WriteRegister16(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
	WriteRegister16(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
	WriteRegister16(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);

	VL53L1X_StartRanging(dev);	/* Enable VL53L1X sensor */

	/* Realize 50 measures without saving distance (time to warm) */

	for (i = 0; i < 50; i++) {
		VL53L1X_ClearInterrupt(dev);
		while (VL53L1X_CheckForDataReady(dev) == 0)
			WaitMs(1);
	}
	/* Realize 50 measures with saving distance         */
	for (i = 0; i < 50; i++) {
		VL53L1X_ClearInterrupt(dev);
		VL53L1X_CheckForDataReady(dev);
		while (VL53L1X_CheckForDataReady(dev) == 0)
			WaitMs(1);
		AverageDistance = AverageDistance + VL53L1X_GetDistance(dev);
	}
	VL53L1X_StopRanging(dev);
	AverageDistance = AverageDistance / 50;
	Offset = TargetDistInMm - AverageDistance;
	return Offset;
}

uint16_t VL53L1X_CalibrateXtalk(uint16_t dev, uint16_t TargetDistInMm)
{
	uint8_t i = 0;
	float AverageSignalRate = 0;
	float AverageDistance = 0;
	float AverageSpadNb = 0;

	VL53L1X_StartRanging(dev);
	for (i = 0; i < 50; i++) {
		VL53L1X_ClearInterrupt(dev);
		while (VL53L1X_CheckForDataReady(dev) == 0)
			WaitMs(1);
	}

	for (i = 0; i < 50; i++) {
		VL53L1X_ClearInterrupt(dev);
		while (VL53L1X_CheckForDataReady(dev) == 0)
			WaitMs(1);
		AverageDistance = AverageDistance + VL53L1X_GetDistance(dev);
		AverageSpadNb = AverageSpadNb + VL53L1X_GetSpadNb(dev);
		AverageSignalRate =
		    AverageSignalRate + VL53L1X_GetSignalRate(dev);
	}
	VL53L1X_StopRanging(dev);
	AverageDistance = AverageDistance / 50;
	AverageSpadNb = AverageSpadNb / 50;
	AverageSignalRate = AverageSignalRate / 50;
	/* Calculate Xtalk value */
	Xtalk = (uint16_t)(512 * (AverageSignalRate * (1 -(AverageDistance / TargetDistInMm))) / AverageSpadNb);
	return Xtalk;
}
