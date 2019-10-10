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
#include "VL53L1X_def.h"
#include "string.h"
#include "VL53L1X_platform.h"

#define REVISION 1

uint16_t VL53L1X_GetRevision(void) {
	return REVISION;
}

uint16_t VL53L1X_SetI2CAddress(uint16_t old_dev, uint16_t new_dev) {
	WriteRegister8(old_dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_dev >> 1);
	return 0;
}

/* Write a default configuration to initialize the VL53L1X */
uint16_t VL53L1X_SensorInit(uint8_t dev) {
	uint8_t Addr = 0x00;
	uint8_t VL51L1X_NVM_CONFIGURATION[24];

	WriteRegister8(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
	for (Addr = 0x16; Addr <= 0x2D; Addr++)
		VL51L1X_NVM_CONFIGURATION[Addr - 0x16] = ReadRegister8(dev, Addr);

	for (Addr = 0x16; Addr <= 0x2D; Addr++)
		WriteRegister8(dev, Addr, VL51L1X_NVM_CONFIGURATION[Addr - 0x16]);

	for (Addr = 0x2E; Addr <= 0x87; Addr++)
		WriteRegister8(dev, Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2E]);
	return 0;
}

/* Clear interrupt */
uint16_t VL53L1X_ClearInterrupt(uint8_t dev) {
	WriteRegister8(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
	return 0;
}

/*
 * 1 -> Active Low
 * 0 -> Active High
 */
uint16_t VL53L1X_SetInterruptPolarity(uint8_t dev, uint8_t NewPolarity) {
	uint8_t Temp;

	Temp = ReadRegister8(dev, GPIO_HV_MUX__CTRL) & 0xEF;
	WriteRegister8(dev, GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);
	return 0;
}

uint16_t VL53L1X_GetInterruptPolarity(uint8_t dev) {
	uint8_t Temp;

	Temp = (ReadRegister8(dev, GPIO_HV_MUX__CTRL) & 0x10);
	return !(Temp >> 4);
}

uint16_t VL53L1X_StartRanging(uint8_t dev) {
	WriteRegister8(dev, SYSTEM__MODE_START, 0x40); /* Enable VL53L1X */
	return 0;
}

uint16_t VL53L1X_StopRanging(uint8_t dev) {
	WriteRegister8(dev, SYSTEM__MODE_START, 0x00); /* Disable VL53L1X */
	return 0;
}

/*
 * 0-> Not Ready, 1-> Ready
 */
uint16_t VL53L1X_CheckForDataReady(uint8_t dev) {
	uint8_t Temp;

	Temp = ReadRegister8(dev, GPIO__TIO_HV_STATUS);
	/* Read in the register if a new value is available */
	if ((Temp & 1) == !VL53L1X_GetInterruptPolarity(dev))
		return 0;
	else
		return 1;
}

/* Set the TimingBudget in ms */
uint16_t VL53L1X_SetTimingBudgetInMs(uint8_t dev, uint16_t TimingBudgetInMs) {
	uint16_t status = 0, DM;

	DM = VL53L1X_GetDistanceMode(dev);

	if (DM == 0)
		return 1;
	else if (DM == 1) { /* Short DistanceMode */

		switch (TimingBudgetInMs) {
		case 12:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001B);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0024);
			break;
		case 20:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0067);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0089);
			break;
		case 33:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00EC);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x019E);
			break;
		case 50:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01C8);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0286);
			break;
		case 100:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02DB);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0392);
			break;
		case 200:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03EE);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x049E);
			break;
		case 500:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x059C);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05CF);
			break;
		default:
			status = 1;
			break;
		}
	} else {
		switch (TimingBudgetInMs) {
		case 20:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0032);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x003A);
			break;
		case 33:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0071);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0081);
			break;
		case 50:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00C8);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00E5);
			break;
		case 100:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01DB);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01F1);
			break;
		case 200:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02EE);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
			break;
		case 500:
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x049C);
			WriteRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04B2);
			break;
		default:
			status = 1;
			break;
		}
	}
	return status;
}

uint16_t VL53L1X_GetTimingBudgetInMs(uint8_t dev) {
	uint16_t Temp, TimingBudget;

	Temp = ReadRegister16(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI);
	switch (Temp) {
	case 0x001B:
		TimingBudget = 12;
		break;
	case 0x0067:
	case 0x0032:
		TimingBudget = 20;
		break;
	case 0x00EC:
	case 0x0071:
		TimingBudget = 33;
		break;
	case 0x01C8:
	case 0x00C8:
		TimingBudget = 50;
		break;
	case 0x02DB:
	case 0x01DB:
		TimingBudget = 100;
		break;
	case 0x03EE:
	case 0x02EE:
		TimingBudget = 200;
		break;
	case 0x059C:
	case 0x049C:
		TimingBudget = 500;
		break;
	default:
		TimingBudget = 0;
		break;
	}
	return TimingBudget;

}

/* Set the DistanceMode (Short = 1, Long = 2) */
uint16_t VL53L1X_SetDistanceMode(uint8_t dev, uint16_t DM) {
	uint16_t TB, status = 1;

	TB = VL53L1X_GetTimingBudgetInMs(dev);
	switch (DM) {
	case 1:
		WriteRegister8(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
		WriteRegister8(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
		WriteRegister8(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
		WriteRegister8(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
		WriteRegister16(dev, SD_CONFIG__WOI_SD0, 0x0705);
		WriteRegister16(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
		status = 0;
		break;
	case 2:
		WriteRegister8(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
		WriteRegister8(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
		WriteRegister8(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
		WriteRegister8(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
		WriteRegister16(dev, SD_CONFIG__WOI_SD0, 0x0F0D);
		WriteRegister16(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
		status = 0;
		break;
	default:
		break;
	}
	VL53L1X_SetTimingBudgetInMs(dev, TB);
	return status;
}

uint16_t VL53L1X_GetDistanceMode(uint8_t dev) {
	uint16_t TempDM;
	uint16_t DM = 0;

	TempDM = ReadRegister8(dev, PHASECAL_CONFIG__TIMEOUT_MACROP);

	if (TempDM == 0x14)
		DM = 1;
	if (TempDM == 0x0A)
		DM = 2;

	return DM;
}

/* Set the InterMeasurement in ms */
uint16_t VL53L1X_SetInterMeasurementInMs(uint8_t dev, uint16_t InterMeasMs) {
	uint16_t ClockPLL;

	ClockPLL = ReadRegister16(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL) & 0x3FF;
	WriteRegister32(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, (uint32_t)(ClockPLL * InterMeasMs * 1.075));
	return 0;
}

uint16_t VL53L1X_GetInterMeasurementInMs(uint8_t dev) {
	uint16_t ClockPLL;
	uint32_t Temp;

	Temp = ReadRegister32(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD);
	ClockPLL = ReadRegister16(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL) & 0x3FF;
	return (uint16_t)(Temp / (ClockPLL * 1.065));
}

/*
 * Return chipId if booted, else return 0
 * ModelId of VL53L1X is 0xEACC
 */
uint16_t VL53L1X_BootState(uint8_t dev) {
	uint16_t id = 0;

	if ((ReadRegister8(dev, VL53L1_FIRMWARE__SYSTEM_STATUS) & 0x1) == 1)
		id = ReadRegister16(dev, VL53L1_IDENTIFICATION__MODEL_ID);
	return id;
}

/* Return the current sensor state (return 0 if sensor disabled) */
uint16_t VL53L1X_GetSensorState(uint8_t dev) {
	return (uint16_t) ReadRegister8(dev, VL53L1_FIRMWARE__SYSTEM_STATUS);
}

/* Get the distance in mm measured by the VL53L1X */
uint16_t VL53L1X_GetDistance(uint8_t dev) {
	return (ReadRegister16(dev,
	VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0));
}

/* Get the signal rate and SpadNb measured by the VL53L1X
 *  Return the SignalPerSpad in Kcps/spad
 */
uint16_t VL53L1X_GetSignalPerSpad(uint8_t dev) {
	uint16_t SignalRate, SpNb;

	SignalRate = ReadRegister16(dev,
	VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);
	SpNb = ReadRegister16(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
	return (uint16_t) (2000.0 * SignalRate / SpNb);
}

/* Get the ambient rate measured by the VL53L1X
 * Return the SignalPerSpad in Kcps/spad
 */
uint16_t VL53L1X_GetAmbientPerSpad(uint8_t dev) {
	uint16_t AmbientRate, SpNb;

	AmbientRate = ReadRegister16(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD);
	SpNb = ReadRegister16(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
	return (uint16_t) (2000.0 * AmbientRate / SpNb);
}

uint16_t VL53L1X_GetSignalRate(uint8_t dev) {
	return 8 * ReadRegister16(dev,
	VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);
}

/* Get the current number of SPADS of the VL53L1X */
uint16_t VL53L1X_GetSpadNb(uint8_t dev) {
	return ReadRegister16(dev,
	VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0) >> 8;
}

/* Get the approximate current ambient rate in Kcps	*/
uint16_t VL53L1X_GetAmbientRate(uint8_t dev) {
	return 8 * ReadRegister16(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD);
}

/* Read status of the measure and convert it to what is in the User Manual*/
uint16_t VL53L1X_GetRangeStatus(uint8_t dev) {
	uint16_t RgSt;

	RgSt = (uint16_t) ReadRegister8(dev, VL53L1_RESULT__RANGE_STATUS) & 0x1F;
	switch (RgSt) {
	case 9:
		RgSt = 0;
		break;
	case 6:
		RgSt = 1;
		break;
	case 4:
		RgSt = 2;
		break;
	case 8:
		RgSt = 3;
		break;
	case 5:
		RgSt = 4;
		break;
	case 3:
		RgSt = 5;
		break;
	case 19:
		RgSt = 6;
		break;
	case 7:
		RgSt = 7;
		break;
	case 12:
		RgSt = 9;
		break;
	case 18:
		RgSt = 10;
		break;
	case 22:
		RgSt = 11;
		break;
	case 23:
		RgSt = 12;
		break;
	case 13:
		RgSt = 13;
		break;
	default:
		RgSt = 255;
		break;
	}
	return (RgSt);
}

/* Send measured offset to device */
uint16_t VL53L1X_SetOffset(uint8_t dev, int16_t OffsetValue) {
	int16_t Temp;
	Temp = (OffsetValue * 4);
	WriteRegister16(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t) Temp);
	WriteRegister16(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
	WriteRegister16(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
	return 0;
}

int16_t VL53L1X_GetOffset(uint8_t dev) {
	int16_t Temp;

	Temp = ReadRegister16(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM);
	Temp = Temp << 3;
	Temp = Temp >> 5;
	return (int16_t) (Temp);
}

uint16_t VL53L1X_SetXtalk(uint8_t dev, uint16_t XtalkValue) {
	WriteRegister16(dev,
	ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000);
	WriteRegister16(dev, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000);
	WriteRegister16(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, XtalkValue);
	return 0;
}

uint16_t VL53L1X_GetXtalk(uint8_t dev) {
	return ReadRegister8(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS);
}

uint16_t VL53L1X_SetDistanceThreshold(uint8_t dev, uint16_t ThreshLow, uint16_t ThreshHigh, uint8_t Window, uint8_t IntOnNoTarget) {
	/*
	 * Example :
	 * dev,100,300,0 -> Just below 100
	 * dev,100,300,1 -> Just above 300
	 * dev,100,300,2 -> Out of window
	 * dev,100,300,3 -> In window
	 *
	 * If IntOnNoTarget = 0, don't interrupt on error
	 */

	uint8_t Temp = ReadRegister8(dev, SYSTEM__INTERRUPT_CONFIG_GPIO) & 0x47;

	if (IntOnNoTarget == 0) {
		WriteRegister8(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, (Temp | (Window & 0x07)));
	} else {
		WriteRegister8(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, ((Temp | (Window & 0x07)) | 0x40));
	}

	WriteRegister16(dev, SYSTEM__THRESH_HIGH, ThreshHigh);
	WriteRegister16(dev, SYSTEM__THRESH_LOW, ThreshLow);

	return 0;
}

uint16_t VL53L1X_GetDistanceThreshold_Window(uint8_t dev) {
	return (uint16_t) (ReadRegister8(dev, SYSTEM__INTERRUPT_CONFIG_GPIO) & 0x7);
}

uint16_t VL53L1X_GetDistanceThreshold_Low(uint8_t dev) {
	return ReadRegister16(dev, SYSTEM__THRESH_LOW);
}

uint16_t VL53L1X_GetDistanceThreshold_High(uint8_t dev) {
	return ReadRegister16(dev, SYSTEM__THRESH_HIGH);
}

/* All ROI are centered on the optical center if possible
 * If the ROI is too large, use the actual center
 * X and Y are the width and height of the ROI
 * WARNING: It is different from the other drivers
 */
uint16_t VL53L1X_SetROI(uint8_t dev, uint16_t X, uint16_t Y) {
	uint8_t OpticalCenter;

	OpticalCenter = ReadRegister8(dev, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD);
	if (X > 16)
		X = 16;
	if (Y > 16)
		Y = 16;
	if (X > 10 || Y > 10)
		OpticalCenter = 199;
	WriteRegister8(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
	WriteRegister8(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (Y - 1) << 4 | (X - 1));

	return 0;
}

uint16_t VL53L1X_GetROI_XY(uint8_t dev) {
	return (uint16_t) (ReadRegister8(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE));
}

/*
 * Set SignalRate in kcps
 */
uint16_t VL53L1X_SetSignalThreshold(uint8_t dev, uint16_t Signal) {
	WriteRegister16(dev, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, Signal >> 3);
	return 0;
}

uint16_t VL53L1X_GetSignalThreshold(uint8_t dev) {
	return (ReadRegister16(dev,
	RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS) << 3);
}

uint16_t VL53L1X_SetSigmaThreshold(uint8_t dev, uint16_t Sigma) {
	if (Sigma > (0xFFFF >> 2))
		return 1;
	WriteRegister16(dev, RANGE_CONFIG__SIGMA_THRESH, Sigma << 2);
	return 0;
}

uint16_t VL53L1X_GetSigmaThreshold(uint8_t dev) {
	return (ReadRegister16(dev, RANGE_CONFIG__SIGMA_THRESH) >> 2);
}

uint16_t VL53L1X_StartTemperature_Update(uint8_t dev) {
	WriteRegister8(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x31);
	return 0;
}

uint16_t VL53L1X_StopTemperature_Update(uint8_t dev) {
	WriteRegister8(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x11);
	return 0;
}

uint16_t VL53L1X_FastMode_10ms(uint8_t dev) {
	/* Must be finished ! */
	WriteRegister8(dev, 0x5E, 0x00);
	WriteRegister8(dev, 0x5F, 0x1F);
	WriteRegister8(dev, 0x61, 0x0);
	WriteRegister8(dev, 0x62, 0x00);
	WriteRegister8(dev, 0x43, 0x22);
	return 0;
}
