#ifndef _API_H_
#define _API_H_
#pragma once

#include "BlueNRG1_conf.h"


uint16_t VL53L1X_GetRevision(void);
uint16_t VL53L1X_SetI2CAddress(uint16_t old_dev, uint16_t new_dev);
uint16_t VL53L1X_SensorInit(uint8_t dev);
uint16_t VL53L1X_ClearInterrupt(uint8_t dev);
uint16_t VL53L1X_SetInterruptPolarity(uint8_t dev, uint8_t Polarity);
uint16_t VL53L1X_GetInterruptPolarity(uint8_t dev);
uint16_t VL53L1X_StartRanging(uint8_t dev);
uint16_t VL53L1X_StopRanging(uint8_t dev);
uint16_t VL53L1X_CheckForDataReady(uint8_t dev);

uint16_t VL53L1X_SetTimingBudgetInMs(uint8_t dev, uint16_t TimingBudgetInMs);
uint16_t VL53L1X_GetTimingBudgetInMs(uint8_t dev);
uint16_t VL53L1X_SetDistanceMode(uint8_t dev, uint16_t DistanceMode);
uint16_t VL53L1X_GetDistanceMode(uint8_t dev);
uint16_t VL53L1X_SetInterMeasurementInMs(uint8_t dev,
					 uint16_t InterMeasurementInMs);
uint16_t VL53L1X_GetInterMeasurementInMs(uint8_t dev);
uint16_t VL53L1X_BootState(uint8_t dev);
uint16_t VL53L1X_GetSensorState(uint8_t dev);
uint16_t VL53L1X_GetDistance(uint8_t dev);
uint16_t VL53L1X_GetSignalPerSpad(uint8_t dev);
uint16_t VL53L1X_GetAmbientPerSpad(uint8_t dev);
uint16_t VL53L1X_GetSignalRate(uint8_t dev);
uint16_t VL53L1X_GetSpadNb(uint8_t dev);
uint16_t VL53L1X_GetAmbientRate(uint8_t dev);
uint16_t VL53L1X_GetRangeStatus(uint8_t dev);

uint16_t VL53L1X_SetOffset(uint8_t dev, int16_t OffsetValue);
int16_t VL53L1X_GetOffset(uint8_t dev);
uint16_t VL53L1X_SetXtalk(uint8_t dev, uint16_t XtalkValue);
uint16_t VL53L1X_GetXtalk(uint8_t dev);
uint16_t VL53L1X_SetDistanceThreshold(uint8_t dev, uint16_t ThreshLow,
			      uint16_t ThreshHigh, uint8_t Window,
			      uint8_t IntOnNoTarget);
uint16_t VL53L1X_GetDistanceThreshold_Window(uint8_t dev);
uint16_t VL53L1X_GetDistanceThreshold_Low(uint8_t dev);
uint16_t VL53L1X_GetDistanceThreshold_High(uint8_t dev);
uint16_t VL53L1X_SetROI(uint8_t dev, uint16_t X, uint16_t Y);
uint16_t VL53L1X_GetROI_XY(uint8_t dev);

uint16_t VL53L1X_SetSignalThreshold(uint8_t dev, uint16_t Signal);
uint16_t VL53L1X_GetSignalThreshold(uint8_t dev);
uint16_t VL53L1X_SetSigmaThreshold(uint8_t dev, uint16_t Sigma);
uint16_t VL53L1X_GetSigmaThreshold(uint8_t dev);
uint16_t VL53L1X_StartTemperature_Update(uint8_t dev);
uint16_t VL53L1X_StopTemperature_Update(uint8_t dev);
uint16_t VL53L1X_FastMode_10ms(uint8_t dev);

#endif
