
#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_
#pragma once


int16_t VL53L1X_CalibrateOffset(uint16_t dev, uint16_t RealDistInMm);
uint16_t VL53L1X_CalibrateXtalk(uint16_t dev, uint16_t RealDistInMm);

#endif
