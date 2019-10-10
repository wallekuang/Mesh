#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

#define VL53L1_I2C_SLAVE_ADDR 0x29


int8_t VL53L1_WriteMulti(uint8_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_ReadMulti(uint8_t dev, uint16_t index, uint8_t *pdata, uint32_t count);

uint16_t ReadRegister32(uint8_t dev, uint16_t RegisterAdress);
uint16_t ReadRegister16(uint8_t dev, uint16_t registerAddr);
uint8_t ReadRegister8(uint8_t dev, uint16_t registerAddr);
void WriteRegister8(uint8_t dev, uint16_t registerAddr, uint8_t value);
void WriteRegister16(uint8_t dev, uint16_t RegisterAdress, uint16_t value);
void WriteRegister32(uint8_t dev, uint16_t RegisterAdress, uint32_t value);
void WaitMs(uint32_t TimeMs);

int rd_write_verification( uint8_t dev, uint16_t addr, uint32_t expected_value);
void i2c_test(uint8_t dev);

void WaitMs(uint32_t TimeMs);

#endif
