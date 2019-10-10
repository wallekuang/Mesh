/**
*   @file      handler_eeprom.h
*   @author    IPC - Industrial BU
*   @date      15 september 2013
*   @brief     Defines the public interface for the eeprom handler routines
*   @note      (C) COPYRIGHT 2013 STMicroelectronics
*
* @attention
*
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*/

#ifndef HANDLER_EEPROM_H
#define HANDLER_EEPROM_H

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/
#include <stdint.h>

/** @addtogroup GENERIC
  * @{
  */
#define MANUFACTURER_ADD                                        0x0000
#define MODELNAME_ADD                                           0x0014
#define SERIAL_NUMBER_ADD                                       0x0028
#define HARDWARE_VERSION_ADD	                                0x003C 
#define FIRMWARE_VERSION_ADD                                    0x0044
#define MANUFACTURING_LOCATION_ADD	                        0x004C
#define MANUFACTURING_DATE_ADD   	                        0x0060 

#define CALI_PARAMETERS_ADD                                     0x0068
#define OTHER_CALI_PARAMETERS_ADD                               0x0098
#define CURRENT_MONTH_ENRGY_CONSUMPTION_ADD                     0x0160
#define ENERGY_CUNSUMPTION_LAST_6Months_ADD                     0x0190
#define CUMULATIVE_ENERGY_OF_LINE_ADD                           0x02B8
#define CUMULATIVE_ENERGY_OF_EACH_PHASE_ADD                     0x02D0
#define MONTHLY_MAX_DEMAND_OF_LINE_6MONTHS_ADD                  0x0318
#define MONTHLY_MAX_DEMAND_OF_PHASES_6MONTHS_ADD                0x03A4 
#define MONTHLY_AVERAGE_PF_ADD                                  0x0548
/*                                    TAMPERS                        */
#define DETECTION_OF_MISSING_POTENTIAL_ALL_PHASE_ADD            0x0574
#define POWER_ON_OFF_ADD                                        0x079C  
#define TOP_COVER_OPEN_ADD                                      0x0854
#define NEUTRAL_DISTURBANCE_ADD                                 0x090C
#define ONLY_TWO_PHASE_ADD                                      0x09C4
#define WIRE_TRANSACTIONS_ADD                                   0x0A7C
#define REVERSE_PHASE_ENERGY_ADD                                0x0CA4
#define PHASE SEQUENCE REVERSAL_ADD                             0x0D5C
#define CURRENT_COIL_SHORTING_ADD                               0x0E14
#define LOW_VOLTAGE_EVENT_ALL_PHASES_ADD                        0x0ECC                
#define LOW_POWER_FACTOR_ALL_PHASE_ADD                          0x10F4
#define LOW_OR_HIGH_VOLTAGE_EVENT_ADD                           0x131C               
#define TAMPER_HISTORY_ADD                                      0x1554

#define DATE_STATUS_ADD                                         0x1624
#define TIME_STATUS_ADD                                         0x1625
#define DATE_ADD                                                0x1628
#define TIME_ADD                                                0x162C


/*******************************************************************************
* MACROS & CONSTANTS:
*******************************************************************************/
#define EEPROM_CMD_RD_MEMORY_ARAY       0xA0
#define EEPROM_CMD_WR_MEMORY_ARAY       0xA0
#define EEPROM_PAGE_SIZE                04


/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* FUNCTIONS:
*******************************************************************************/
//void EEPROM_Conf(void);
void EEPROM_Read(uint8_t *buf, uint32_t address, uint16_t size);
void EEPROM_Write(uint32_t address, uint8_t *buf, uint16_t size);
void EEPROM_STPM_Write(void);
void BIT_Date_Time_Single_conv(uint8_t *buff);
void BIT_E_conv(uint64_t  *active_64_t,uint64_t  *reactive_64_t,uint64_t  *apparent_64_t,uint8_t *buff);
void BIT_E_Single_conv(uint64_t  *power_factor,uint8_t *buff);
#endif /* HANDLER_EEPROM_H */

/** 
  * @}
  */

/* End Of File */

