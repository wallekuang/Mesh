/**
******************************************************************************
* @file    bluenrg1_config.h
* @author  BLE Mesh Team
* @version V1.08.000
* @date    31-July-2018
* @brief   Header file for the BlueNRG-1 configuration 
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
#ifndef _BLUENRG1_CONFIG_H_
#define _BLUENRG1_CONFIG_H_

#include <bluenrg1_stack.h>
#include <bluenrg_mesh.h>

/* This file contains all the information needed to init the BlueNRG-1 stack. 
 * These constants and variables are used from the BlueNRG-1 stack to reserve RAM and FLASH 
 * according the application requests
 */

/*************All Values are as per BlueNRG MESH, Please don't change these values***********/
/* Default number of link */
#define MIN_NUM_LINK                3
/* Default number of GAP and GATT services */
#define DEFAULT_NUM_GATT_SERVICES   2
/* Default number of GAP and GATT attributes */
#define DEFAULT_NUM_GATT_ATTRIBUTES 9

/* Number of services requests from the beacon demo */
#define NUM_APP_GATT_SERVICES 1

/* Number of attributes requests from the beacon demo */
#define NUM_APP_GATT_ATTRIBUTES  6

/* Number of links needed for the demo: 1
 * Only 1 the default
 */
#define NUM_LINKS               (MIN_NUM_LINK)

/* Number of GATT attributes needed for the beacon demo. */
#define NUM_GATT_ATTRIBUTES     (DEFAULT_NUM_GATT_ATTRIBUTES + NUM_APP_GATT_ATTRIBUTES)

/* Number of GATT services needed for the beacon demo. */
#define NUM_GATT_SERVICES       (DEFAULT_NUM_GATT_SERVICES + NUM_APP_GATT_SERVICES)

/*************All Values are as per BLE MESH, Please don't change these values***********/

/* Array size for the attribute value for OTA service */
#if defined (ST_OTA_LOWER_APPLICATION) || defined (ST_OTA_HIGHER_APPLICATION)
#define OTA_ATT_VALUE_ARRAY_SIZE  (119)    /* OTA service is used: 4 characteristics (1 notify property) */
#else
#define OTA_ATT_VALUE_ARRAY_SIZE (0)       /* No OTA service is used */
#endif

/* Array size for the attribte value */
#define ATT_VALUE_ARRAY_SIZE (256)

/* Flash security database size */
#define FLASH_SEC_DB_SIZE       (0x400)


/* Flash server database size */

#define FLASH_SERVER_DB_SIZE    (0x400)

/* Maximum duration of the connection event */
#define MAX_CONN_EVENT_LENGTH 0xFFFFFFFF

/* Sleep clock accuracy in Slave mode 100 ppm */
#define SLAVE_SLEEP_CLOCK_ACCURACY 100

/* Sleep clock accuracy in Master mode 100 ppm */
#define MASTER_SLEEP_CLOCK_ACCURACY 3

/* Low Speed Oscillator source */
#if (LS_SOURCE == LS_SOURCE_INTERNAL_RO)
#define LOW_SPEED_SOURCE  1 /* Internal RO */
#else
#define LOW_SPEED_SOURCE  0 /* External 32 KHz */
#endif

/* High Speed start up time */
#define HS_STARTUP_TIME 0x150

/* Low level hardware configuration data for the device */
#define CONFIG_TABLE            \
{                               \
  NULL,                         \
  MAX_CONN_EVENT_LENGTH,        \
  SLAVE_SLEEP_CLOCK_ACCURACY,   \
  MASTER_SLEEP_CLOCK_ACCURACY,  \
  LOW_SPEED_SOURCE,             \
  HS_STARTUP_TIME               \
}

/* MACROS for Power Level definitions */
#define POWER_LEVEL_LOW            0 
#define TX_POWER_LEVEL_MINUS_18DBM 0 // = -18 dBm,
#define TX_POWER_LEVEL_MINUS_15DBM 1 // = -15 dBm,
#define TX_POWER_LEVEL_MINUS_12DBM 2 // = -12 dBm,
#define TX_POWER_LEVEL_MINUS_9DBM  3 // = -9 dBm,
#define TX_POWER_LEVEL_MINUS_6DBM  4 // = -6 dBm,
#define TX_POWER_LEVEL_MINUS_2DBM  5 // = -2 dBm,
#define TX_POWER_LEVEL_0DBM        6 // =  0 dBm,
#define TX_POWER_LEVEL_PLUS_5DBM   7 // =  5 dBm.
#define POWER_LEVEL_HIGH           1
#define TX_POWER_LEVEL_MINUS_14DBM 0 // = -14 dBm,
#define TX_POWER_LEVEL_MINUS_11DBM 1 // = -11 dBm,
#define TX_POWER_LEVEL_MINUS_8DBM  2 // = -8 dBm,
#define TX_POWER_LEVEL_MINUS_5DBM  3 // = -5 dBm,
//#define TX_POWER_LEVEL_MINUS_2DBM  4 // = -2 dBm,
#define TX_POWER_LEVEL_PLUS_2DBM   5 // =  2 dBm,
#define TX_POWER_LEVEL_PLUS_4DBM   6 // =  4 dBm,
#define TX_POWER_LEVEL_PLUS_8DBM   7 // =  8 dBm


#endif 
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

