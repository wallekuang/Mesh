/**
*   @file       mnsh_tx.h
*   @author     IPC - Industrial BU
*   @date       15 september 2013
*   @brief      Defines the public interface for the main entry point for
*               minishell
*   @note       (C) COPYRIGHT 2013 STMicroelectronics
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

#ifndef MNSH_TX_H
#define MNSH_TX_H

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/

#include "st_device.h"

/** @addtogroup GENERIC
  * @{
  */


/*******************************************************************************
* MACROS & CONSTANTS:
*******************************************************************************/
#define APP_RX_DATA_SIZE        1024
#define APP_TX_DATA_SIZE        1024

#define SHELL_PRINTF_BUF_SIZE   (256)
#define SHELL_PRINTF_BUF1_SIZE  (256)
#define SHELL_PRINTF_MAXSIZE    (128)
#define SHELL_PRINTF_BUF_THR    (SHELL_PRINTF_BUF_SIZE - SHELL_PRINTF_MAXSIZE)
#define SHELL_PRINTF_BUF1_THR   (SHELL_PRINTF_BUF1_SIZE - SHELL_PRINTF_MAXSIZE)

#define MNSH_TIMER              (1)

/*******************************************************************************
* TYPES:
*******************************************************************************/
typedef enum
{
  X_MNSH_RX_EVENT       = 0x0100,
  X_MNSH_UNLOCKRX_EVENT,

  X_METRO_TICK          = 0x1000,
  X_METRO_KEY_P1,
  X_METRO_KEY_P2,
  X_METRO_UPDATE_DATA,
  X_METRO_CONFIG_INIT,
  X_METRO_CONFIG_REST,
  X_METRO_CONFIG_SAVE,
  X_METRO_PROD_TEST_PARAM,
  X_METRO_INIT,
  X_METRO_RST,
  X_METRO_SETUP,
  X_METRO_FACTOR,
  X_METRO_SW_RELEASE,
  X_METRO_PING,
  X_METRO_RD_REG,
  X_METRO_WR_REG,
  X_METRO_DEVICE,
  X_METRO_SAG_SWELL,
  X_METRO_CAL,
  X_METRO_METRO,
  X_METRO_IRQ,
  X_METRO_CLOSE,

  Undefined_Message = 0xFFFFFFFF
} msgId_t;

typedef struct
{
  msgId_t     id;
  uint32_t    payload[2];
} msg_t;

typedef struct
{
  uint8_t            rxData[APP_RX_DATA_SIZE];
  uint8_t            txData[APP_TX_DATA_SIZE];
  uint8_t            nbOfModules;
  uint8_t            lockRXNE;
  uint8_t            disableEcho;
  uint32_t           buffer[64];
  uint8_t            mnshTimer;
  msg_t              msg;  
  uint32_t           UserTxBufPtrIn ;
  uint32_t           UserTxBufPtrOut ; 
  uint32_t           UserRxBufPtr ; 
} mnshVars_t;

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/
extern uint8_t mnshTxBuf[SHELL_PRINTF_BUF_SIZE];
extern uint8_t mnshTxBuf1[SHELL_PRINTF_BUF1_SIZE];
extern mnshVars_t mnshVars;

/*******************************************************************************
* FUNCTIONS:
*******************************************************************************/
void MNSH_PutByte(uint8_t byte);
uint32_t MNSH_Printf(const char* format,...);

#endif /* MNSH_TX_H */

/**
  * @}
  */

/* End Of File */
