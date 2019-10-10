/**
*   @file      mnsh_tx.c
*   @author    IPC - Industrial BU
*   @date      17 May 2016
*   @brief     Start-up code in the case of the Standalone Minishell
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

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/
#include "st_device.h"
#include "mnsh_rx.h"
#include "st_device.h"
#include "usbd_cdc.h"
#include <stdarg.h>
#include <string.h>
#include <stdint.h>


/** @addtogroup GENERIC
* @{
*/

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/
#define WAIT_DURATION       2

/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/
uint8_t mnshTxBuf[SHELL_PRINTF_BUF_SIZE];
mnshVars_t mnshVars;

extern USBD_HandleTypeDef hUsbDeviceFS;

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/
static void MNSH_WaitBufferSpace(void);

/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/

/**
* @brief      This function trigs the transmission of a byte on the UART link
* @param[in]  byte: byte to transmit
* @retval     None
*/
void MNSH_PutByte(uint8_t byte)
{
  /* Increment Index for buffer writing */
  mnshVars.txData[mnshVars.UserTxBufPtrIn] = byte;
  mnshVars.UserTxBufPtrIn++;
  
  /* To avoid buffer overflow */
  if(mnshVars.UserTxBufPtrIn == APP_TX_DATA_SIZE)
  {
    mnshVars.UserTxBufPtrIn = 0;
  }
}

/**
* @brief      This function trigs the transmission of a string over the UART
*             for printing
* @param[in]  Format: string with formatting
* @param[in]  Optional arguments to fit with formatting
* @retval     Lengthj of the string to print (uint32_t)
*/
uint32_t MNSH_Printf(const char* format,...)
{
  uint8_t buffer[SHELL_PRINTF_MAXSIZE];
  va_list args;
  uint32_t size, retSize;

  // the string to transmit is copied in a temporary buffer
  va_start(args, format);
  size=vsprintf((char *)buffer, (const char*)format, args);
  va_end(args);
  retSize = size;
  if ( size >= SHELL_PRINTF_MAXSIZE )
    while (1);
  if (buffer[size-1] == '\n')
  {
    buffer[size-1] = '\r';
    buffer[size] = '\n';
    size++;
  }

      
  MNSH_WaitBufferSpace();
  
  for (int i=0;i<size;i++)
  {
    mnshVars.txData[mnshVars.UserTxBufPtrIn + i] = buffer[i];
  }
 
  mnshVars.UserTxBufPtrIn += size;

  return(retSize);
}


/*******************************************************************************
*
*                       IMPLEMENTATION: Private functions
*
*******************************************************************************/


/**
  * @brief  This function handles wait
  * @param  time : time to wait in microseconds
  * @retval None
  */
static void MNSH_WaitBufferSpace(void)
{
  while(mnshVars.UserTxBufPtrIn > (APP_TX_DATA_SIZE - SHELL_PRINTF_MAXSIZE))
    ;
}

/**
* @}
*/

/* End Of File */
