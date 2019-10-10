/**
*   @file      mnsh_tx.h
*   @author    IPC - Industrial BU
*   @date      15 september 2013
*   @brief     Defines the private interface necessary to handle the minishell
*              (simple console connected to a serial port for test and debug
*               purposes).
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

#ifndef MNSH_RX_H
#define MNSH_RX_H

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/
#include "mnsh_tx.h"
#include "st_types.h"
#include "st_device.h"

/** @addtogroup GENERIC
  * @{
  */

/*******************************************************************************
* MACROS & CONSTANTS:
*******************************************************************************/
/* Constant values for command structure: */
#define MIN_MODNAME_SIZE        3
#define MAX_MODNAME_SIZE        8
#define MAX_COMMAND_SIZE        8
#define MAX_NBDATA              16
#define COMMAND_END             0xFF

#define SHELL_CMD_MAXSIZE       (128)

#define MNSH_Default_Help  MNSH_Printf("Help of mnsh disabled\n")

#define CHECK_NB_CMDARGS(pCommand, expectedNb)         \
   if (pCommand->nbData != expectedNb)                 \
   {                                                   \
      retCode = MNSHERR_WRONG_NB_DATA;                 \
      break;                                           \
   }
#define CHECK_MINMAX_NB_CMDARGS(pCommand, expectedMin, expectedMax) \
   if ((pCommand->nbData < expectedMin) || (pCommand->nbData > expectedMax))\
   {                                                   \
      retCode = MNSHERR_WRONG_NB_DATA;                 \
      break;                                           \
   }

/*******************************************************************************
* TYPES:
*******************************************************************************/
/* Definition of common error codes: */
typedef enum
{
   /* The first 2 values shall not be moved since they match the StatusType  */
   MNSHERR_OK = 0,
   MNSHERR_WRONG_NB_DATA,       /* if an unexpected nb of data is received   */
   MNSHERR_DATA_OUT_OF_RANGE,   /* if a parameter is out of its normal range */
   MNSHERR_TEST_FAILED,         /* if a test has failed                      */
   MNSHERR_UNKNOWN_COMMAND,     /* if the command is unknown                 */
   MNSHERR_UNKNOWN_MODULE,      /* if the module name is unknown             */
   MNSHERR_MAXNB_OF_ERROR_CODE
} MnshErrorType;

/* Definition of command type: */
typedef enum
{
   MNSH_DRIVER,
   MNSH_DEVICE_DRIVER,
   MNSH_APPLICATION,
   MNSH_TOOLS,
   MNSH_HELP
}MnshCommandType;

/* Definition of a parsed command structure: */
typedef struct
{
   uint8_t  modName[MAX_MODNAME_SIZE]; /* name of the module addressed by the cmd */
   uint8_t  cmdName[MAX_COMMAND_SIZE]; /* name of the command                     */
   uint32_t nbData;                    /* number of arguments                     */
   uint32_t data[MAX_NBDATA];          /* command's arguments                     */
} CmdHandleType;

/* Definition of the command name structure: */
typedef struct
{
   const uint8_t *cmdName;             /* command name */
   uint8_t cmdId;                      /* command Id   */
}CommandProcessType;

/* Definition of a module structure: */
typedef struct
{
   const uint8_t *modName;                         /* name of the module          */
   MnshErrorType (*cmdHandler)(CmdHandleType*); /* Module Entry-point function */
   MnshCommandType command;
} MnshModuleType;

/* Definition of the return codes: */
typedef enum
{
   MNSH_OK,
   MNSH_ERROR
}MnshStatusType;

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/
/* The following table is defined in a separate file and contains the supported
** module and their associated entry point:
*/
extern const MnshModuleType mnshModTable[];

/*******************************************************************************
* FUNCTIONS:
*******************************************************************************/
MnshErrorType MNSH_Help(CmdHandleType* pCmd);
uint32_t MNSH_GetCommandIdentifier(uint8_t *pCommand, const CommandProcessType *pCommandTable);

void MNSH_UsbRxHandler(void);
void MNSH_ModuleInit(void);
void MNSH_Prompt(void);
void MNSH_DisplayInfo(void);
bool MNSH_IsCommandDone(uint8_t Byte);
MnshStatusType MNSH_ProcessCommand(void);

#endif /* MNSH_RX_H */


/**
  * @}
  */

/* End Of File */
