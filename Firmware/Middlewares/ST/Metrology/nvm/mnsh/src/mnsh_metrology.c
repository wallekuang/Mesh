/**
*   @file      mnsh_metrology.c
*   @author    IPC - Industrial BU
*   @date      17 May 2016
*   @brief     Implements routines for metrology minishell module
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
#include "mnsh_metrology.h"
#include <string.h>

/** @addtogroup GENERIC
  * @{
  */

/*******************************************************************************
* TYPES:
*******************************************************************************/
typedef enum
{
   // old met commands
   CMD_CFG_SAVE,
   CMD_CFG_REST,
   CMD_CFG_INIT,
   // new met commands
   CMD_INIT,
   CMD_RST,
   CMD_SETUP,
   CMD_INFO,
   CMD_RD,
   CMD_WR,
   CMD_DEVICE,
   CMD_FACTOR,
   CMD_SAG_SWELL,
   CMD_CAL,
   CMD_METRO,
   CMD_IRQ,
   CMD_CLOSE,
   CMD_HELP,
   CMD_PING
} CommandType;

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/
#define MNSH_MODULE_NAME      "metro"

static const CommandProcessType dbgCmdTable[] =
{
   {"cfgsave", CMD_CFG_SAVE},
   {"cfgrest", CMD_CFG_REST},
   {"cfginit", CMD_CFG_INIT},

   {"init",    CMD_INIT},
   {"rst",     CMD_RST},
   {"setup",   CMD_SETUP},
   {"info",    CMD_INFO},
   {"rd",      CMD_RD},
   {"wr",      CMD_WR},
   {"device",  CMD_DEVICE},
   {"factor",  CMD_FACTOR},
   {"sgsw",    CMD_SAG_SWELL},
   {"cal",     CMD_CAL},
   {"metro",   CMD_METRO},
   {"irq",     CMD_IRQ},
   {"close",   CMD_CLOSE},
   {"help",    CMD_HELP},
   {"",        CMD_PING},
   {"end",     COMMAND_END}
};

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/

/**
  * @brief  Command processor for the metrology module
  * @param  pCmd: pointer on the command structure
  * @retval The error code (0 if no error)
  */
MnshErrorType MNSH_MetrologyParser (CmdHandleType* pCmd)
{
  MnshErrorType  retCode = MNSHERR_OK;

  switch (MNSH_GetCommandIdentifier(pCmd->cmdName,dbgCmdTable))
  {
    case CMD_CFG_SAVE:
      mnshVars.msg.id = X_METRO_CONFIG_SAVE;
      mnshVars.lockRXNE = 1;
      break;
    case CMD_CFG_REST:
      mnshVars.msg.id = X_METRO_CONFIG_REST;
      mnshVars.lockRXNE = 1;
      break;
    case CMD_CFG_INIT:
        mnshVars.msg.id = X_METRO_CONFIG_INIT;

      if (pCmd->nbData == 0)
      {
        mnshVars.msg.payload[0] = 1;
      }
      else if (pCmd->nbData == 1)
      {
        mnshVars.msg.payload[0] =  pCmd->data[0];
      }
      mnshVars.lockRXNE = 1;
      break;
    case CMD_INIT:
      CHECK_NB_CMDARGS(pCmd, 0)
      {
        mnshVars.msg.id = X_METRO_INIT;
        mnshVars.lockRXNE = 1;
      }
      break;
    case CMD_RST:
      CHECK_NB_CMDARGS(pCmd, 1)
      {
        mnshVars.msg.id = X_METRO_RST;
        mnshVars.msg.payload[0] =  pCmd->data[0];
        mnshVars.lockRXNE = 1;
      }
      break;
    case CMD_INFO:
      CHECK_NB_CMDARGS(pCmd, 0)
      {
        mnshVars.msg.id = X_METRO_SW_RELEASE;
        mnshVars.lockRXNE = 1;
      }
      break;
    case CMD_PING:
      CHECK_NB_CMDARGS(pCmd, 0)
      {
        mnshVars.msg.id = X_METRO_PING;
        mnshVars.lockRXNE = 1;
      }
      break;
    case CMD_SETUP:
      {
        memcpy(mnshVars.buffer, pCmd->data, pCmd->nbData*4);
        mnshVars.msg.id = X_METRO_SETUP;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      }
      break;
      case CMD_METRO:
      {
        memcpy(mnshVars.buffer, pCmd->data, pCmd->nbData*4);
        mnshVars.msg.id = X_METRO_METRO;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      }
      break;
     case CMD_DEVICE:
      {
        memcpy(mnshVars.buffer, pCmd->data, pCmd->nbData*4);
        mnshVars.msg.id = X_METRO_DEVICE;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      }
      break;
      case CMD_FACTOR:
      {
        memcpy(mnshVars.buffer, pCmd->data, pCmd->nbData*4);
        mnshVars.msg.id = X_METRO_FACTOR;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      }
      break;
      case CMD_IRQ:
      {
        memcpy(mnshVars.buffer, pCmd->data, pCmd->nbData*4);
        mnshVars.msg.id = X_METRO_IRQ;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      }
      break;
      case CMD_SAG_SWELL:
      {
        memcpy(mnshVars.buffer, pCmd->data, pCmd->nbData*4);
        mnshVars.msg.id = X_METRO_SAG_SWELL;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      }
      break;
      case CMD_CAL:
      {
        memcpy(mnshVars.buffer, pCmd->data, pCmd->nbData*4);
        mnshVars.msg.id = X_METRO_CAL;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      }
      break;
    case CMD_RD:
      CHECK_NB_CMDARGS(pCmd, 3)
      {
        memcpy(mnshVars.buffer, pCmd->data, 3 * 4);
        mnshVars.msg.id = X_METRO_RD_REG;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      }
      break;
    case CMD_WR:
        memcpy(mnshVars.buffer, pCmd->data, pCmd->nbData*4);
        mnshVars.msg.id = X_METRO_WR_REG;
        mnshVars.msg.payload[0] = (uint32_t)mnshVars.buffer;
        mnshVars.lockRXNE = 1;
      break;
    case CMD_CLOSE:
      CHECK_NB_CMDARGS(pCmd, 0)
      {
        mnshVars.msg.id = X_METRO_CLOSE;
        mnshVars.lockRXNE = 1;
      }
      break;
    case CMD_HELP:
      /*
      **           Maximum length is 80 bytes (to be well displayed in most teminal emulators):
      **           0---------1---------2---------3---------4---------5---------6---------7---------
      */
#ifdef ENABLE_MNSH_HELP
      MNSH_Printf("conf  -> init  Metrology Sub System\n");
      MNSH_Printf("reset -> reset Metrology Sub System\n");
      MNSH_Printf("write <reg_offset> <value> -> write 1 Metrology register\n");
      MNSH_Printf("read  <reg_offset>         -> read 1 Metrology register\n");
      MNSH_Printf("rdblk <reg_offset> <size>  -> read a block of Metrology registers\n");
      MNSH_Printf("cfginit <nb_phase> to restore default config\n");
      MNSH_Printf("cfgrest to restore config from NVM\n");
      MNSH_Printf("cfgsave to save config to NVM\n");
      MNSH_Printf("Note that reg_offset is mod 4 [0:4:0xB8]\n");

      MNSH_Printf("init  -> initializes Metrology Sub System\n");
      MNSH_Printf("rst    1 or 2 -> 1 : SYN reset hw, 2 SW_reset resets Metrology Sub System\n");
      MNSH_Printf("setup  0  <Host Config>  <STPM Config>  -> Set Metrology Config \n");
      MNSH_Printf("setup  1  ->  Get Metrology Config   \n");
      MNSH_Printf("rd  <device> <@> <Nb U32 Registers to read> -> Read Metrology registers Block \n");
      MNSH_Printf("wr <device> <@> <data1> <data2> .... -> Write Metrology registers block \n\n");
      MNSH_Printf("close  -> Close the Metrology driver and hardware block \n");
#else /* ENABLE_MNSH_HELP */
         MNSH_Default_Help;
#endif /* ENABLE_MNSH_HELP */
      break;
    default:
      retCode = MNSHERR_UNKNOWN_COMMAND;
      break;
   }
   return(retCode);
}

/*******************************************************************************
*
*                       IMPLEMENTATION: Private functions
*
*******************************************************************************/


/**
  * @}
  */

/* End Of File */

