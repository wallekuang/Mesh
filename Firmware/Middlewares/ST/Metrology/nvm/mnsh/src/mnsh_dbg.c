/**
*   @file      mnsh_dbg.c
*   @author    IPC - Industrial BU
*   @date      17 May 2016
*   @brief     Implements some various debug routines
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
#include <string.h>
#include "mnsh_rx.h"
#include "mnsh_dbg.h"
#include "nvram.h"
#include "handler_nvram.h"

/** @addtogroup GENERIC
  * @{
  */


/*******************************************************************************
* TYPES:
*******************************************************************************/
typedef enum
{
   CMD_INFO,
   CMD_METRO,
   CMD_ECHO,
   CMD_NVMRD,
   CMD_NVMWR,
   CMD_NVMSV,
   CMD_NVMER,   
   CMD_HELP
} CommandType;

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/
#define MNSH_MODULE_NAME      "dbg"

#define CHECK_NB_CMDARGS(pCommand, expectedNb)         \
   if (pCommand->nbData != expectedNb)                 \
   {                                                   \
      retCode = MNSHERR_WRONG_NB_DATA;                 \
      break;                                           \
   }

#define LONG_LITTLE_TO_BIG(a) ( (((uint32_t)a & 0xFF) << 24)        \
                              | (((uint32_t)a & 0xFF00) << 8)       \
                              | (((uint32_t)a & 0xFF0000) >> 8)     \
                              | (((uint32_t)a & 0xFF000000) >> 24))


#define HALF_LITTLE_TO_BIG(a) ( (((uint32_t)a & 0xFF) << 8)         \
                              | (((uint32_t)a & 0xFF00) >> 8))

const char stackName[][8] = {
  "MNSH",
  "BG",
  "METRO"
};

const char queueName[][8] = {
  "MNSH",
  "METRO"
};

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/
static const CommandProcessType dbgCmdTable[] =
{
   {"info",    CMD_INFO},
   {"metro",   CMD_METRO},
   {"echo",    CMD_ECHO},
   {"nvmrd",   CMD_NVMRD},
   {"nvmwr",   CMD_NVMWR},
   {"nvmsv",   CMD_NVMSV},
   {"nvmer",   CMD_NVMER},   
   {"help",    CMD_HELP},
   {"",        CMD_HELP},
   {"end",     COMMAND_END},
};

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/
/**
  * @brief      Command processor for the dbg module
  * @param[in]  pCmd: pointer on the command structure
  * @retval     the error code (0 if no error)
  */
MnshErrorType MNSH_DbgParser (CmdHandleType* pCmd)
{
   MnshErrorType retCode = MNSHERR_OK;
   uint32_t cmd;

   cmd = MNSH_GetCommandIdentifier(pCmd->cmdName,dbgCmdTable);
   switch (cmd)
   {
      case CMD_INFO:
         {
         /* Read and display a version identifier stored in Flash: */
         MNSH_DisplayInfo();
         }
         break;

         case CMD_ECHO:
         CHECK_NB_CMDARGS(pCmd,1);
         if (pCmd->data[0] == 0)
         {
           mnshVars.disableEcho = 1;
         }
         else
         {
           mnshVars.disableEcho = 0;
         }
         break;
      case CMD_METRO:
        {
          CHECK_NB_CMDARGS(pCmd,9);
          memcpy(mnshVars.buffer, pCmd->data, 9 * 4);
          mnshVars.lockRXNE = 1;
        }
        break;
      case CMD_NVMRD:
      case CMD_NVMWR:
      case CMD_NVMSV:
        {
          uint8_t *ptr;
          uint32_t size=0;

          nvmSave_t save=NVM_SAVE_ALL;
          switch (pCmd->data[0])
          {
            case NVRAM_GEN_ID:  ptr = (uint8_t *)nvm.tmpGen;  size = NVRAM_SIZE_GEN;  save = NVM_SAVE_GEN; break;
            case NVRAM_LEG_ID:  ptr = (uint8_t *)nvm.tmpLeg;  size = NVRAM_SIZE_LEG;  save = NVM_SAVE_LEG; break;
            default:
              MNSH_Printf("ID is 0/gen 3/LEG \n"); break;
          }
          if ((cmd == CMD_NVMRD) && (size != 0))            // case NVM read
          {
            CHECK_NB_CMDARGS(pCmd,1);
            for (int i=0; i<size; i++)
            {
              MNSH_Printf("%02X ", ptr[i]);
              if (((i&0xF) == 0xF) || (i == size-1))
                MNSH_Printf("\n");
            }
          }
          else if ((cmd == CMD_NVMWR) && (size != 0))       // case NVM write
          {
            CHECK_NB_CMDARGS(pCmd,4);
            uint32_t offset;
            offset = pCmd->data[1];
            ptr += offset;
            uint32_t data = pCmd->data[3];
            for (int i=0; i<pCmd->data[2]; i++)
            {
              if (offset++ < size)
              {
                uint8_t tmp;
                tmp = data & 0xFF;
                data >>= 8;

                *ptr++ = tmp;
              }
            }
          }
          else if ((cmd == CMD_NVMSV) && (save != 0))       // case NVM save
          {
            CHECK_NB_CMDARGS(pCmd,1);
            NVM_Write(save);
           }
        }
        break;
      case CMD_NVMER:
        {
          NVM_Erase();
        }
        break;
      case CMD_HELP:
         /*
         **           Maximum length is 80 bytes (to be well displayed in most teminal emulators):
         **           0---------1---------2---------3---------4---------5---------6---------7---------
         */
#ifdef ENABLE_MNSH_HELP
         MNSH_Printf("info    Display information on the HW and SW versions\n");
         MNSH_Printf("metro   primMin primMax secMin secMax voltMin voltMax frqMin frqMax gain\n");
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

