/**
*   @file      mnsh_eeprom.c
*   @author    IPC - Industrial BU
*   @date      17 May 2016
*   @brief     Implements routines to handle EEPROM
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
#include "mnsh_eeprom.h"
#include "handler_eeprom.h"

/** @addtogroup GENERIC
  * @{
  */


/*******************************************************************************
* TYPES:
*******************************************************************************/
typedef enum
{
   CMD_CONF,
   CMD_READ,
   CMD_WRITE,
   CMD_HELP
} CommandType;

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/
#define MNSH_MODULE_NAME                "eeprom"

static const CommandProcessType dbgCmdTable[] =
{
   {"cfg",     CMD_CONF},
   {"read",    CMD_READ},
   {"write",   CMD_WRITE},
   {"help",    CMD_HELP},
   {"",        CMD_HELP},
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
  * @brief      Command processor for the eeprom module
  * @param[in]  pCmd: pointer on the command structure
  * @retval     the error code (0 if no error)
  */
MnshErrorType MNSH_EepromParser (CmdHandleType* pCmd)
{
   MnshErrorType retCode = MNSHERR_OK;

   switch (MNSH_GetCommandIdentifier(pCmd->cmdName,dbgCmdTable))
   {
      case CMD_CONF:
         CHECK_NB_CMDARGS(pCmd, 0)
         {
           //EEPROM_Conf();
           MNSH_Printf("I2C_EEPROM initialized for access to EEPROM\n");
         }
         break;
      case CMD_READ:
         CHECK_NB_CMDARGS(pCmd, 2)
         {
           uint8_t *ptr;
           ptr = (uint8_t *)mnshVars.buffer;
           EEPROM_Read(ptr, pCmd->data[0], EEPROM_PAGE_SIZE);
           if (pCmd->data[1])
           {
             int i;
             for (i=0; i< EEPROM_PAGE_SIZE; i++)
             {
               MNSH_Printf("%02x ", ptr[i]);
               if ((i & 0xF) == 0xF)
               {
                 MNSH_Printf("\n");
               }
             }
           }
           else
           {
             MNSH_Printf("%02x\n", ptr[0]);
           }
         }
         break;
      case CMD_WRITE:
         CHECK_NB_CMDARGS(pCmd, 3)
         {
           int i;
           uint8_t *ptr;
           ptr = (uint8_t *)mnshVars.buffer;

           for (i=0; i< EEPROM_PAGE_SIZE; i+=4)
           {
             ptr[i+0] = (uint8_t)(pCmd->data[1] >> 0);
             ptr[i+1] = (uint8_t)(pCmd->data[1] >> 8);
             ptr[i+2] = (uint8_t)(pCmd->data[1] >> 16);
             ptr[i+3] = (uint8_t)(pCmd->data[1] >> 24);
           }
           if (pCmd->data[2])
           {
              EEPROM_Write(pCmd->data[0], ptr, EEPROM_PAGE_SIZE);
           }
           else
           {
              EEPROM_Write(pCmd->data[0], ptr, 1);
           }
         }
         break;
      case CMD_HELP:
         /*
         **           Maximum length is 80 bytes (to be well displayed in most teminal emulators):
         **           0---------1---------2---------3---------4---------5---------6---------7---------
         */
#ifdef ENABLE_MNSH_HELP
         MNSH_Printf("cfg   to activate EEPROM access\n");
         MNSH_Printf("read  <add> <0=byte-1=page>\n");
         MNSH_Printf("write <add> <data> <0=byte-1=page>\n");
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
