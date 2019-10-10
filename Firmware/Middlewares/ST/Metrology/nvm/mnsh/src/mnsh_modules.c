/**
*   @file      mnsh_modules.c
*   @author    IPC - Industrial BU
*   @date      17 May 2016
*   @brief     Define the global array of minishell modules.
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
#include "st_types.h"
#include "mnsh_rx.h"
/* Following include files export entrypoint functions for each module: */
#include "mnsh_dbg.h"
#include "mnsh_metrology.h"
#include "mnsh_eeprom.h"

/** @addtogroup GENERIC
  * @{
  */


__weak MnshErrorType MNSH_MetrologyParser (CmdHandleType* pCmd);
__weak MnshErrorType MNSH_EepromParser (CmdHandleType* pCmd);


/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/

/* The following table contains the supported commands: */
const MnshModuleType mnshModTable[]= {

   /***********************************************************
   * "Platform Driver" modules
   ***********************************************************/
   { "met",        MNSH_MetrologyParser, MNSH_DRIVER           },

   /***********************************************************
   * Application modules
   ***********************************************************/
#ifdef EEPROM_PRESENT
   { "eeprom",     MNSH_EepromParser   , MNSH_DEVICE_DRIVER    },
#endif 
   /***********************************************************
   * Help module
   ***********************************************************/
   { "help",       MNSH_Help          , MNSH_HELP             },
      
   /***********************************************************
   * Tools module
   ***********************************************************/
   { "dbg",        MNSH_DbgParser      , MNSH_TOOLS            }
  

};

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
void MNSH_ModuleInit(void)
{
  mnshVars.nbOfModules = (sizeof (mnshModTable) / sizeof(MnshModuleType));
}

/*******************************************************************************
*
*                       IMPLEMENTATION: Local functions
*
*******************************************************************************/
__weak MnshErrorType MNSH_MetrologyParser (CmdHandleType* pCmd)
{
  return(MNSHERR_UNKNOWN_MODULE);
}
__weak MnshErrorType MNSH_EepromParser (CmdHandleType* pCmd)
{
  return(MNSHERR_UNKNOWN_MODULE);
}

/**
  * @}
  */

/* End Of File */
