/**
*   @file      mnshTask.c
*   @author    IPC - Industrial BU
*   @date      17 May 2016
*   @brief     This source code includes Minishell task related functions
*   @note      (C) COPYRIGHT 2013 STMicroelectronics
*
* @attention
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
#include "mnshTask.h"
#include "mnsh_rx.h"
#include "st_device.h"

/** @addtogroup GENERIC
  * @{
  */

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/

/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/

/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/

/**
  * @brief  This function implements the Minishell init
  * @param  None
  * @retval None
  */
void MNSH_TaskInit(void)
{
  MNSH_ModuleInit();
  MNSH_DisplayInfo();
  MNSH_Prompt();
}

/**
  * @brief  This function implements the Minishell task
  * @param  None
  * @retval None
  */
void MNSH_Task(void)
{
    switch (mnshVars.msg.id)
    {
    case X_MNSH_RX_EVENT:
      {
        MNSH_ProcessCommand();
        
        if (mnshVars.lockRXNE == 0)
        {    
          if (mnshVars.disableEcho == 0)
          {
          MNSH_Prompt();
          }
          mnshVars.msg.id = (msgId_t)0; 
        }
      }
      break;

    case X_MNSH_UNLOCKRX_EVENT:
      mnshVars.lockRXNE = 0;
      mnshVars.msg.id = (msgId_t)0;
      MNSH_Prompt();
      break;
    
    default:
    
      break;
    }
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
