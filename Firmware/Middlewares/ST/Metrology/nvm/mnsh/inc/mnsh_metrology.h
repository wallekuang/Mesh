/**
*   @file       mnsh_metrology.h
*   @author     IPC - Industrial BU
*   @date       15 september 2013
*   @brief      Defines the public interface for the metrology minishell 
*               routines
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

#ifndef MNSH_METROLOGY_H
#define MNSH_METROLOGY_H

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/
#include "mnsh_rx.h"

/** @addtogroup GENERIC
  * @{
  */

/*******************************************************************************
* MACROS & CONSTANTS:
*******************************************************************************/

/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* FUNCTIONS:
*******************************************************************************/
MnshErrorType MNSH_MetrologyParser (CmdHandleType* pCmd);

#endif /* MNSH_GPIO_H */

/** 
  * @}
  */

/* End Of File */
