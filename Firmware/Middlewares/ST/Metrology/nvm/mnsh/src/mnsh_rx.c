/**
*   @file      mnsh_rx.c
*   @author    IPC - Industrial BU
*   @date      17 May 2016
*   @brief     Implements the main engine of the minishell (simple console
*              connected to a serial port for test and debug purposes).
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
#include "mnshTask.h"

#include <stdlib.h>
#include <string.h>

/** @addtogroup GENERIC
  * @{
  */


/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/
#define ST_PROMPT                "STM32>" 
#define SHELL_COMMAND_LIST_SIZE  2
#define CR_ASCII_CODE            13   /* Carriage Return */
#define LF_ASCII_CODE            10   /* Line Feed       */
#define ESC_ASCII_CODE           0x1B   /* Escace       */
#define HELP_SPACE               (12)
#define HELP_OFFSET_1            (7)
#define HELP_OFFSET_2            (HELP_OFFSET_1+HELP_SPACE)
#define HELP_OFFSET_3            (HELP_OFFSET_2+HELP_SPACE)

#define SHELLMODTABLE_SIZE       (mnshVars.nbOfModules)

/* Local Debug Compilation switch: */
//#define MINISHELL_DEBUG
#define CONFIG_REMOTE_ECHO

/* Comment the following switch to save some data space: */
#define MNSH_ERROR_VERBOSE

#define SHELL_NEXT_MESSAGE(message)          \
   message++;                                \
   if (message == SHELL_COMMAND_LIST_SIZE)   \
   {                                         \
      message = 0;                           \
   }
#define SHELL_PREVIOUS_MESSAGE(message)      \
   if (message == 0)                         \
   {                                         \
      message = SHELL_COMMAND_LIST_SIZE;     \
   }                                         \
   message--;

/*******************************************************************************
* TYPES:
*******************************************************************************/
typedef enum
{
   RECEIVING_DATA,
   RECEIVING_LF,
   RECEIVING_ESC,
   RECEIVING_BRACKET,
   RECEIVING_ARROW,
   END_OF_COMMAND
} MnshCmdStatusType;

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/
static void   MNSH_BlankLine(void);
static uint32_t    MNSH_ParseCommand(uint8_t* pCmd, uint32_t cmdSize, CmdHandleType* hCmd);
static void   MNSH_ExtractCmdArgument(uint8_t* pCmd, CmdHandleType* hCmd,
                                      uint32_t argLength, uint32_t argId);


/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/
static uint8_t mnshCmd[SHELL_CMD_MAXSIZE];
static uint8_t mnshCmdList[SHELL_COMMAND_LIST_SIZE][SHELL_CMD_MAXSIZE];
static uint32_t mnshRxNbBytes = 0;
static MnshCmdStatusType cmdStatus = RECEIVING_DATA;
static const uint8_t mnshErrorMessage[MNSHERR_MAXNB_OF_ERROR_CODE-1][20] =
{
   /*
   BE CAREFUL: Maximum message length is 20 bytes !
   "0---------1---------2---------"
   */
   "wrong nb of data",
   "data out of range",
   "test failed",
   "unknown command",
   "unknown module",
};

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/
/**
  * @brief      handles RX data
  * @param      None
  * @retval     None
  */
void MNSH_UsbRxHandler(void)
{
  /* Send a message on the queue to flag that a char has been read.         */
  uint8_t i;
  
  i=0;

  while  (i !=mnshVars.UserRxBufPtr)
  {
    
    if (MNSH_IsCommandDone(mnshVars.rxData[i]))
    {
      mnshVars.msg.id = X_MNSH_RX_EVENT;
    }
     
    mnshVars.rxData[i]=0;  
    i=i+1;
  }


  
}

/**
  * @brief      This function waits for a valid mnsh command and processes it
  * @param[in]  None
  * @retval     status of type MnshStatusType: MNSH_OK or MNSH_KO
  */
MnshStatusType MNSH_ProcessCommand(void)
{
   CmdHandleType handle;
   uint32_t i;
   uint32_t nbArgs  = 0;
   MnshErrorType code = MNSHERR_UNKNOWN_MODULE;
   MnshStatusType status = MNSH_OK;

   /*
   ** Parse the received command:
   */
   nbArgs = MNSH_ParseCommand(mnshCmd, mnshRxNbBytes, &handle);

   /*
   ** Call the command handler:
   */
   if (nbArgs != 0)
   {
      for (i=0; i<SHELLMODTABLE_SIZE;i++)
      {
         if (strcmp((const char*)mnshModTable[i].modName,
                    (const char*)handle.modName) == 0)
         {
            code = mnshModTable[i].cmdHandler(&handle);
            break;
         }
      }

      if (code != MNSHERR_OK)
      {
#ifdef MNSH_ERROR_VERBOSE
         MNSH_Printf("SHELL_ERROR: %s\n", mnshErrorMessage[code-1]);
#else
         MNSH_Printf("SHELL_ERROR: %d\n", code);
#endif
        status = MNSH_ERROR;
      }
   }
   mnshRxNbBytes = 0;
   return(status);
}

/**
  * @brief      This function returns the selected command in the chosen module
  * @param[in]  pCommand String with the command
  * @param[in]  pCommandTable table with all the available commands
  * @retval     id of the command
  */
uint32_t MNSH_GetCommandIdentifier(uint8_t *pCommand, const CommandProcessType *pCommandTable)
{
   CommandProcessType *pCmdTable = (CommandProcessType *)pCommandTable;
   uint8_t *pCurrentCommand = pCommand;
   const uint8_t *pcmdName = pCmdTable->cmdName;

   while (pCmdTable->cmdId != COMMAND_END)
   {
      while ((*pcmdName != 0) && (*pCurrentCommand == *pcmdName))
      {
         pCurrentCommand++;
         pcmdName++;
      }

      if ((*pCurrentCommand == 0) && (*pcmdName == 0))
      {
         break;
      }
      pCmdTable++;
      pCurrentCommand = pCommand;
      pcmdName = pCmdTable->cmdName;
   }
   return(pCmdTable->cmdId);
}

/**
  * @brief      This function displays the help corresponding to the command
  *             in input argument
  * @param[in]  command enum of type MnshCommandType
  * @retval     none
  */
void MNSH_HelpDisplay(MnshCommandType command)
{
   uint32_t i = 0;
   uint32_t j = 0;
   uint32_t column = 0;

   for (i=0; i<SHELLMODTABLE_SIZE; i++)
   {
      if (mnshModTable[i].command == command)
      {
         switch(column)
         {
         case 0 :
            for (j=0; j<HELP_OFFSET_1;j++)
               MNSH_Printf(" ");
            MNSH_Printf("%s", mnshModTable[i].modName);
            for (j+=strlen((const char *)(mnshModTable[i].modName)); j<HELP_OFFSET_2;j++)
               MNSH_Printf(" ");
            column++;
            break;
         case 1 :
            MNSH_Printf("%s", mnshModTable[i].modName);
            for (j+=strlen((const char *)(mnshModTable[i].modName)); j<HELP_OFFSET_3;j++)
               MNSH_Printf(" ");
            column++;
            break;
         case 2 :
            MNSH_Printf("%s\n", mnshModTable[i].modName);
            column=0;
            break;
         }
      }
   }
   if (column != 0)
   {
      MNSH_Printf("\n");
   }
}

/**
  * @brief      This function returns the help corresponding to the command
  *             in input argument
  * @param[in]  pCmd string with the command for which the help shall be printed
  * @retval     status if the mnsh command is known or unknown
  */
MnshErrorType MNSH_Help(CmdHandleType* pCmd)
{
   MNSH_Printf("Driver modules:\n");
   MNSH_HelpDisplay(MNSH_DRIVER);

   MNSH_Printf("\nApplication modules:\n");
   MNSH_HelpDisplay(MNSH_DEVICE_DRIVER);

   MNSH_Printf("\nTool modules:\n");
   MNSH_HelpDisplay(MNSH_TOOLS);

   return(MNSHERR_OK);
}

/**
  * @brief      This function prints the HW version
  * @param[in]  none
  * @retval     none
  */
void MNSH_DisplayInfo (void)
{
  MNSH_Printf("\n------------------------------------\n");
  MNSH_Printf("PLATFORM TYPE     : %02X\n", PLTF_STM32_METER);
    
  MNSH_Printf("SW VERSION        : V%d.%d - patch %d\n", MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION);
  MNSH_Printf("                 \n");

}

/**
  * @brief      Processes the latest character received on the UART
  * @param[in]  Byte: latest character received from the UART
  * @retval     bool: true when the whole command has been received
  */
bool MNSH_IsCommandDone(uint8_t Byte)
{
   static uint32_t mnshCmdListIndex = 0;
   static uint32_t mnshCurrentCmd = 0;
   uint32_t currentCmd;
   uint32_t *nbBytes = &mnshRxNbBytes;

   mnshCmd[*nbBytes] = Byte;

   switch (cmdStatus)
   {
   case RECEIVING_DATA :
     if (mnshCmd[*nbBytes] == ESC_ASCII_CODE)
     {
       cmdStatus = RECEIVING_BRACKET;
       break;
     }

     if (mnshCmd[*nbBytes] == 0)
     {
       break;
     }

#ifdef CONFIG_REMOTE_ECHO
     if (mnshVars.disableEcho == 0)
     {
       MNSH_PutByte(mnshCmd[*nbBytes]);
     }
#endif /* CONFIG_REMOTE_ECHO */

     /*
     ** Manage the CR and/or LF detection:
     */
     if (mnshCmd[*nbBytes] == CR_ASCII_CODE)
     {
       cmdStatus = END_OF_COMMAND;
       break;
     }

     if (mnshCmd[*nbBytes] == LF_ASCII_CODE)
     {
       cmdStatus = END_OF_COMMAND;
       break;
     }

     if (mnshCmd[*nbBytes] != '\b')
     {
       (*nbBytes)++;
     }
     /* Manage the backspace character: */
     else
     {
       if (*nbBytes>0)
       {
         (*nbBytes)--;
#ifdef CONFIG_REMOTE_ECHO
         MNSH_PutByte(' ');
         MNSH_PutByte('\b');
#endif /* CONFIG_REMOTE_ECHO */
       }
     }
     break;
   case RECEIVING_BRACKET :
     /* Process esc sequence */
     if (mnshCmd[*nbBytes] == '[')
     {
       cmdStatus = RECEIVING_ARROW;
     }
     else
     {
       cmdStatus = RECEIVING_DATA;
     }
     break;
   case RECEIVING_ARROW :
     switch (mnshCmd[*nbBytes])
     {
     case 'A' : /* up */
       MNSH_BlankLine();
       MNSH_Prompt();
       SHELL_PREVIOUS_MESSAGE(mnshCurrentCmd);
       memcpy(mnshCmd,mnshCmdList[mnshCurrentCmd],SHELL_CMD_MAXSIZE);
       for (*nbBytes=0;
            ((mnshCmd[*nbBytes] != LF_ASCII_CODE) && (mnshCmd[*nbBytes] != CR_ASCII_CODE) && (mnshCmd[*nbBytes] != 0));
            (*nbBytes)++);
       mnshCmd[*nbBytes] = '\0';
       if (mnshCmd[0]!=0)
       {
         MNSH_Printf((char *)mnshCmd);
       }
       break;
     case 'B' : /* down */
       MNSH_BlankLine();
       MNSH_Prompt();
       SHELL_NEXT_MESSAGE(mnshCurrentCmd);
       memcpy(mnshCmd,mnshCmdList[mnshCurrentCmd],SHELL_CMD_MAXSIZE);
       for (*nbBytes=0;
            ((mnshCmd[*nbBytes] != LF_ASCII_CODE) && (mnshCmd[*nbBytes] != CR_ASCII_CODE) && (mnshCmd[*nbBytes] != 0));
            (*nbBytes)++);
       mnshCmd[*nbBytes] = '\0';
       if (mnshCmd[0]!=0)
       {
         MNSH_Printf((char *)mnshCmd);
       }
       break;
     case 'C' : /* right */
       break;
     case 'D' : /* left */
       break;
     }
     cmdStatus = RECEIVING_DATA;
     break;
   }

   if ((*nbBytes==(SHELL_CMD_MAXSIZE-1)) || (cmdStatus == END_OF_COMMAND))
   {
     /* if command != cr/lf */
     if (*nbBytes != 0)
     {
       currentCmd = mnshCmdListIndex;
       SHELL_PREVIOUS_MESSAGE(currentCmd);
       /* if new command save it */
       if (strcmp((const char*)mnshCmdList[currentCmd],(const char*)mnshCmd) != 0)
       {
         memcpy(mnshCmdList[mnshCmdListIndex],mnshCmd,SHELL_CMD_MAXSIZE);
         SHELL_NEXT_MESSAGE(mnshCmdListIndex);
       }
     }
     mnshCurrentCmd = mnshCmdListIndex;
     cmdStatus = RECEIVING_DATA;
     mnshCmd[(*nbBytes) + 1] = 0;
     return(true);
   }
   else
   {
     return(false);
   }
}

/**
  * @brief      Sends the prompt for printing
  * @param[in]   none
  * @retval     none
  */
void MNSH_Prompt(void)
{
   const char prompt[] = ST_PROMPT;

   MNSH_Printf(prompt);
}


/*******************************************************************************
*
*                       IMPLEMENTATION: Private functions
*
*******************************************************************************/

/**
  * @brief  Prints a blank line
  * @param  None
  * @retval None
  */
static void MNSH_BlankLine(void)
{
   /*                0        1         2         3         4         5         6         7         8 */
   /*                1        0         0         0         0         0         0         0         0 */
   char blank[] = "\r                                                                                \r";

   MNSH_Printf(blank);
}

/**
  * @brief      This function parses the mnsh command
  * @param[out] pCmd string containing the module, the command and the arguments
  * @param[in]  cmdSize
  * @param[in]  hCmd handle of the command containing the module name, command
  *             and arguments
  * @retval     nbArgs number of the arguments received with the command
  */
static uint32_t MNSH_ParseCommand(uint8_t* pCmd, uint32_t cmdSize, CmdHandleType* hCmd)
{
   uint32_t startIndex = 0;
   uint32_t nbArgs     = 0;
   uint32_t i          = 0;
   uint32_t foundArg   = 0;

   /* Initialize command's fields: */
   hCmd->modName[0] = '\0'; /* set the NULL string terminator on 1st position */
   hCmd->cmdName[0] = '\0'; /* set the NULL string terminator on 1st position */
   hCmd->nbData     = 0;

#ifdef MINISHELL_DEBUG
   /*
   ** Acknowledge the received command by displaying it back:
   */
   MNSH_Printf("%d bytes received\n",cmdSize);
   if (cmdSize<SHELL_CMD_MAXSIZE)
   {
      mnshCmd[cmdSize] = 0; /* Add the Null terminator: */
   }
   else
   {
      mnshCmd[SHELL_CMD_MAXSIZE-1] = 0;
   }
   if (cmdSize != 0)
   {
      MNSH_Printf("received command is: %s\n",mnshCmd);
   }
#endif /* MINISHELL_DEBUG */

   /*
   ** Parse the command string and convert the 2 first arguments into module
   ** and command and the subsequent arguments into numerical parameters:
   */
   for (i=0;i<cmdSize;i++)
   {
      /* Detect the start of a comment (through a semi-column character): */
      if (pCmd[i] == ';')
      {
         break;
      }

      /* arguments shall be separated by space and tabs characters */
      if ((pCmd[i] != ' ') && (pCmd[i] != '\t'))
      {
         if (foundArg == 0)
         {
            foundArg = 1;
            startIndex = i;
            nbArgs++;
         }
      }
      else
      {
         if (foundArg != 0)
         {
            MNSH_ExtractCmdArgument(pCmd+startIndex, hCmd,i-startIndex,nbArgs);
            foundArg = 0;
         }
      }
   }
   /* Case of the last argument not necessary followed by a white character */
   if (foundArg != 0)
   {
      MNSH_ExtractCmdArgument(pCmd+startIndex, hCmd, i-startIndex, nbArgs);
   }

#ifdef MINISHELL_DEBUG
   if (nbArgs != 0)
   {
      MNSH_Printf("%d argument(s) found\n",nbArgs);
      MNSH_Printf("module name: %s\ncommand name: %s\n",hCmd->modName, hCmd->cmdName);
      MNSH_Printf("command has %d numerical field(s)\n",hCmd->nbData);
      for (i=0;i<hCmd->nbData; i++)
      {
         MNSH_Printf("\t%d\n", hCmd->data[i]);
      }
   }
#endif /* MINISHELL_DEBUG */

   return(nbArgs);
}

/**
  * @brief      This function parses the mnsh command
  * @param[out]  pCmd string with the command for which the help shall be printed
  * @param[in]  hCmd handle on the command
  * @param[in]  argLength number of arguments
  * @param[in]  argId Id of the argument to treat (1: module, 2: command)
  * @retval     None
  */
static void MNSH_ExtractCmdArgument(uint8_t* pCmd, CmdHandleType* hCmd,
                                    uint32_t argLength, uint32_t argId)
{
   uint32_t length = 0;
   switch(argId)
   {
      case 1:
         /* Module name */
         length = MIN_VALUE(MAX_MODNAME_SIZE-1,argLength);
         strncpy((char *)hCmd->modName,
                  (const char *)pCmd,
                  (unsigned int)length);
         hCmd->modName[length] = '\0'; /* add the null terminator */
         break;

      case 2:
         /* Command name */
         length = MIN_VALUE(MAX_COMMAND_SIZE-1,argLength);
         strncpy((char *)hCmd->cmdName,
                  (const char *)pCmd,
                  (unsigned int)length);
         hCmd->cmdName[length] = '\0'; /* add the null terminator */
         break;

      default:
         /* Numerical fields */
         if (hCmd->nbData < MAX_NBDATA)
         {
            hCmd->data[(hCmd->nbData)++] = strtoul((const char *)pCmd,0,0);
         }
         break;
   }
}

/**
  * @}
  */

/* End Of File */
