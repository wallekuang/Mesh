/**
*   @file      handler_eeprom.c
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
#include "handler_eeprom.h"
 #include "st_device.h"
 
extern uint32_t Cumm_ActiveFEnergy;
extern uint32_t Cumm_ApparentEnergy;
extern uint32_t Cumm_ReactiveEnergy;
uint8_t i=0;

/** @addtogroup GENERIC
  * @{
  */

/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/
//#define EEPROM_CMD_RD_MEMORY_ARAY       0xA1
//#define EEPROM_CMD_WR_MEMORY_ARAY       0xA0
 
/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/
//I2C_HandleTypeDef hi2c;
 

 
 
/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/
static void I2Cx_Error(uint8_t Addr);

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/

///**
//  * @brief  This function implements EEPROM configuration
//  * @param  None
//  * @retval None
//  */
//void EEPROM_Conf(void)
//{
//  hi2c.Instance = I2C_EEPROM;
//  hi2c.Init.ClockSpeed = 100000;
//  hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c.Init.OwnAddress1 = 0;
//  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
//  hi2c.Init.OwnAddress2 = 0;
//  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
//  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
//  HAL_I2C_Init(&hi2c);
//}

/**
  * @brief  This function implements EEPROM Read
  * @param  buf is a pointer to destination buffer
  * @param  address is the source address in EEPROM
  * @param  size is the nulber of bytes to read
  * @retval None
  */
void EEPROM_Read(uint8_t *buf, uint32_t address, uint16_t size)
{

  int status =1;
 status= SdkEvalI2CRead(buf,EEPROM_CMD_RD_MEMORY_ARAY, (uint16_t)address,size);

 
  
  /* Check the communication status */
  if(status != 0)
  {
    
  }

  
}

 
/**
  * @brief  This function implements EEPROM write
  * @param  address is the destination address in EEPROM
  * @param  buf is a pointer to source buffer
  * @param  size is the nulber of bytes to write
  * @retval None
  */
void EEPROM_Write(uint32_t address, uint8_t *buf, uint16_t size)
{

    int status =1;
 status= SdkEvalI2CWrite(buf,EEPROM_CMD_RD_MEMORY_ARAY, (uint16_t)address,size);

   
  /* Check the communication status */
  if(status !=0)
  {
    /* Execute user timeout callback */
    
  }

}


/*******************************************************************************
*
*                       IMPLEMENTATION: Private functions
*
*******************************************************************************/
static void I2Cx_Error(uint8_t Addr)
{
  /* De-initialize the I2C comunication bus */
  //HAL_I2C_DeInit(&hi2c2);

  /* Re-Initiaize the I2C comunication bus */
  //EEPROM_Conf();
}

/**
  * @}
  */

/* End Of File */
