/**
******************************************************************************
* @file    misc.c 
* @author  AMG - RF Application Team
* @version V1.1.0
* @date    3-April-2018
* @brief   Miscellaneous utilities
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
******************************************************************************
*/ 
/* Includes ------------------------------------------------------------------*/
#include "BlueNRG_x_device.h"
#include "miscutil.h"
#include "mesh_cfg.h"

NO_INIT_SECTION(crash_info_t crash_info_ram, ".__crash_RAM");

/** @addtogroup BlueNRG1_Miscellaneous_Utilities
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLUENRG2_IDCODE      (0x0200A041)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void HAL_GetPartInfo(PartInfoType *partInfo)
{
  volatile uint32_t *ptr = (volatile uint32_t *)(CKGEN_SOC_BASE + 0x28);

  partInfo->die_major    = CKGEN_SOC->DIE_ID_b.VERSION;
  partInfo->die_cut      = CKGEN_SOC->DIE_ID_b.REV;

  partInfo->jtag_id_code = *ptr;
  if (partInfo->jtag_id_code == BLUENRG2_IDCODE) {
    /* BlueNRG-2 id is conventionally 2 */
    partInfo->die_id = DIE_ID_BLUENRG2;
  } else {
    /* BlueNRG-1 id is conventionally 1 */
    partInfo->die_id = DIE_ID_BLUENRG1;
 }

  /* Patch for early samples */
  if (partInfo->die_major == 0)
    partInfo->die_major = 1;

  partInfo->flash_size = (FLASH->SIZE + 1) * 4;
}

/**
 * @brief Get Crash Information utility
 */
void HAL_GetCrashInfo(crash_info_t *crashInfo)
{
  *crashInfo = crash_info_ram;
  /* Reset crash info value */
  crash_info_ram.signature = 0;
}
void HAL_CrashHandler(uint32_t msp, uint32_t signature)
{
  volatile uint32_t * crash_info = (volatile uint32_t *)&crash_info_ram;
  register uint32_t reg_content;
  /* Init to zero the crash_info RAM locations */
  for (reg_content=0; reg_content<NMB_OF_EXCEP_RAM_WORD; reg_content++) {
    crash_info[reg_content] = 0;
  }
  /* Store Crash Signature */
  crash_info_ram.signature = signature;
  /* Store SP register */
  crash_info_ram.SP = msp;
  for (reg_content=2; reg_content<NMB_OF_EXCEP_RAM_WORD; reg_content++) {
    uint32_t *ptr = ((uint32_t *)msp)+(reg_content-2);
    if ((ptr >= ((uint32_t *)  _MEMORY_RAM_BEGIN_)) && 
        (ptr <= ((uint32_t *) _MEMORY_RAM_END_)))
      crash_info[reg_content] = *ptr;
  }
  NVIC_SystemReset();
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
 * @brief Get Device ID, Version and Revision numbers
 */
void BLEPLAT_get_part_info(uint8_t *device_id, uint8_t *major_cut, uint8_t *minor_cut)
{
   PartInfoType partInfo;
   
   /* get partInfo */
   HAL_GetPartInfo(&partInfo);
  
  /* Set device ID */
  *device_id  = partInfo.die_id;
  
  /* Set major cut  */
  *major_cut = partInfo.die_major; 
 
  /* Set minor cut */
  *minor_cut = partInfo.die_cut;
}

/**
 *  @brief Get Crash Handler Information
 */
void Get_CrashHandlerInfo(void)
{
  crash_info_t crashInfo;
  
  HAL_GetCrashInfo(&crashInfo);
 
  if ((crashInfo.signature & 0xFFFF0000) == CRASH_SIGNATURE_BASE) {
    TRACE_M(TF_HANDLER,"Application crash detected\r\n");
    TRACE_M(TF_HANDLER,"Crash Info signature = 0x%08lx\r\n", crashInfo.signature);
    TRACE_M(TF_HANDLER,"Crash Info SP        = 0x%08lx\r\n", crashInfo.SP);
    TRACE_M(TF_HANDLER,"Crash Info R0        = 0x%08lx\r\n", crashInfo.R0);
    TRACE_M(TF_HANDLER,"Crash Info R1        = 0x%08lx\r\n", crashInfo.R1);
    TRACE_M(TF_HANDLER,"Crash Info R2        = 0x%08lx\r\n", crashInfo.R2);
    TRACE_M(TF_HANDLER,"Crash Info R3        = 0x%08lx\r\n", crashInfo.R3);
    TRACE_M(TF_HANDLER,"Crash Info R12       = 0x%08lx\r\n", crashInfo.R12);
    TRACE_M(TF_HANDLER,"Crash Info LR        = 0x%08lx\r\n", crashInfo.LR);
    TRACE_M(TF_HANDLER,"Crash Info PC        = 0x%08lx\r\n", crashInfo.PC);
    TRACE_M(TF_HANDLER,"Crash Info xPSR      = 0x%08lx\r\n", crashInfo.xPSR); 
  }  
}
/** 
 *@
} */ /* End of group BlueNRG1_Miscellaneous_Utilities */
