/**
********************************************************************************
* @file    PWM_handlers.c
* @author  BLE Mesh Team
* @version V1.0.0
* @date    31-July-2018
* @brief   Handlers for PWM and other support functions.
********************************************************************************
* @attention
*
*<h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Initial BlueNRG-Mesh is built over Motorola�s Mesh over Bluetooth Low Energy 
* (MoBLE) technology. The present solution is developed and maintained for both 
* Mesh library and Applications solely by STMicroelectronics.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "PWM_handlers.h"
#include "mesh_cfg_usr.h"

/* Global Variables */
uint16_t DUTY;
int NonZeroPWM0=0;
int NonZeroPWM1=0;
uint8_t Duty_flag_A = 0;
uint16_t PWM0_on, 
	 PWM1_on, 
	 PWM2_on = MFT1_TON_2,  /* Timer2, MFT1 */
	 PWM3_on = MFT2_TON_2,  /* Timer2, MFT2 */
	 PWM4_on = MFT2_TON_3;  /* Timer2, MFT2 */

uint32_t channel = PWM3_PIN;    /* It is the channel with initial lowest duty-cycle */
uint32_t period = TIME_PERIOD;  /* Period in ticks of the PWM signal */
uint8_t channel_number = 0;

uint32_t PWM_channels[] = {PWM3_PIN, PWM4_PIN};  /* Channels managed by MFT2 Timer 2 */                                                    
uint16_t Ton_values[] = {MFT2_TON_2, MFT2_TON_3}; /* Sorted array containing duty-cycle values
                                                    respectively for PWM3_PIN and PWM4_PIN */
uint16_t delta_1 = MFT2_TON_3 - MFT2_TON_2; /* Delta between the first two duty-cycles */
uint16_t delta = MFT2_TON_3 - MFT2_TON_2;

int number_of_channels = 2;  /* Number of channels managed by MFT2 Timer 2 */
int max_duty = MFT2_TON_3;  /* Maximum duty-cycle */

/**
  *@brief  PWM2 handler
  *@retval None
  */
void PWM2_handler() 
{
  if ( (MFT1->TNICTRL & MFT_IT_TND) != (uint32_t)RESET )
  { 
    SET_BIT(MFT1->TNICLR, MFT_IT_TND);
    if(Duty_flag_A == 1)
      {
        if(PWM2_on >1)
        {
          WRITE_REG(GPIO->DATS, PWM2_PIN);
          Duty_flag_A = 0;
          MFT1->TNCNT2 = PWM2_on;
          return;
        }
      }
    else               
      {
	WRITE_REG(GPIO->DATC, PWM2_PIN);
	Duty_flag_A = 1;
	MFT1->TNCNT2 = TIME_PERIOD - PWM2_on;
	return;
      }
  }
}

/**
  *@brief  PWM3 and PWM4 handler
  *@retval None
  */
void PWM3_PWM4_handler() {
  if ( (MFT2->TNICTRL & MFT_IT_TND) != (uint32_t)RESET)
  {
    SET_BIT(MFT2->TNICLR, MFT_IT_TND);
    /*STATE 0
    if delta != 0, there is a channel with a duty lower than the other one.
    So the channel with the lower duty is cleared. Delta is period - max_duty
    (the duty of the other channel)
    The machine passes to STATE 1.

    if delta == 0, the two channel are cleared together.
    The machine passes to STATE 2

    The delta is computed for the next timer event*/

    if (channel_number < number_of_channels - 1)
    { 
      if (delta != 0)
      {
        WRITE_REG(GPIO->DATC, channel);
      }
      else
      {
        WRITE_REG(GPIO->DATC, PWM3_PIN | PWM4_PIN);
        delta = period - max_duty;
        MFT2->TNCNT2 = delta - 1;        
        channel_number = number_of_channels;
        return;
      }
      MFT2->TNCNT2 = delta - 1;
      channel_number ++;
      delta = period - max_duty; /*computed delta for the next reset of the counter*/
      channel = PWM_channels[channel_number];
      return;
    }
  
    /*STATE 1
    Here the channel with the max duty-cycle is cleared. The delta is that one 
    computed in STATE 0. The machine passes to STATE 2 */
   if (channel_number == number_of_channels - 1)
   {
      WRITE_REG(GPIO->DATC, channel);
      MFT2->TNCNT2 = delta - 1;
      channel_number ++;
      return;
   }     
    
   /*STATE 2
   All the channels have been cleared and now are set again. The machine passes to STATE 0 */
    if (channel_number > number_of_channels - 1)
    {
      if(max_duty > 1)
      {
        WRITE_REG(GPIO->DATS, PWM3_PIN | PWM4_PIN);
      }
      
      MFT2->TNCNT2 = Ton_values[0] - 1;
      Ton_sorting();
      channel_number = 0;
      delta = Ton_values[1] - Ton_values[0];
      channel = PWM_channels[0];
      return;
    } 
  }
}

/**
  *@brief  Sorting on basis of duty cycle
  *@retval None
  */

void Ton_sorting(void)
{
  if (PWM3_on > PWM4_on)
  {
    max_duty = PWM3_on; 
    Ton_values[1] = PWM3_on;
    Ton_values[0] = PWM4_on;
    PWM_channels[1] = PWM3_PIN;
    PWM_channels[0] = PWM4_PIN;
  }
  else
  {
    max_duty = PWM4_on;
    Ton_values[0] = PWM3_on;
    Ton_values[1] = PWM4_on;
    PWM_channels[0] = PWM3_PIN;
    PWM_channels[1] = PWM4_PIN;  
  }
}


/**
  *@brief  PWM modification
  *@param  PWM_ID: PWM number
  *@param  duty_cycle: Duty cycle at output 
  *@retval None
  */
void Modify_PWM(uint8_t PWM_ID, uint16_t duty_cycle) 
{  
   GPIO_InitType GPIO_InitStructure1;
   
   if (PWM_ID == 0)     /* GPIO4 */
   {
       if (duty_cycle == 0 && NonZeroPWM0 == 0)   
       {
           SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
           GPIO_InitStructure1.GPIO_Pin = PWM0_PIN; 
           GPIO_InitStructure1.GPIO_Mode = GPIO_Output; 
           GPIO_InitStructure1.GPIO_Pull = DISABLE;
           GPIO_InitStructure1.GPIO_HighPwr = DISABLE;
           GPIO_Init( &GPIO_InitStructure1);
           GPIO_WriteBit(PWM0_PIN, Bit_RESET);   
          
           NonZeroPWM0=1;
       }
       else if ((duty_cycle != 0 && NonZeroPWM0 == 1))
       {
           SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
           GPIO_InitStructure1.GPIO_Pin = PWM0_PIN ;                                   
           GPIO_InitStructure1.GPIO_Mode =  Serial2_Mode;                                
           GPIO_InitStructure1.GPIO_Pull = DISABLE;
           GPIO_InitStructure1.GPIO_HighPwr = DISABLE;
           GPIO_Init( &GPIO_InitStructure1);
           NonZeroPWM0=0;
         
       }
   }
   else if((PWM_ID == 1))   /* GPIO3 */
   {
      if (duty_cycle == 0 && NonZeroPWM1 == 0)
       {
           SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
           GPIO_InitStructure1.GPIO_Pin =  PWM1_PIN; 
           GPIO_InitStructure1.GPIO_Mode = GPIO_Output; 
           GPIO_InitStructure1.GPIO_Pull = DISABLE;
           GPIO_InitStructure1.GPIO_HighPwr = DISABLE;
           GPIO_Init( &GPIO_InitStructure1);
           GPIO_WriteBit(PWM1_PIN, Bit_RESET);            
           NonZeroPWM1=1;
       }
       else if ((duty_cycle != 0 && NonZeroPWM1 == 1))
       {
           SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
           GPIO_InitStructure1.GPIO_Pin = PWM1_PIN;
           
#ifdef CUSTOM_BOARD_PWM_SELECTION
           
           GPIO_InitStructure1.GPIO_Mode =  Serial2_Mode; 
#else
           GPIO_InitStructure1.GPIO_Mode =  Serial1_Mode; 
#endif                                
           GPIO_InitStructure1.GPIO_Pull = DISABLE;
           GPIO_InitStructure1.GPIO_HighPwr = DISABLE;
           GPIO_Init( &GPIO_InitStructure1);
           NonZeroPWM1=0;
          
       }
   }
   else
   {
   }
   
   DUTY = duty_cycle;
   switch (PWM_ID) 
    {
    case 0: /* PWM0 */ 
      {                  
        MFT1->TNCRA = DUTY;
        MFT1->TNCRB = TIME_PERIOD - duty_cycle;
        
      }
      break;
    case 1: /* PWM1 */
      {
        MFT2->TNCRA = DUTY;
        MFT2->TNCRB = TIME_PERIOD - duty_cycle;
      }
      break;
    case 2: /* PWM2 */
      {
        PWM2_on = DUTY;
      }
      break;
    case 3: /* PWM3 */
      {
        PWM3_on = DUTY;
      }
      break;
    case 4: /* PWM4 */
      {
        PWM4_on = DUTY;
        
      }
      break;
    }
}
