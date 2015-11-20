/**
******************************************************************************
* @file    ADC/3ADCs_DMA/stm32f10x_it.c 
* @author  MCD Application Team
* @version V3.4.0
* @date    10/15/2010
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and peripherals
*          interrupt service routine.
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
*/ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "I2CRoutines.h"

void TimingDelay_Decrement(void);
void Sampling_IRQHandler(void);
void MeaFreq_IRQHandler(void);


extern uint8_t TxBuffer[];
extern uint8_t RxBuffer[];
extern uint16_t Flag_Uart_Send;
extern uint16_t Flag_Uart_Recv;

/** @addtogroup STM32F10x_StdPeriph_Examples
* @{
*/

/** @addtogroup ADC_3ADCs_DMA
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief  This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSV_Handler exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
  DiScanning();  
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
* @brief  This function handles ADC1 and ADC2 global interrupts requests.
* @param  None
* @retval None
*/
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET) 
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
  
  Sampling_IRQHandler();
}

///////////////////////////
void TIM5_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM5,TIM_IT_CC4)!=RESET) 
    TIM_ClearITPendingBit(TIM5,TIM_IT_CC4 );
  
  MeaFreq_IRQHandler();
}

//---------------------------- uart 3 ---------
//////////////////////////////////
void USART3_IRQHandler(void)
{
  uint32_t temp;

  //--------------UART3
  if(USART_GetITStatus(USART3, USART_IT_ORE) == SET)
  {
    uint32_t temp;
    
    temp = USART3->SR;  
    temp = USART3->DR; //清USART_IT_IDLE标志     
    
    //USART_Cmd(USART2, DISABLE);
    //USART_Cmd(USART2, ENABLE);
    
    DMA_Cmd(DMA1_Channel3, DISABLE);
    DMA1_Channel3->CNDTR = 300;//重装填
    DMA_Cmd(DMA1_Channel3, ENABLE);    
  } 
}

////////////////////////////
void DMA1_Channel2_IRQHandler(void)
{
  __IO uint32_t timeout = 0;
  
  //---------------AI uart3
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_ClearFlag(DMA1_FLAG_TC2);
  
  Flag_Uart_Send = 0; //重置发送标志
  
  TIM_Cmd(TIM3, ENABLE);  // Emable TIM3
}

////////////////////////////
void TIM3_IRQHandler(void)
{
  uint16_t len, sr;
  static uint8_t i = 0;
  
  sr = TIM3->SR & 0x0202;
  if(sr != 0)
  {
    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
    TIM3->SR &= 0xfdfd;
  }  
  
  if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    
    //关闭接收通道
    DMA_Cmd(DMA1_Channel3, DISABLE);
    
    /* Signal the DMA timeout by toggling Pin PC.07 */
    len = 300-DMA_GetCurrDataCounter(DMA1_Channel3);
    if(len > 0)
    {
      i++;
      /* Disable TIM2*/
//      TIM_Cmd(TIM3, DISABLE);
//    TIM_SetCounter(TIM3, 0);
//    TIM_Cmd(TIM3, ENABLE);

      Flag_Uart_Recv = len;   //开启接收完成标志      
    }
    
    //处理完毕打开接收通道
    DMA1_Channel3->CNDTR = 300;//重装填
    DMA_Cmd(DMA1_Channel3, ENABLE);  
  }
  
  TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);    
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/

/**
* @}
*/ 

/**
* @}
*/ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
