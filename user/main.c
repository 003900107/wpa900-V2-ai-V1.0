
/* Includes ------------------------------------------------------------------*/
#include  <string.h>

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "bsp.h"
#include "table_fft.h"
#include "dft.h"
#include "adc_calc.h"
#include "Realtimedb.h"
#include "soe.h"


extern float MeaTab[MEANUM];
uint16_t Value;
extern int16_t PeriodCycleTab[10][SAMP_POINT_NBR];
//extern uint32_t FFT_ResultTab[7][SAMP_POINT_NBR];
//extern uint32_t lBUFMAG[SAMP_POINT_NBR];
extern float fBUFANG[7];
extern Setting SetCurrent;
extern uint32_t PeriodCycle_Index;

MEA_DFT  Ua_Channel;
Mea_Para meas={1,};
__IO uint8_t RunStatturn;

/** @addtogroup STM32F10x_StdPeriph_Examples
* @{
*/

/** @addtogroup ADC_3ADCs_DMA
* @{
*/ 


/* Private functions ---------------------------------------------------------*/

/**
* @brief   Main program
* @param  None
* @retval None
*/
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32f10x_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f10x.c file
  */  
  uint32_t delay;
  uint8_t sendFlag_Ai = 0;
  SOE_Struct soe;
  
  uint32_t start = 0;
  
  
  /* System clocks configuration ---------------------------------------------*/
  SystemInit();
  
  RCC_ClocksTypeDef clock;
  RCC_GetClocksFreq(&clock);
  
  RCC_Configuration();
  
  GPIO_Configuration();
  
  ADC_Configuration();
  
  COM3_Configuration(9600);
  
  DMA_Configuration();
  
  TIM_UART_Config();  //uart3-rx
  
  FREQ_MEA_Init();  //freq采样
  
  TIM2_Configuration(); //adc采样
  
  SysTick_Configuration();  //DI采样
  
  RTC_Init();
  
  NVIC_Configuration();
  
  DataBase_Init();
  //memset(DiStatus_DI, 0, sizeof(DiStatus_Type)*DI_NUM); //初始化DI内存列表
  GetComAddr();
  
  SetCurrent.ChannelCoef[0] =1050;//1#:1029;2#:1028
  SetCurrent.ChannelCoef[1] =1029;//1#:1029;2#:1027
  SetCurrent.ChannelCoef[2] =1050;//1#:1030;2#:1028
  SetCurrent.ChannelCoef[3] =1028;//1#:1033;2#:1028
  SetCurrent.ChannelCoef[4] =1029;//1#:1033;2#:1029
  SetCurrent.ChannelCoef[5] =1028;//1#:1033;2#:1028
  SetCurrent.ChannelCoef[6] =1025;//1#:1029;2#:1028
  SetCurrent.ChannelCoef[7] =1026;//1#:1029;2#:1027
  SetCurrent.ChannelCoef[8] =1027;//1#:1030;2#:1028
  

#ifdef WATCHDOG
  IWDG_Configuration();
#endif
  
  DMA_Cmd(DMA1_Channel3, ENABLE); // Emable TIM3, 使能串口接收通道 
  TIM_Cmd(TIM3, ENABLE);  //UART3接收，使能
  
  while (1)
  {
    Di_PostWork();
    
    if(63==PeriodCycle_Index)
    {
      TOTAL_MEASURE(&meas);
      
      SequenceFilter_2(&meas);
      SequenceFilter_0(&meas);
      
      ValueScaling(MeaTab,&meas);
      
      //tyh:20150629 添加遥测数据上送
      if(Begin_AI_Send)
      {
        if(Is_new_soe())
        {
          get_soe(&soe, 1);
          SoeResponse(soe);
        }
        else
        {
          if((sendFlag_Ai%3)==0)
            AiResponse(0);
          else
          {
            if((sendFlag_Ai%5)==0)    //tyh:20150803 增加遥信数据上送
              DiResponse(0);
          }
          
          sendFlag_Ai++;
        }
      }
    }
    
    if(Flag_Uart_Recv) 
      BusCalling_Process(Flag_Uart_Recv);     //处理数据    
    
#ifdef WATCHDOG
    WDGFeeding();
#endif
  }
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  
  
  /* Infinite loop */
  while (1)
  {  }
}
#endif

/**
* @}
*/ 

/**
* @}
*/ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
