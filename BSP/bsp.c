#include "bsp.h"
#include "stdio.h"
#include "I2CRoutines.h"
#include "soe.h"

__IO  uint32_t timingdelay;
__IO  uint32_t LocalTime = 0;

//#if I2C_METHOD == SINGLEBYTE
//extern unsigned char TxBuffer[8];
//#endif
//#if MEA_UPDATE_METHOD == MEMBLKCP
//extern unsigned char TxBuffer[127];
//#endif

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
//#define FRONTDITRIP	//前面个别点做非电量开入 
//#define REARDITRIP		//后面个别点做非电量开入
#define SPECIALDITRIP

#define NonElectro_Nbr  4             //the total number of Non-Electrical Trip DI
#define TRIPDIMAP       12, 13, 14, 15   //非电量 DI序号
#define TRIPDOMAP       2, 2, 2, 2       //非电量跳闸 DO序号
#define ON 1
#define OFF 0
#define DiRetThreshold 10
#define DiActThreshold 16

#define TX_SEND_TIMEOUT    ((uint32_t)0x01FFFF) //UART send timeout
#define TX_SIGN_TIMEOUT    ((uint32_t)0x01FFFF) //UART send sign timeout

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC2_DR_Address    ((uint32_t)0x4001284C)
#define ADC3_DR_Address    ((uint32_t)0x40013C4C)

#define UA_CHN  ADC_Channel_4
#define UB_CHN	ADC_Channel_5
#define UC_CHN	ADC_Channel_7
#define U0_CHN	ADC_Channel_6
#define IA_CHN	ADC_Channel_11
#define IB_CHN	ADC_Channel_12
#define IC_CHN	ADC_Channel_13



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC1_InitStructure;
ADC_InitTypeDef ADC2_InitStructure;
ADC_InitTypeDef ADC3_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
DMA_InitTypeDef  I2CDMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
I2C_InitTypeDef I2C_InitStruct;

//DI 数据结构
DiStatus_Type DiStatus_DI[DI_NUM];

//DO 跳闸矩阵
__IO uint8_t Trip_turnned[NonElectro_Nbr];
__IO uint8_t turnned[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t NonElectr_TripMap[NonElectro_Nbr]={TRIPDOMAP};   //the element write as the number of Trip
uint8_t NonElectr_TripDi[NonElectro_Nbr]={TRIPDIMAP};
//DO 跳闸状态
DoStructure DoStruct[DO_NUM];
uint8_t __IO DoStatusMap;

//UART 收/发缓冲区
uint8_t TxBuffer[TX_BUFSIZE];
uint8_t RxBuffer[RX_BUFSIZE];

//UART 收/发送完成标识
uint16_t Flag_Uart_Send = 0;
uint16_t Flag_Uart_Recv = 0;

//UART ADDR
uint8_t UartAddr = 0;

//uart 接收定时器 间隔时间参数
__IO uint16_t CCR1_Val = 7000;


//以下定义来自"BaseBus_Protocol.c"
extern uint16_t DiTab;   //遥信值列表


/* Private function prototypes -----------------------------------------------*/


//void DMA_Configuration(void) 
//{  /* DMA1 channel1 configuration ----------------------------------------------*/
//  
//  DMA_DeInit(DMA1_Channel1);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC12ConvertedValueTab;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//  DMA_InitStructure.DMA_BufferSize = 16;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(DMA1_Channel1, &DMA_InitStructure);  
//  /* Enable DMA1 channel1 */
//  DMA_Cmd(DMA1_Channel1, ENABLE);
//
////  DMA_DeInit(DMA1_Channel2);
////  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC2_DR_Address;
////  DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t)ADC12ConvertedValueTab;
////  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
////  DMA_InitStructure.DMA_BufferSize = 16;
////  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
////  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
////  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
////  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
////  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
////  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
////  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
////  DMA_Init(DMA1_Channel2, &DMA_InitStructure);  
////  /* Enable DMA1 channel1 */
////  DMA_Cmd(DMA1_Channel2, ENABLE);
//
//  /* DMA2 channel3 configuration ----------------------------------------------*/
//  DMA_DeInit(DMA1_Channel3);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC3_DR_Address;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC3ConvertedValueTab;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//  DMA_InitStructure.DMA_BufferSize = 16;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
// // DMA_Init(DMA1_Channel3, &DMA_InitStructure);  
//  /* Enable DMA2 channel5 */
//  //DMA_Cmd(DMA1_Channel3, ENABLE);
//
//  DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
// // DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
// // DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);
//}

void ADC_Configuration(void)
{  /* ADC1 configuration ------------------------------------------------------*/
  
  ADC1_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC1_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC1_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC1_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC1_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC1_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC1_InitStructure);
  /* ADC1 regular channels configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC1, 3);
  ADC_InjectedChannelConfig(ADC1, IA_CHN, 1, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC1, IB_CHN, 2, ADC_SampleTime_1Cycles5); 
  ADC_InjectedChannelConfig(ADC1, IC_CHN, 3, ADC_SampleTime_1Cycles5); 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);  
  /* Enable ADC1 DMA */
  // ADC_DMACmd(ADC1, ENABLE);
  
  /* ADC2 configuration ------------------------------------------------------*/
  
  ADC2_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC2_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC2_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC2_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC2_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC2_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC2, &ADC2_InitStructure);
  /* ADC2 regular channels configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC2, 4);
  ADC_InjectedChannelConfig(ADC2, UA_CHN, 1, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC2, UB_CHN, 2, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC2, UC_CHN, 3, ADC_SampleTime_1Cycles5);
  ADC_InjectedChannelConfig(ADC2, U0_CHN, 4, ADC_SampleTime_1Cycles5);
  
  ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);
  /* Enable ADC2 EOC interupt */
  //  ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
  
  /* ADC3 configuration ------------------------------------------------------*/
  
  /* Enable ADC3 DMA */
  //ADC_DMACmd(ADC3, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
  
  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
  
  /* Enable ADC2 reset calibaration register */   
  ADC_ResetCalibration(ADC2);
  /* Check the end of ADC2 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC2));
  
  /* Start ADC2 calibaration */
  ADC_StartCalibration(ADC2);
  /* Check the end of ADC2 calibration */
  while(ADC_GetCalibrationStatus(ADC2));
  
}
/**
* @brief  Configures the different system clocks.
* @param  None
* @retval None
*/
void RCC_Configuration(void)
{
  /*
  HCLK(AHB)=56M
  PCLK2(高APB)=56M
  PCLK1(低APB)=28M
  */
  /* ADCCLK = PCLK2/4 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div4);   //14M
  
  /* Enable peripheral clocks ------------------------------------------------*/
  /* Enable DMA1 and DMA2 clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 |RCC_AHBPeriph_CRC, ENABLE);
  
  /* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM5
                         | RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM3, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2
                         | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
                           | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD 
                             | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO , ENABLE);
}

/**
* @brief  Configures the different GPIO ports.
* @param  None
* @retval None
*/
void GPIO_Configuration(void)
{
  //-----------UART CONTRLO
  //------ADDR
  GPIO_InitStructure.GPIO_Pin = ADDR_IO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ADDR_IO_GPIO, &GPIO_InitStructure);
  //------SEND CONTROL
  GPIO_InitStructure.GPIO_Pin = CONTROL_IO_PIN1 | CONTROL_IO_PIN2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(CONTROL_IO_GPIO, &GPIO_InitStructure);
  
  //-----------UART3
  //Configure USART3 Tx 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //Configure USART3 Rx 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //-----------TIM3
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);  
  
  //chanl1
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //-----------ADC
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //-----------DI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  //-----------DO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  
  //继电器出口, 低电平驱动继电器闭合; 初始化为输出高电平
  GPIO_SetBits(GPIOE, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13); 
}


/**
* @brief  Configures Vector Table base location.
* @param  None
* @retval None
*/
void NVIC_Configuration(void)
{
  /* Set the Vector Table base address at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);  
  
  /* Configure and enable ADC interrupt */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  //adc
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	 
  
  //freq
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //DI
  NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  

  //-------uart3
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}


void I2C_Configuration(void)
{
  uint16_t timeout;
  GPIO_InitTypeDef GPIO_InitStructure; 
  DMA_InitTypeDef  I2CDMA_InitStructure;
  
  /* Enable I2C2 reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* Release I2C2 from reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  
  
  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;   //复用开漏输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_ClockSpeed =100000;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  
  
  I2C_InitStruct.I2C_OwnAddress1 = 0xC0;
  I2C_Init(I2C1,&I2C_InitStruct);
  timeout=0x1ff;
  while(timeout--);
  
  I2C_DeInit(I2C1);
  I2C_Init(I2C1,&I2C_InitStruct);
  I2C_Cmd(I2C1,ENABLE);
  I2C_ITConfig(I2C1, I2C_IT_EVT|I2C_IT_ERR, ENABLE);
#if I2C_METHOD == DMA  
  {
    I2C1->CR2 |= CR2_DMAEN_Set;
  }
#endif
#if I2C_METHOD == INTERRUPT  
  {
    I2C1->CR2 |= I2C_IT_BUF;
  }
#endif

#if I2C_METHOD ==DMA		
  DMA_DeInit(DMA1_Channel6);
  I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
  I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer;   
  I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    
  I2CDMA_InitStructure.DMA_BufferSize = /*98*/102;    //tyh:20130730           
  I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
  I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &I2CDMA_InitStructure);
  
  DMA_DeInit(DMA1_Channel7);
  DMA_Init(DMA1_Channel7, &I2CDMA_InitStructure);
  DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
#endif
}

void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructure.TIM_Prescaler = (10-1);//64个采样点=9 128点=4
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = (1750-1);	   //56M除1750 32K	   当HCLK = 56MHZ, 而APB1 prescaler != 1 所以timer2时钟56M
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM2,ENABLE);
  TIM_ClearITPendingBit(TIM2,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2,ENABLE);
  
}
#ifdef WATCHDOG
void IWDG_Configuration(void)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: 32KHz(LSI) / 32K/256 = 125Hz */
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  
  /* Set counter reload value to 349 */
  IWDG_SetReload(499);
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
  
}

void WDGFeeding(void)
{
  IWDG_ReloadCounter();
} 
#endif
void Time_Update(void)
{
  LocalTime++;
}

void SysTick_Configuration()

{
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
}

void Delay(volatile uint32_t nCount)
{ 
  timingdelay = LocalTime + nCount;  
  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime);
}

void FREQ_MEA_Init ( void )
{
  /* TIM2 Input Capture Channel 4 mode Configuration */
  
  TIM_ICInitTypeDef TIM_ICInitStructure;
  //TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  //  NVIC_InitTypeDef NVIC_InitStructure; 
  
  //DBGMCU_Config(DBGMCU_TIM3_STOP,ENABLE); 
  //fDTS = fCK_INT 
  //CK_INT 1M
  //TIM_TimeBaseStructure.TIM_Period =;     
  //TIM_TimeBaseStructure.TIM_Prescaler = 55;      
  //TIM_TimeBaseStructure.TIM_ClockDivision = 0;   
  //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_PrescalerConfig(TIM5, 559, TIM_PSCReloadMode_Immediate);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0f;
  TIM_ICInit(TIM5, &TIM_ICInitStructure); 
  
  //PA3
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU; //GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  TIM_ClearITPendingBit(TIM5,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
  TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);
  
  
  /* TIM enable counter */
  TIM_Cmd(TIM5, ENABLE);
}

void I2CHW_Reset(void)
{
  __IO uint32_t Timeout;
  //uint8_t i;
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
  I2C_DeInit(I2C1);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
 
  //  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_8;
  //  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //  
  //  GPIO_SetBits(GPIOB,GPIO_Pin_9);
  //  GPIO_SetBits(GPIOB,GPIO_Pin_8);
  //  Timeout=0x1ff;
  //  while(Timeout--);
  
  I2C_Configuration();
  Timeout=0x1ff;
  while(Timeout--);  
}    


//////////////////////////////////////////
//tyh:用于和 AI采样芯片通信
void COM3_Configuration(uint32_t BaudRate)
{
  USART_InitTypeDef USART_InitStructure;
  
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART3, &USART_InitStructure);
  
  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);   //设置发送完成中断
  
  //USART_ITConfig(USART3, /*USART_IT_ORE*/USART_IT_RXNE, ENABLE);     //设置ORE中断   0x0360 
  
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);    //启用DMA传输
  USART_DMACmd(USART3, USART_DMAReq_Rx , ENABLE);   
  
  USART_Cmd(USART3, ENABLE);
  //如下语句解决第1个字节无法正确发送出去的问题
  USART_GetFlagStatus(USART3, USART_FLAG_TC);     // 清标志  
}

//////////////////////////////////////////
void DMA_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  
  //-----------------uart3
  DMA_DeInit(DMA1_Channel2);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = TX_BUFSIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  DMA_DeInit(DMA1_Channel3);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART3->DR));
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = RX_BUFSIZE;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);  
  
  /* Enable DMA1_Channel Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
}


/////////////////////////////////////
void TIM_UART_Config()
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535/*10000*/;  //这个就是自动装载的计数值，由于计数是从0开始的，计数10000次后为9999
  TIM_TimeBaseStructure.TIM_Prescaler = 719; // 这个就是预分频系数，当由于为0时表示不分频所以要减1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;  //定义在定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx) 使用的采样频率之间的分频比例
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);   //初始化定时器2
  
  /* TIM3 通道2 输出模式 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);    //配置 通道2
  
  /* TIM3 通道1 捕获比较模式 */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);     //配置 通道1   
  
  /* TIM3 Input trigger configuration: External Trigger connected to TI1 */
  TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
  
  /* TIM3 configuration in slave reset mode  where the timer counter is 
  re-initialied in response to rising edges on an input capture (TI1) */
  TIM_SelectSlaveMode(TIM3,  TIM_SlaveMode_Reset);  
  
  
  /* Clear TIM3 update pending flag[清除TIM3溢出中断标志] */
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  
  /* TIM IT enable */ //打开中断
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);  
}


//////////////////////////////////////////////
void GPIO_DealInputData(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint8_t pos)
{
  if(Bit_SET!=GPIO_ReadInputDataBit(GPIOx,GPIO_Pin))	//开入无输入
  {
    if(DiStatus_DI[pos].Retcnt>3) 
      DiStatus_DI[pos].Actcnt=0;	
    
    if(DiStatus_DI[pos].Retcnt<DiRetThreshold) 
      DiStatus_DI[pos].Retcnt++;
    else
    {
      if(DiStatus_DI[pos].Value !=1)
      {
        if(DiStatus_DI[pos].Value != 0)
          soe_engine(pos, 1);
        
        DiStatus_DI[pos].Value=1;
        DiTab &= ~(1<<pos);
      }
    }
  }
  else  //开入有输入
  {
    if(DiStatus_DI[pos].Actcnt>3) 
      DiStatus_DI[pos].Retcnt=0;
    
    if(DiStatus_DI[pos].Actcnt<DiActThreshold) 
      DiStatus_DI[pos].Actcnt++;
    else
    {
      if(DiStatus_DI[pos].Value !=2)
      {
        if(DiStatus_DI[pos].Value != 0)
          soe_engine(pos, 2);
        
        DiStatus_DI[pos].Value=2;
        DiTab |= (1<<pos);
      }
    }
  }
}


////////////////////////////////
void DiScanning(void)
{
  GPIO_DealInputData(DI0,0);
  GPIO_DealInputData(DI1,1);
  GPIO_DealInputData(DI2,2);
  GPIO_DealInputData(DI3,3);
  GPIO_DealInputData(DI4,4);
  GPIO_DealInputData(DI5,5);
  GPIO_DealInputData(DI6,6);
  GPIO_DealInputData(DI7,7);
  GPIO_DealInputData(DI8,8);
  GPIO_DealInputData(DI9,9);
  GPIO_DealInputData(DI10,10);
  GPIO_DealInputData(DI11,11);
  GPIO_DealInputData(DI12,12);
  GPIO_DealInputData(DI13,13);
  GPIO_DealInputData(DI14,14);
  GPIO_DealInputData(DI15,15);
}


////////////////////////////////////
void Di_PostWork(void)
{
  uint8_t i;
  
#ifdef SPECIALDITRIP
  for(i=0; i<NonElectro_Nbr; i++) //非电量跳闸
  {		 
    if(2 == DiStatus_DI[NonElectr_TripDi[i]-1].Value)
    {
      if(Trip_turnned[i]==0)
      {      
        if( DoExecute(NonElectr_TripMap[i]))
        {          
          //printlog("input%d occur",i-7);
          turnned[i]=1;
        }        
      }		  
    }
    else
    {
      Trip_turnned[i]=0;
    }
  }
/*  
  for(i=0;i<16;i++)
  {
    if(ON==DiStatus_DI[i].Value)
    {
      if(turnned[i]==OFF)
      {
        //soe_engine(i,ON);
        turnned[i]=ON;
      }		  
    }
    else if(OFF==DiStatus_DI[i].Value)
    {
      if(turnned[i]==ON)
      {
        //soe_engine(i,OFF);
        turnned[i]=OFF;
      }	
    }
  }
*/
#endif
  
//#ifdef FRONTDITRIP
//  for(i=0;i<NonElectro_Nbr;i++)
//  {
//    if(1==DiStatus_DI[i].Value)
//    {
//      if(turnned[i]==0)
//      {
//        if( DoExecute(NonElectr_TripMap[i]))
//        {          
//          //printlog("input%d occur",i-7);
//          turnned[i]=1;
//        }
//      }		  
//    }
//    else
//    {
//      turnned[i]=0;
//    }
//  }
//  
//  for(i=NonElectro_Nbr;i<16;i++)
//  {
//    if(ON==DiStatus_DI[i].Value)
//    {
//      if(turnned[i]==OFF)
//      {
//        //soe_engine(i,ON);
//        turnned[i]=ON;
//      }		  
//    }
//    else if(OFF==DiStatus_DI[i].Value)
//    {
//      if(turnned[i]==ON)
//      {
//        //soe_engine(i,OFF);
//        turnned[i]=OFF;
//      }	
//    }
//  }
//#endif
//  
//#ifdef REARDITRIP
//  for(i=16-NonElectro_Nbr;i<16;i++)
//  {
//    if(1==DiStatus_DI[i].Value)
//    {
//      if(turnned[i]==0)
//      {
//        if( DoExecute(NonElectr_TripMap[i]) )
//        {
//          //printlog("input%d occur",i-7);
//          turnned[i]=1;
//        }
//      }		  
//    }
//    else
//    {
//      turnned[i]=0;
//    }
//  }
//  for(i=0;i<16-NonElectro_Nbr;i++)
//  {
//    if(ON==DiStatus_DI[i].Value)
//    {
//      if(turnned[i]==OFF)
//      {
//        //soe_engine(i,ON);
//        turnned[i]=ON;
//      }		  
//    }
//    else if(OFF==DiStatus_DI[i].Value)
//    {
//      if(turnned[i]==ON)
//      {
//        //soe_engine(i,OFF);
//        turnned[i]=OFF;
//      }	
//    }
//  }
//#endif
}

//////////////////////tyh:20150629 出口函数需重写
bool DoExecute(unsigned char DoSeq)
{
  if(DoSeq >= DO_NUM)
  {
    return false;
  }
  
  DoStruct[DoSeq-1].status =1;  
  switch(DoSeq)
  {
  case 1:
    DoStatusMap|=0x01;
    RelayClose(DO0);
    break;
  case 2:	
    DoStatusMap|=0x02;
    RelayClose(DO1);
    break;
  case 3:	
    DoStatusMap|=0x04;
    RelayClose(DO2);
    break;
  case 4:
    DoStatusMap|=0x08;
    RelayClose(DO3);
    break;
/*    
  case 5:	
    DoStatusMap|=0x10;
    RelayClose(DO4);
    break;
  case 6:
    DoStatusMap|=0x20;
    RelayClose(DO5);
    break;
  case 7:
    DoStatusMap|=0x40;
    RelayClose(DO6);
    break;
  case 8:
    DoStatusMap|=0x80;
    RelayClose(DO7);
    break;
*/    
  default:
    return false;
  }	  
  
  return true;
}

/////////////////////
void DO_Process(void)
{
  uint8_t i;
  if(DoStatusMap)
  {
    for(i=0; i<DO_NUM; i++)
    {
      if(DoStruct[i].status)
      {
        DoStruct[i].DoTimer++;
        
        if((DoStruct[i].DoTimer)>= DO_TMR_INTERVAL)
        {
          DoStruct[i].status=0x00;
          DoStruct[i].DoTimer=0;
          
          switch(i)
          {
          case 0:
            DoStatusMap &= 0xfe;
            RelayOpen(DO0);
            break;
            
          case 1:
            DoStatusMap &= 0xfd;
            RelayOpen(DO1);
            break;
            
          case 2:
            DoStatusMap &= 0xfb;
            RelayOpen(DO2);
            break;
            
          case 3:
            DoStatusMap &= 0xf7;
            RelayOpen(DO3);
            break;
/*            
          case 4:
            DoStatusMap &= 0xef;
            RelayOpen(DO4);
            break;
            
          case 5:
            DoStatusMap &= 0xdf;
            RelayOpen(DO5);
            break;
            
          case 6:
            DoStatusMap &= 0xbf;
            RelayOpen(DO6);
            break;
            
          case 7:
            DoStatusMap &= 0x7f;
            RelayOpen(DO7);
            break;
*/            
          default:
            break;
          }	
        } 
      }
    }
  }
}

//////////////////////
void RelayClose(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{ 
  GPIO_ResetBits(GPIOx,GPIO_Pin);
}


//////////////////////
void RelayOpen(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{ 
  GPIO_SetBits(GPIOx,GPIO_Pin);
}

//////////////////////
bool SendUartData(uint16_t lenght)
{
  __IO uint32_t timeout = 0;
  
  if(lenght > TX_BUFSIZE)
    return false;
  
  while((!GetSendSign()) & (timeout < TX_SIGN_TIMEOUT))
  {
    timeout++;
  }
  if(timeout == TX_SIGN_TIMEOUT)
    return false;
  
  Flag_Uart_Send = 1;
  
  DMA1_Channel2->CNDTR = lenght;
  DMA_Cmd(DMA1_Channel2, ENABLE); //使能发送通道

  while(Flag_Uart_Send & (timeout < TX_SEND_TIMEOUT))
  {
    timeout++;
  }
  if(timeout == TX_SEND_TIMEOUT)
    return false;

  return true;
}

void RTC_Init()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  
  //采用HSE/128作为时钟频率
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128);

  RCC_RTCCLKCmd(ENABLE);
  
  /* Wait for RTC registers synchronization */
  RTC_WaitForSynchro();
  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
  
  /* RTC period = RTCCLK/RTC_PR = (8MHz/128)/(62499+1) = 1  (HSE)*/
  RTC_SetPrescaler(62499);
  
  /* 等待上一次对RTC寄存器的写操作完成 */
  RTC_WaitForLastTask();
}


uint8_t GetComAddr()
{
  if(Bit_SET == GPIO_ReadInputDataBit(ADDR_IO_GPIO, ADDR_IO_PIN))
    UartAddr = 1;
  
  return UartAddr;
}

bool GetSendSign()
{
  uint8_t sign;
  
  sign = GPIO_ReadInputDataBit(CONTROL_IO_GPIO, CONTROL_IO_PIN1);
  //sign |= (GPIO_ReadInputDataBit(CONTROL_IO_GPIO, CONTROL_IO_PIN2)<<1);
  
  if(sign == UartAddr)
    return true;
  else
    return false;
}