//傅立葉轉換
Fourier24()

OverSampling(uint16_t *pSampleBase,uint8_t nsize)
{
/*STM32采样过采样分辨率对应关系
	---------------------------------
	分辨率  | 采样次数  | 每秒采样次数 
	12ADC   |    1      |      1M 
	13ADC   |    4      |      250K 
	14ADC   |    16     |      62.5K 
	15ADC   |    64     |      15.6K 
	16ADC   |    256    |      3.9K
	---------------------------------
		
	每多一位分辨率多4倍次采样，每次中断采样过采样16次提高分辨率至14位
*/
uint8_t sampseq= 0;
uint32_t result = 0;
  
  for( sampseq = 0; sampseq < nszie ; sampseq++)
    result += pSampleBase[index];
  
 return  (result>>2) ;


}

ac_model_init()
{
	/* TIM1 configuration ------------------------------------------------------*/ 
  TIM1_DeInit();
  /* Time Base configuration */
  TIM1_TimeBaseStructure.TIM1_Prescaler = 0x4;
  TIM1_TimeBaseStructure.TIM1_CounterMode = TIM1_CounterMode_Up;
  TIM1_TimeBaseStructure.TIM1_Period = 0xFF;
  TIM1_TimeBaseStructure.TIM1_ClockDivision = 0x0;
  TIM1_TimeBaseStructure.TIM1_RepetitionCounter = 0x0;
  TIM1_TimeBaseInit(&TIM1_TimeBaseStructure);
  /* Channel1 Configuration in PWM mode */
  TIM1_OCInitStructure.TIM1_OCMode = TIM1_OCMode_PWM1; 
  TIM1_OCInitStructure.TIM1_OutputState = TIM1_OutputState_Enable;                
  TIM1_OCInitStructure.TIM1_Pulse = 0x7F; 
  TIM1_OCInitStructure.TIM1_OCPolarity = TIM1_OCPolarity_Low;         
  TIM1_OC1Init(&TIM1_OCInitStructure);

  /* DMA Channel1 Configuration ----------------------------------------------*/
  DMA_DeInit(DMA_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_RegularConvertedValueTab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 32;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA_Channel1, &DMA_InitStructure);
  
  /* Enable DMA channel1 */
  DMA_Cmd(DMA_Channel1, ENABLE);

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_13Cycles5);

  /* Set injected sequencer length */
  ADC_InjectedSequencerLengthConfig(ADC1, 1);
  /* ADC1 injected channel Configuration */ 
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_71Cycles5);
  /* ADC1 injected external trigger configuration */
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);

  /* Enable automatic injected conversion start after regular one */
  ADC_AutoInjectedConvCmd(ADC1, ENABLE);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 external trigger */ 
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);

  /* Enable JEOC interupt */
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

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

  /* TIM1 counter enable */
  TIM1_Cmd(ENABLE);
  /* TIM1 main Output Enable */
  TIM1_CtrlPWMOutputs(ENABLE);  

  /* Test on channel1 transfer complete flag */
  while(!DMA_GetFlagStatus(DMA_FLAG_TC1));
  /* Clear channel1 transfer complete flag */
  DMA_ClearFlag(DMA_FLAG_TC1); 

  /* TIM1 counter disable */
  TIM1_Cmd(DISABLE);	
}