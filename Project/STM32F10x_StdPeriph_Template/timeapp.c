#include "timeapp.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "radio.h"

static unsigned int TimingDelay;


/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  MsTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void DelayMs(unsigned int MsTime)
{ 
  TimingDelay = MsTime;

  while(TimingDelay != 0);
}

void NopDelayUs(short int x)
{
  char i;
  while(x--)
  {
    for(i=0;i<4;i++)
      asm("nop");
  }
}

void NopDelayMs(short int x)
{
  while(x--)
  {
    NopDelayUs(1000);
  }
}

/*******************************************************************************
* Function Name  : TIM2_Configuration
* Description    : 每1秒发生一次更新事件(进入中断服务程序).
* Input          : None
* Return         : None
*******************************************************************************/
void TIM2_Configuration(unsigned int MsTime)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  //重新将Timer设置为缺省值
  TIM_DeInit(TIM2);
  
  //采用内部时钟给TIM2提供时钟源
  TIM_InternalClockConfig(TIM2);
  
  //预分频系数为36000-1，这样计数器时钟为64MHz/32000 = 2kHz
  TIM_TimeBaseStructure.TIM_Prescaler = 32000 - 1;
  
  //设置时钟分割
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  
  //设置计数器模式为向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  //设置计数溢出大小，每计2000个数就产生一个更新事件
  TIM_TimeBaseStructure.TIM_Period = (2*MsTime);
  
  //将配置应用到TIM2中
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  //清除溢出中断标志
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  
  //禁止ARR预装载缓冲器
  TIM_ARRPreloadConfig(TIM2, DISABLE);  //预装载寄存器的内容被立即传送到影子寄存器 
  
  //开启TIM2的中断
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

/*******************************************************************************
* Function Name  : TIM2_Configuration_Us
* Description    : 每1秒发生一次更新事件(进入中断服务程序).
* Input          : None
* Return         : None
*******************************************************************************/
void TIM2_Configuration_Us(unsigned int UsTime)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  //重新将Timer设置为缺省值
  TIM_DeInit(TIM2);
  
  //采用内部时钟给TIM2提供时钟源
  TIM_InternalClockConfig(TIM2);
  
  //预分频系数为36-1，这样计数器时钟为70MHz/35 = 2MHz
  TIM_TimeBaseStructure.TIM_Prescaler = 35 - 1;
  
  //设置时钟分割
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  
  //设置计数器模式为向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  
  //设置计数溢出大小，每计2个数就产生一个更新事件
  TIM_TimeBaseStructure.TIM_Period = (2*UsTime);
  
  //将配置应用到TIM2中
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  //清除溢出中断标志
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  
  //禁止ARR预装载缓冲器
  TIM_ARRPreloadConfig(TIM2, DISABLE);  //预装载寄存器的内容被立即传送到影子寄存器 
  
  //开启TIM2的中断
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  
  TIM_Cmd(TIM2, ENABLE);
}

unsigned int timingBase=0;

void TimingBaseInit(unsigned int UsTime)
{
  timingBase=0;
  TIM2_Configuration_Us(UsTime);  
}


enum SlotState SS_Global=S_NULL;

void TimSlotCycle(void)
{
  timingBase++;
  
  if(timingBase==TIME_SLOT_LEN)
  {    
    if(SS_Global==S_RTD)
    {
      RadioGotoRdySta();
      SS_Global=S_P1T;      
    }
    else
    {
      if(SS_Global!=S_NULL)
      {
        SS_Global++;
        if(SS_Global==S_NULL)
        {
          RadioGotoRxSta();
        } 
      }
      else
      { 
        if(GetRadioSta()==RF_RDY)
        {
          RadioGotoRxSta();
        }     
      }
        
    }
    timingBase=0;    
//    LedD3StaInvert(); 
//    LedD4StaInvert(); 
  }
  

}

void TimSlotCycleRst(void)
{
  SS_Global=S_RTS;
  timingBase=0;
}

void TIM2_IsTimeOn(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);    
//    LedD3StaInvert(); 
//    LedD4StaInvert(); 
  }
}

void TimingSyncReset(void)
{
  timingBase=0;
}

unsigned int GetTimingBase()
{
  return timingBase;
}

void EXTILine_TimingSync_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOB clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);  
  
  //SI4463_Enable_NIRQ_RX();
  
  /* Connect EXTI Line0 to PB1 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);   //选择 GPIO管脚用作外部中断线路
  
  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line    = EXTI_Line7;                 //外部中断线
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;        //中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;       //中断触发方式
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     //打开中断
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel            = EXTI9_5_IRQn; //指定中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // 指定响应优先级别1
  NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Enable_Timing_Sync(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               // 抢占式优先级别
    NVIC_InitStructure.NVIC_IRQChannel            = EXTI9_5_IRQn; //指定中断源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;          // 指定响应优先级别1
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Disable_Timing_Sync(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               // 抢占式优先级别
    NVIC_InitStructure.NVIC_IRQChannel            = EXTI9_5_IRQn; //指定中断源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;           // 指定响应优先级别1
    NVIC_InitStructure.NVIC_IRQChannelCmd         = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TimingSync_IRQHandler()
{
  TimingSyncReset();
}

void ClkSwitch2HseSystemInit (void)
{
  ErrorStatus HSEStartUpStatus;
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();
  
  RCC_HSICmd(DISABLE); //Turn of the internal RC;
 
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);
 
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 
 
    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
 
    /* PLLCLK = 10MHz * 7 = 70 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);
 
    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);
 
    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
 
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
 
    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }  
}


//void TIM2_DMA_Init(void)
//{
//  //启动DMA时钟   
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
//  //启动ADC1时钟   
//  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  
//  //采样脚设置   
//  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
//  DMA_InitTypeDef DMA_InitStructure;
// /* DMA1 channel1 configuration ----------------------------------------------*/
//  DMA_DeInit(DMA1_Channel1);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32)&ADCConvertedValue;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//  DMA_InitStructure.DMA_BufferSize = 2;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
//  
//  /* Enable DMA1 channel1 */
//  DMA_Cmd(DMA1_Channel1, ENABLE);
//  
//  ADC_InitTypeDef ADC_InitStructure;
//  /* ADC1 configuration ------------------------------------------------------*/
//  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
//  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//  ADC_InitStructure.ADC_NbrOfChannel = 2;
//  ADC_Init(ADC1, &ADC_InitStructure);
//
//  /* ADC1 regular channel12,13 configuration */ 
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_55Cycles5); //Current
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 2, ADC_SampleTime_55Cycles5); //Voltage
//
//  /* Enable ADC1 DMA */
//  ADC_DMACmd(ADC1, ENABLE);
//  
//  /* Enable ADC1 */
//  ADC_Cmd(ADC1, ENABLE);
//
//  /* Enable ADC1 reset calibration register */   
//  ADC_ResetCalibration(ADC1);
//  /* Check the end of ADC1 reset calibration register */
//  while(ADC_GetResetCalibrationStatus(ADC1));
//  
//  /* Start ADC1 calibration */
//  ADC_StartCalibration(ADC1);
//  /* Check the end of ADC1 calibration */
//  while(ADC_GetCalibrationStatus(ADC1));
//     
//  /* Start ADC1 Software Conversion */ 
//  ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
//  
//  Init_Real_VA_List();
//}