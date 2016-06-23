#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#include "radio_mcu.h"


extern uint8 trx_state;//0:tx,1:rx

/*
  初始化与SI4463相关的引脚
*/
void Init_SI4463_Pin(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
     /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
  
  /*PA4,SDN*/
  GPIO_InitStruct.GPIO_Pin = PIN_SDN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_SDN, &GPIO_InitStruct);   
  
  /*PB0,SCLK*/
  GPIO_InitStruct.GPIO_Pin = PIN_SCLK;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_SCLK, &GPIO_InitStruct);
  
  /*PC5,SDO*/
  GPIO_InitStruct.GPIO_Pin = PIN_SDO;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(PORT_SDO, &GPIO_InitStruct);
  
  /*PC4,SDI*/
  GPIO_InitStruct.GPIO_Pin = PIN_SDI;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_SDI, &GPIO_InitStruct);
  
  /*PA7,NSEL*/
  GPIO_InitStruct.GPIO_Pin = PIN_NSEL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_NSEL, &GPIO_InitStruct);  
 
  EXTILineNIRQ_Config();
  
}

void SI4463_SDN_UP(void)
{
  GPIO_SetBits(PORT_SDN, PIN_SDN);
}

void SI4463_SDN_DOWN(void)
{
  GPIO_ResetBits(PORT_SDN, PIN_SDN);
}

void SI4463_SCLK_Up(void)
{
  GPIO_SetBits(PORT_SCLK, PIN_SCLK);
}

void SI4463_SCLK_Down(void)
{
  GPIO_ResetBits(PORT_SCLK, PIN_SCLK);
}

void SI4463_SDI_Up(void)
{
  GPIO_SetBits(PORT_SDI, PIN_SDI);
}

void SI4463_SDI_Down(void)
{
  GPIO_ResetBits(PORT_SDI, PIN_SDI);
}

uint8 SI4463_SDO_Bit(void)
{
  return (GPIO_ReadInputDataBit(PORT_SDO, PIN_SDO));
}

uint8 SI4463_NIRQ_Bit(void)
{
  return (GPIO_ReadInputDataBit(PORT_NIRQ, PIN_NIRQ));
}

void SI4463_NSEL_Up(void)
{
  GPIO_SetBits(PORT_NSEL, PIN_NSEL);
}

void SI4463_NSEL_Down(void)
{
  GPIO_ResetBits(PORT_NSEL, PIN_NSEL);
}

void SI4463_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               // 抢占式优先级别
    NVIC_InitStructure.NVIC_IRQChannel            = NIRQ_IRQChannel; //指定中断源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // 指定响应优先级别1
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               // 抢占式优先级别
    NVIC_InitStructure.NVIC_IRQChannel            = GDO2_IRQChannel; //指定中断源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 指定响应优先级别1
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void SI4463_EXTI_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line    = NIRQ_LINE;                 //外部中断线
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;        //中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;       //中断触发方式
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     //打开中断
    EXTI_Init(&EXTI_InitStructure);    
    GPIO_EXTILineConfig(NIRQ_PortSource, NIRQ_PinSource);   //选择 GPIO管脚用作外部中断线路    
}


void SI4463_Enable_NIRQ_TX()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*PB1,NIRQ*/
  GPIO_InitStruct.GPIO_Pin = PIN_NIRQ;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_NIRQ, &GPIO_InitStruct);
}

void SI4463_Enable_NIRQ_RX()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Configure PB1 pin as input floating */
  GPIO_InitStruct.GPIO_Pin = PIN_NIRQ;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_Init(PORT_NIRQ, &GPIO_InitStruct);
}

void SI4463_Enable_NIRQ_Int(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               // 抢占式优先级别
    NVIC_InitStructure.NVIC_IRQChannel            = NIRQ_IRQChannel; //指定中断源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;          // 指定响应优先级别1
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void SI4463_Disable_NIRQ_Int(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);               // 抢占式优先级别
    NVIC_InitStructure.NVIC_IRQChannel            = NIRQ_IRQChannel; //指定中断源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;           // 指定响应优先级别1
    NVIC_InitStructure.NVIC_IRQChannelCmd         = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*************************************************************************
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
*************************************************************************/
void EXTILineNIRQ_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);  
  
  SI4463_Enable_NIRQ_RX();
  
  /* Connect EXTI Line0 to PB1 pin */
  GPIO_EXTILineConfig(NIRQ_PortSource, NIRQ_PinSource);   //选择 GPIO管脚用作外部中断线路
  
  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line    = NIRQ_LINE;                 //外部中断线
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;        //中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;       //中断触发方式
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     //打开中断
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel            = NIRQ_IRQChannel; //指定中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // 指定响应优先级别1
  NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Change_NIRQ_Falling_Int(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line    = NIRQ_LINE;                 //外部中断线
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;        //中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;       //中断触发方式
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     //打开中断
    EXTI_Init(&EXTI_InitStructure);
    GPIO_EXTILineConfig(NIRQ_PortSource, NIRQ_PinSource);   //选择 GPIO管脚用作外部中断线路   
}

void Change_NIRQ_Rising_Int(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line    = NIRQ_LINE;                 //外部中断线
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;        //中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;       //中断触发方式
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                     //打开中断
    EXTI_Init(&EXTI_InitStructure);
    GPIO_EXTILineConfig(NIRQ_PortSource, NIRQ_PinSource);   //选择 GPIO管脚用作外部中断线路   
}

void RF_Switch_RX()
{
  //SI4463_SW1_DOWN();
  //SI4463_SW2_UP();
}

void RF_Switch_TX()
{
//  SI4463_SW2_DOWN();
//  SI4463_SW1_UP();
}

void SI4463_SW1_UP(void)
{
  //GPIO_SetBits(PORT_ANTSW1, PIN_ANTSW1);
}

void SI4463_SW1_DOWN(void)
{
  //GPIO_ResetBits(PORT_ANTSW1, PIN_ANTSW1);
}

void SI4463_SW2_UP(void)
{
  GPIO_SetBits(PORT_ANTSW2, PIN_ANTSW2);
}

void SI4463_SW2_DOWN(void)
{
  GPIO_ResetBits(PORT_ANTSW2, PIN_ANTSW2);
}

