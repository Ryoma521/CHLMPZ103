/**
  ******************************************************************************
  * @file    USART/Printf/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include <stdlib.h>



#include "macro.h"
#include "radio.h"

#include "node.h" 
    
#include "uartapp.h" 
#include "timeapp.h" 
#include "ledapp.h"
    
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern SEGMENT_VARIABLE_SEGMENT_POINTER(pRadioConfiguration, tRadioConfiguration) ;

extern uint16_t uartRxCount;
extern uint8_t* uartRxDataBuff;
extern uint8_t uartRxFlag; //0: no data received, 1: data received
extern uint8_t uartRxDataCheckFail; 

extern enum RF_STATE rf_state;//0:tx,1:rx,2:rdy

extern uint8_t rfRxFlag;
extern uint8_t xxx[];

extern __IO uint8_t RxCounter1;

extern enum SlotState SS_Global;
extern uint8 PKT_Sent_Flag;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{   
  RCC_ClocksTypeDef RCC_ClockFreq;
  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  if (SysTick_Config(RCC_ClockFreq.HCLK_Frequency/1000))
  { 
    /* Capture error */ 
    while (1);
  }
  
  Il_Hw_Init(); 
  
  Init_SI4463_Pin();
  
  vRadio_Init(); 
  
  ClkSwitch2HseSystemInit();

  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  if (SysTick_Config(RCC_ClockFreq.HCLK_Frequency/1000))
  { 
    /* Capture error */ 
    while (1);
  }
  
  Il_Hw_Init(); 
  
  Uart_Init();
    
  Init_SI4463_Pin();
  
//TimingBaseInit(50000);
  
//  while(1)
//  {
////  //LedD4StaInvert();  
////    GPIOB->BSRR  = 0x00000040;
////    GPIOB->BRR  = 0x00000040;
//  }
  
  //EXTILine_TimingSync_Config();
  
  //si446x_get_int_status(0u, 0u, 0u);
  uint8_t testBuff[6]={0x05,0xD0,0x01,0x02,0x03,0xD6};
  
  //UartSendByte(testBuff, 6); 
  
  vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber);
  
  RadioGotoRxSta();
  
  SI4463_Enable_NIRQ_Int();
  
  while(1)
  { 
    //vRadio_StartTx_Variable_Packet(0u,&trx_state,1);
    
    //Px rx sync?
    if(SS_Global==S_RTS)
    {

    }
    
    //Px tx data?
    if((SS_Global==S_P1T)&&(PX_NUM==1))
    {
      //unsigned int timMark=GetTimingBase();
      if(GetPubRxBufCount()>0x00&&(GetTimingBase())<(TIME_SLOT_LEN/3)&&(GetTimingBase())>2&&(GetPktSendFlag()==0))
      {
        //UartSendByte(timMark, 1);
        //UartSendByte(uartRxDataBuff, uartRxCount);
        //uartDataProcess();
        PubRxDataProcess();
      }    
    }
    
    //Px tx data?
    if(SS_Global==S_P2T&&(PX_NUM==2))
    {
      if(GetPubRxBufCount()>0x00&&(GetTimingBase())<(TIME_SLOT_LEN/3)&&(GetTimingBase())>2&&(GetPktSendFlag()==0))
      {
        //UartSendByte(timMark, 1);
        //UartSendByte(uartRxDataBuff, uartRxCount);
        //uartDataProcess();
        PubRxDataProcess();
      }  
    }
    
    //RF receive data?
    if(GetPubTxBufCount()>0x00)
    {
      PubTxDataProcess();
    }   

  }

}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
