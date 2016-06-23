/**/
/**/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include <stdlib.h>

#include "radio_mcu.h"
#include "radio.h"
#include "macro.h"
#include "si4463.h"
#include "si4463_def.h"
#include "radio_config.h"
#include "radio_comm.h"
#include "node.h" 

#include "uartapp.h"
#include "timeapp.h"


uint8_t uartRxPacketHead=0x00;
uint16_t uartRxPacketLength=0x00;
uint8_t uartRxLenCount=0x00;
uint16_t uartRxCount=0x0000;
uint8_t* uartRxDataBuff=NULL;

uint8_t uartRxFlag=0x00; //0: no data received, 1: data received
uint8_t uartRxDataCheckFail=0x00; // 1: check fail;
uint8_t SdkLenVer[]={0x00,0x00};

struct PxUsartBuf pub_rx, pub_tx;
uint8_t DmaUartTxBuf[UartFrameMaxLen];
uint8_t DmaUartRxBuf[UartFrameMaxLen];

enum UsartDmaTxSta usart_dma_tx_sta=IDLE;

void uartRxData(USART_TypeDef* USARTx)
{
  if(uartRxFlag==0x00)
  {
    if(uartRxPacketHead==0xAA) //Packet head is received
    {
      if(uartRxLenCount<2)
      {       
        if(uartRxLenCount<1)
        {
          uartRxCount++;
          uartRxLenCount++;
          SdkLenVer[0]=USART_ReceiveData(USARTx);     
        }
        else
        {
          if(uartRxLenCount<2)
          {
            uartRxCount++;
            uartRxLenCount++;
            SdkLenVer[1]=USART_ReceiveData(USARTx); 
            
            uartRxPacketLength=SdkLenVer[1];            
            uartRxPacketLength=(uartRxPacketLength<<8)+SdkLenVer[0];            
            
            if(uartRxPacketLength>0x0000&&uartRxPacketLength<=0x00FF)
            {
              uartRxDataBuff=(uint8_t*)malloc(uartRxPacketLength+6);
              uartRxCount+=5;
              *uartRxDataBuff=((uartRxPacketLength+6-2)>>8);
              *(uartRxDataBuff+1)=(uartRxPacketLength+6-2);
              *(uartRxDataBuff+2)=0xD3;
              *(uartRxDataBuff+3)=0x00;
              *(uartRxDataBuff+4)=PX_NUM;
              *(uartRxDataBuff+5)=0xAA;
              *(uartRxDataBuff+6)=SdkLenVer[0];
              *(uartRxDataBuff+7)=SdkLenVer[1];
            }
            else
            {
              uartRxPacketHead=0x00;
            }          
          }        
        }
      }
      else
      {
        if(uartRxCount<uartRxPacketLength+5)
        {
          *(uartRxDataBuff+uartRxCount)=USART_ReceiveData(USARTx);
          uartRxCount++;
          if(uartRxCount==(uartRxPacketLength+5))
          {
            U8 CheckByte=0x00;
            for(int ii=2;ii<uartRxCount;ii++)
            {
              CheckByte+=*(uartRxDataBuff+ii);          
            }
            *(uartRxDataBuff+uartRxCount)=(CheckByte-3); //0xD3 is not 0xD0;
            uartRxCount++;            
            uartRxFlag=0x01;  
            
//            UartSendByte(uartRxDataBuff, uartRxCount);
//            uartRxReset();
//            uartRxFlag=0x00; 
          }
        }
        else
        {
         uartRxReset();
        }
      }      
    }
    else
    {
      if(USART_ReceiveData(USARTx)==0xAA)
      {
        uartRxCount++;
        uartRxPacketHead=USART_ReceiveData(USARTx);
        uartRxDataCheckFail=0x00;
      }
      else
      {      
        uartRxPacketHead=0x00;       
      }
    }
  }
  else
  {
    USART_ReceiveData(USARTx);
  } 
}

void uartRxReset(void)
{
  uartRxPacketHead=0x00;
  uartRxPacketLength=0x00;
  if(uartRxDataBuff!=NULL)
  {
    //U8 ErrFlag[]={0xEE,0xF4};
    //UartSendByte(ErrFlag, 2); 
    free(uartRxDataBuff);
  }
  uartRxCount=0x00;
  uartRxFlag=0x00;
  uartRxLenCount=0x00;
}

uint8_t uartRxDataCheck(uint8_t* dataBuf, uint16_t dataLen)
{
  uint8_t tmp=0x00;
  for(uint16_t i=0;i<dataLen-1;i++) //include ctrlword;
  {
    tmp+=(*(dataBuf+i+2)); //check from ctrlword to data;
  }
  if(tmp==*(dataBuf+dataLen+1))
  {
    return 0x01;
  }
  else
  {
    return 0x00;
  }

}

uint8_t PxRfRxDataCheck(uint8_t* dataBuf, uint16_t dataLen)
{
  uint8_t tmp=0x00;
  for(uint16_t i=0;i<dataLen-1;i++) //include ctrlword;
  {
    tmp+=(*(dataBuf+i)); //check from ctrlword to data;
  }
  if(tmp==*(dataBuf+dataLen-1))
  {
    return 0x01;
  }
  else
  {
    return 0x00;
  }

}

extern uint8 PKT_Sent_Flag;

void uartDataProcess(void)
{  
  U16 uDataLen=uartRxDataBuff[0];
  uDataLen=(uDataLen<<8)+uartRxDataBuff[1];
  //UartSendByte(uartRxDataBuff, 2);
  
  if(uDataLen>0x00FF)
  {
    U8 ErrFlag[]={0xEE,0x01};
    UartSendByte(&ErrFlag, 2);
    uartRxReset();
    return;
  }  

  switch(uartRxDataBuff[2])
  {
  case 0xC0:
    uartRxReset();
    //back C1
    break;
  case 0xC2:
    uartRxReset();
    //back C3
    break;
  case 0xD0:
    //back D1
    //send data by RF
    //vRadio_SlotStartTx_Variable_Packet(0u,uartRxDataBuff[0]+1);    
    vRadio_StartTx_Variable_Packet(0u,uartRxDataBuff,uDataLen+2);
    while(PKT_Sent_Flag)
    {  
      
    }
    uartRxReset();
    break;
  case 0xD3:
    vRadio_StartTx_Variable_Packet(0u,uartRxDataBuff,uDataLen+2);
    while(PKT_Sent_Flag)
    {  
      
    }
    uartRxReset();
    break;
  default:
    uartRxReset();
    break;
  }  
}

void PubRxDataProcess(void)
{ 
  
  uint16_t length=pub_rx.Buf[0][7]; 
  length=(length<<8)+pub_rx.Buf[0][6];   
  
  if(length>0x0000&&length<=0x00F9)
  {
    pub_rx.Buf[0][0]=((length+6-2)>>8);
    pub_rx.Buf[0][1]=(length+6-2);
    pub_rx.Buf[0][2]=0xD3;
    pub_rx.Buf[0][3]=0x00;
    pub_rx.Buf[0][4]=PX_NUM;
    
    U8 CheckByte=0x00;
    for(int ii=2;ii<length+5;ii++)
    {
      CheckByte+=pub_rx.Buf[0][ii]; 
    }
    pub_rx.Buf[0][length+5]=(CheckByte-3); //0xD3 is not 0xD0;
    
    switch(pub_rx.Buf[0][2])
    {
    case 0xC0:
      //back C1
      break;
    case 0xC2:
      //back C3
      break;
    case 0xD0:
      //back D1
      //send data by RF
      //vRadio_SlotStartTx_Variable_Packet(0u,uartRxDataBuff[0]+1);
      //vRadio_StartTx_Variable_Packet(0u,uartRxDataBuff,uDataLen+2);
      break;
    case 0xD3:
      vRadio_StartTx_Variable_Packet(0u,pub_rx.Buf[0],length+6);
      break;
    default:
      break;
    }   
  } 
}

void PubTxDataProcess(void)
{ 
  if(usart_dma_tx_sta==IDLE)
  {
    uint16_t length=pub_tx.Buf[0][2]; 
    length=(length<<8)+pub_tx.Buf[0][1]; 
    if(length<UartFrameMaxLen)
    {
      PubTx2DmaTxBuf(length);
      LumMod_Uart_Start_DMA_Tx(length);
    }
  }
}


void PubRxShift(void)
{     
 if(pub_rx.BufCount>0&&pub_rx.BufCount<=PxUsartBufLen)
  {
    /* Enable USART2 Receive and Transmit interrupts */
    USART_ITConfig(LUMMOD_UART, USART_IT_IDLE, DISABLE);  // 开启 串口空闲IDEL 中断
    for(int i=0;i<pub_rx.BufCount;i++)
    {
      for(int j=0;j<UartFrameMaxLen;j++)
      {
        pub_rx.Buf[i][j]=pub_rx.Buf[i+1][j];      
      }
    }
    pub_rx.BufCount--;
    USART_ITConfig(LUMMOD_UART, USART_IT_IDLE, ENABLE);  // 开启 串口空闲IDEL 中断 
  }
  
}

void PubTxShift(void)
{
 
 if(pub_tx.BufCount>0&&pub_tx.BufCount<=PxUsartBufLen)
  {
    SI4463_Disable_NIRQ_Int(); 
    for(int i=0;i<pub_tx.BufCount;i++)
    {
      for(int j=0;j<UartFrameMaxLen;j++)
      {
        pub_tx.Buf[i][j]=pub_tx.Buf[i+1][j];      
      }
    }
    pub_tx.BufCount--;
    SI4463_Enable_NIRQ_Int() ;
  }

}

void buff2Packet(struct uartRxPacket* pkt)
{  
  (*pkt).packetLength=uartRxPacketLength;
  (*pkt).ctrlWord=*uartRxDataBuff;
  (*pkt).addr[0]=*(uartRxDataBuff+1);
  (*pkt).addr[1]=*(uartRxDataBuff+2);
  if(uartRxPacketLength>4)
  {
    for(int i=0;i<uartRxPacketLength-4;i++)
    {
      *((*pkt).data+i)=*(uartRxDataBuff+3+i);
    }
  }
  (*pkt).verifyByte=*(uartRxDataBuff+uartRxPacketLength-1);
  uartRxReset();
}


void PubRxTxInit(void)
{
  pub_rx.BufCount=0;
  pub_tx.BufCount=0;
}

void Uart_Init(void)
{
  PubRxTxInit();
  
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  /* System Clocks Configuration */
//= System Clocks Configuration ====================================================================//
   
  /* Enable GPIO clock */
  //RCC_APB2PeriphClockCmd(LUMMOD_UART_GPIO_CLK ,  ENABLE ); // 开启串口所在IO端口的时钟
  /* Enable USART Clock */
  //RCC_APB1PeriphClockCmd(LUMMOD_UART_CLK, ENABLE); // 开始串口时钟
  
   
//=NVIC_Configuration==============================================================================//
 
  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
  
  /* Enable the DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = LUMMOD_UART_Tx_DMA_IRQ;   // 发送DMA通道的中断配置
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 优先级设置
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USART Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = LUMMOD_UART_IRQn;     // 串口中断配置
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
   
//=GPIO_Configuration==============================================================================//
 
    //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);  // 我这里没有用默认IO口，所以进行了重新映射，这个可以根据自己的硬件情况配置选择
   
//    /* Configure USART3 Rx as input floating */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 串口接收IO口的设置
//    GPIO_InitStructure.GPIO_Pin = LUMMOD_UART_RxPin;
//    GPIO_Init(LUMMOD_UART_GPIO, &GPIO_InitStructure);
// 
//    /* Configure USART3 Tx as alternate function push-pull */
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 串口发送IO口的设置
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 这里设置成复用形式的推挽输出   
//    GPIO_InitStructure.GPIO_Pin = LUMMOD_UART_TxPin;
//    GPIO_Init(LUMMOD_UART_GPIO, &GPIO_InitStructure);
 
    DMA_Uart_Init();   // 串口 DMA 配置
 
    /* USART Format configuration ------------------------------------------------------*/
 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    // 串口格式配置
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    /* Configure USART2 */
    USART_InitStructure.USART_BaudRate = 38400;  //  波特率设置
    //USART_Init(LUMMOD_UART, &USART_InitStructure);
    STM_EVAL_COMInit(COM2, &USART_InitStructure);
    
    /* Enable USART2 Receive and Transmit interrupts */
    USART_ITConfig(LUMMOD_UART, USART_IT_IDLE, ENABLE);  // 开启 串口空闲IDEL 中断
   
    /* Enable the USART2 */
    USART_Cmd(LUMMOD_UART, ENABLE);  // 开启串口
    /* Enable USARTy DMA TX request */
    USART_DMACmd(LUMMOD_UART, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    USART_DMACmd(LUMMOD_UART, USART_DMAReq_Rx, ENABLE); // 开启串口DMA接收
}

void DMA_Uart_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
   
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 开启DMA1时钟
   
   
//=DMA_Configuration==============================================================================//
 
/*--- LUMMOD_UART_Tx_DMA_Channel DMA Config ---*/
 
    DMA_Cmd(LUMMOD_UART_Tx_DMA_Channel, DISABLE);                           // 关DMA通道
    DMA_DeInit(LUMMOD_UART_Tx_DMA_Channel);                                 // 恢复缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&LUMMOD_UART->DR);// 设置串口发送数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DmaUartTxBuf;         // 设置发送缓冲区首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // 设置外设位目标，内存缓冲区 -> 外设寄存器
    DMA_InitStructure.DMA_BufferSize = 0;                     // 需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
    DMA_Init(LUMMOD_UART_Tx_DMA_Channel, &DMA_InitStructure);               // 写入配置
    DMA_ClearFlag(LUMMOD_UART_Tx_DMA_FLAG);                                 // 清除DMA所有标志
    DMA_Cmd(LUMMOD_UART_Tx_DMA_Channel, DISABLE); // 关闭DMA
    DMA_ITConfig(LUMMOD_UART_Tx_DMA_Channel, DMA_IT_TC, ENABLE);            // 开启发送DMA通道中断
   
/*--- LUMMOD_UART_Rx_DMA_Channel DMA Config ---*/
 
    DMA_Cmd(LUMMOD_UART_Rx_DMA_Channel, DISABLE);                           // 关DMA通道
    DMA_DeInit(LUMMOD_UART_Rx_DMA_Channel);                                 // 恢复缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&LUMMOD_UART->DR);// 设置串口接收数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DmaUartRxBuf;         // 设置接收缓冲区首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // 设置外设为数据源，外设寄存器 -> 内存缓冲区
    DMA_InitStructure.DMA_BufferSize = UartFrameMaxLen;                     // 需要最大可能接收到的字节数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
    DMA_Init(LUMMOD_UART_Rx_DMA_Channel, &DMA_InitStructure);               // 写入配置
    DMA_ClearFlag(LUMMOD_UART_Rx_DMA_FLAG);                                 // 清除DMA所有标志
    DMA_Cmd(LUMMOD_UART_Rx_DMA_Channel, ENABLE);                            // 开启接收DMA通道，等待接收数据   
}

void USART_IDLE_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)  // 空闲中断
    {
        LumMod_Uart_DMA_Rx_Data();
        USART_ReceiveData( USART2 ); // Clear IDLE interrupt flag bit
    }
}

void LumMod_Uart_DMA_Rx_Data(void)
{
    DMA_Cmd(LUMMOD_UART_Rx_DMA_Channel, DISABLE);       // 关闭DMA ，防止干扰
    DMA_ClearFlag( LUMMOD_UART_Rx_DMA_FLAG );           // 清DMA标志位
    uint16_t DmaUsartRxLen = UartFrameMaxLen - DMA_GetCurrDataCounter(LUMMOD_UART_Rx_DMA_Channel); //获得接收到的字节数
    DmaRxBuf2PubRx(DmaUsartRxLen);
    LUMMOD_UART_Rx_DMA_Channel->CNDTR = UartFrameMaxLen;    //  重新赋值计数值，必须大于等于最大可能接收到的数据帧数目
    DMA_Cmd(LUMMOD_UART_Rx_DMA_Channel, ENABLE);        
    /* DMA 开启，等待数据。注意，如果中断发送数据帧的速率很快，MCU来不及处理此次接收到的数据，中断又发来数据的话，这里不能开启，否则数据会被覆盖。有2种方式解决。
    1. 在重新开启接收DMA通道之前，将LumMod_Rx_Buf缓冲区里面的数据复制到另外一个数组中，然后再开启DMA，然后马上处理复制出来的数据。
    2. 建立双缓冲，在LumMod_Uart_DMA_Rx_Data函数中，重新配置DMA_MemoryBaseAddr 的缓冲区地址，那么下次接收到的数据就会保存到新的缓冲区中，不至于被覆盖。*/
    //OSMboxPost(mbLumModule_Rx,  LumMod_Rx_Buf); // 发送接收到新数据标志，供前台程序查询
}

void DmaRxBuf2PubRx(uint16_t PacketLen)
{
  if(PacketLen<=(UartFrameMaxLen-6)&&pub_rx.BufCount<PxUsartBufLen&&DmaUartRxBuf[0]==0xAA)
  {
    for(int i=0;i<PacketLen;i++)
    {
      pub_rx.Buf[pub_rx.BufCount][i+5]=DmaUartRxBuf[i];
    }
    pub_rx.BufCount++;
  } 
}

void RfRxData2PubTx(u8 *buf, u32 len)
{
  if(len<=(UartFrameMaxLen-6)&&pub_tx.BufCount<PxUsartBufLen)
  {
    for(int i=0;i<len;i++)
    {
      pub_tx.Buf[pub_tx.BufCount][i]=buf[i];
    }
    pub_tx.BufCount++;
  } 
}

void DMA_Channel7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_FLAG_TC7))
    {
        LumMod_Uart_DAM_Tx_Over();
    }
}

void LumMod_Uart_DAM_Tx_Over(void)
{
    DMA_ClearFlag(LUMMOD_UART_Tx_DMA_FLAG);         // 清除标志
    DMA_Cmd(LUMMOD_UART_Tx_DMA_Channel, DISABLE);   // 关闭DMA通道
    usart_dma_tx_sta=IDLE;
    //OSMboxPost(mbLumModule_Tx, (void*)1);           // 设置标志位，这里我用的是UCOSII ，可以根据自己的需求进行修改

}

void LumMod_Uart_Start_DMA_Tx(uint16_t size)
{
  if(usart_dma_tx_sta==IDLE)
  {  
    usart_dma_tx_sta=BUSY;
    LUMMOD_UART_Tx_DMA_Channel->CNDTR = (uint16_t)size; // 设置要发送的字节数目
    DMA_Cmd(LUMMOD_UART_Tx_DMA_Channel, ENABLE);        //开始DMA发送
  }
}


void PubTx2DmaTxBuf(uint16_t PacketLen)
{
  if(PacketLen<=UartFrameMaxLen&&pub_tx.BufCount>0&&usart_dma_tx_sta==IDLE)
  {
    for(int i=0;i<PacketLen;i++)
    {
      DmaUartTxBuf[i]=pub_tx.Buf[0][i];
    }
    PubTxShift();
  } 
}

uint16_t GetPubRxBufCount(void)
{
  return pub_rx.BufCount;
}

uint16_t GetPubTxBufCount(void)
{
  return pub_tx.BufCount;
}

