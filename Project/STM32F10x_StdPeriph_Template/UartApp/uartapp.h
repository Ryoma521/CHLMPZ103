/**/
/**/

#ifndef __UARTAPP_H
#define __UARTAPP_H


#define PxUsartBufLen 4
#define UartFrameMaxLen 0x00FF

#define LUMMOD_UART                      USART2
//#define LUMMOD_UART_GPIO                 GPIOC
//#define LUMMOD_UART_CLK                  RCC_APB1Periph_USART3
//#define LUMMOD_UART_GPIO_CLK        RCC_APB2Periph_GPIOC
//#define LUMMOD_UART_RxPin               GPIO_Pin_11
//#define LUMMOD_UART_TxPin               GPIO_Pin_10
#define LUMMOD_UART_IRQn                USART2_IRQn
#define LUMMOD_UART_DR_Base                  (USART2_BASE + 0x4)  //0x40013804
 
#define LUMMOD_UART_Tx_DMA_Channel      DMA1_Channel7
#define LUMMOD_UART_Tx_DMA_FLAG         DMA1_FLAG_GL7//DMA1_FLAG_TC2 | DMA1_FLAG_TE2 
#define LUMMOD_UART_Tx_DMA_IRQ          DMA1_Channel7_IRQn
 
#define LUMMOD_UART_Rx_DMA_Channel      DMA1_Channel6
#define LUMMOD_UART_Rx_DMA_FLAG         DMA1_FLAG_GL6//DMA1_FLAG_TC3 | DMA1_FLAG_TE3 
#define LUMMOD_UART_Rx_DMA_IRQ      DMA1_Channel3_IRQn

struct uartRxPacket
{
  uint8_t packetLength;
  uint8_t ctrlWord;
  uint8_t addr[2];
  uint8_t* data;
  uint8_t verifyByte;  
};

struct PxUsartBuf
{
  uint16_t BufCount;
  uint8_t Buf[PxUsartBufLen][UartFrameMaxLen];
};

enum UsartDmaTxSta
{
  IDLE=0,
  BUSY
};

void uartRxData(USART_TypeDef* USARTx);
void uartRxReset(void);
uint8_t uartRxDataCheck(uint8_t* dataBuf, uint16_t dataLen);
void buff2Packet(struct uartRxPacket* pkt);
void uartDataProcess(void);
void DMA_Uart_Init(void);
void LumMod_Uart_DMA_Rx_Data(void);
void DmaRxBuf2PubRx(uint16_t PacketLen);
void LumMod_Uart_DAM_Tx_Over(void);
void PubTx2DmaTxBuf(uint16_t PacketLen);
void LumMod_Uart_Start_DMA_Tx(uint16_t size);

#endif
