#ifndef RADIO_H
#define RADIO_H

#include "hal_types.h"

#define RADIO_MAX_PACKET_LENGTH     64u

typedef struct
{
    U8   *Radio_ConfigurationArray;
    U8   Radio_ChannelNumber;
    U16   Radio_PacketLength;
    U8   Radio_State_After_Power_Up;
    U16  Radio_Delay_Cnt_After_Reset;
} tRadioConfiguration;

typedef struct
{
    U8   *Freq_ConfigurationArray;
} tFreqConfiguration;

enum RF_RX_FLAG
{
  RfNoData,
  RfRxData
};

enum RF_STATE
{
 RF_TX=0,//RF tx state
 RF_RX,  //RF tx state 
 RF_RDY  //RF ready state
};



void vRadio_Init(void);
uint8 gRadio_CheckReceived(void);
uint8 gRadio_CheckReceived_VariablePacket(void);
uint8 gRadio_CheckTransmitted(void);
void vRadio_StartRX(U8 channel);
uint8 vRadio_StartTx(U8 channel, U8 *pioFixRadioPacket,U8 len);

void RXHandlerInGDO0_SI4463(void);

uint8 gSampleCode_StringCompare(U8* pbiPacketContent, U8* pbiString, U8 lenght);

uint8 vSampleCode_SendFixPacket(U8 * Packet,U8 len);
void SI4463_Transmit(U8 * Packet,U8 length);

void DemoApp_Pollhandler_RX();

uint8 compare(uint32 *input);

void vRadio_StartTx_Variable_Packet(U8 channel, U8 *pioRadioPacket, U16 length);
void gRadio_CheckReceived_ExtreLongPkt(void);

void RadioGotoRxSta(void);
void RadioGotoRdySta(void);
enum RF_RX_FLAG GetRadioSta(void);

#endif
