#ifndef __TIMEAPP_H
#define __TIMEAPP_H

#define PX_NUM 1
#define TIME_SLOT_LEN 40

void TimingDelay_Decrement(void);
void DelayMs(unsigned int MsTime);
void NopDelayMs(short int x);
void TIM2_IsTimeOn(void);
unsigned int GetTimingBase(void);
enum SlotState
{
 S_P1T=0,//p1 tx data
 S_P2T,  //p2 tx data
 S_NULL, //px null
 S_RTS,  //Remote tx sync
 S_RTD  //Remote tx data 
};


#endif