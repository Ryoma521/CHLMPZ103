#include "stm32f10x.h"
#include "ledapp.h"

static char LedD3Sta=0x00;
static char LedD4Sta=0x00;

void LedD3On(void) //PB5
{
  GPIOB->BRR |= 0x00000020;
}

void LedD3Off(void) //PB5
{
  GPIOB->BSRR  |= 0x00000020;
}

void LedD4On(void) //PB6
{
  GPIOB->BRR |= 0x00000040;
}

void LedD4Off(void) //PB6
{
  GPIOB->BSRR  |= 0x00000040;
}

void LedD3StaInvert(void)
{

  if(LedD3Sta==0x00)
  {
    LedD3Off();
    LedD3Sta=0x01;
  }
  else
  {
    LedD3On();
    LedD3Sta=0x00;
  }
}

void LedD4StaInvert(void)
{

  if(LedD4Sta==0x00)
  {
    LedD4Off();
    LedD4Sta=0x01;
  }
  else
  {
    LedD4On();
    LedD4Sta=0x00;
  }
}