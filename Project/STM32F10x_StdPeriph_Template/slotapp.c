

int GetSlotNum(void)
{
  unsigned int timepoint=GetTimingBase();
  int SlotNum=0;
  SlotNum=timepoint/250;
  return SlotNum;
}


void SlotCycle(void)
  {


  }
