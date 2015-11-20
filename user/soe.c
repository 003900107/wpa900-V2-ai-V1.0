
/*écrivain : chinuxer 
SOE 触发引擎      袁博

务要传道，无论得时不得时，总要专心，并用百般的忍耐，各样的教训。因为时候要到，人必厌烦
纯正的道理，耳朵发痒，都随从自己的情欲。掩耳不听真道，偏向荒谬的言语。你却要凡事谨慎，
忍受苦难，作传道的功夫，尽你的职分。我现在被浇奠，我离世的时候到了。那艰苦的仗我已经打
过了，当赶的路我已经跑尽了，当信的道我已经守住了。从此以后，有公义的冠冕为我存留，就是
按着公义审判的主到了那日要赐给我的；不但赐给我，也赐给凡爱慕他显现的人。	
提多书	 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "soe.h"
#include "Realtimedb.h"


#define MAX_SOE_SIZE  32
/////////////////////////////////
extern Setting SetCurrent;

SOE_Struct SOE_Queue[MAX_SOE_SIZE];
static uint8_t inst_seq = 0;
static uint8_t dis_update = 0;
const uint8_t single_len = sizeof(SOE_Struct);
/////////////////////////////////
void soe_engine( u8 i, u8 state)
{
  SOE_Struct temp_soe;
  
  temp_soe.state = state;
  temp_soe.diseq = i;
  temp_soe.timetamp = (time_t)RTC_GetCounter();
  temp_soe.mscd  = Get_Msec();
  
  Soe_enqueue(&temp_soe);	
}

//添加新记录到soe队列
void Soe_enqueue(SOE_Struct *insert)
{
  if(inst_seq >= MAX_SOE_SIZE)
    inst_seq = 0;
  
  memcpy(&SOE_Queue[inst_seq], insert, single_len);
  inst_seq++;
  dis_update++;
}

//是否有新的soe记录
uint8_t Is_new_soe()
{
  return dis_update;
}

//获取未上传的soe记录数
uint8_t get_soe(SOE_Struct* pSoeBuf, uint8_t count)
{
  uint8_t last = 0;
  
  if(pSoeBuf == NULL)
    return 0xff;
  
  if(inst_seq < dis_update) //队列以更新翻转
  {
    last = dis_update-inst_seq; //获得队列后端新增加的数量
    if(count > last)
    {
      memcpy(pSoeBuf, &SOE_Queue[MAX_SOE_SIZE-last], last*single_len);
      memcpy(pSoeBuf+(last*single_len), &SOE_Queue[0], (count-last)*single_len);
    }
    else
    {
      memcpy(pSoeBuf, &SOE_Queue[MAX_SOE_SIZE-last], count*single_len);
    }
  }
  else
    memcpy(pSoeBuf, &SOE_Queue[inst_seq-dis_update], count*single_len);
  
  dis_update -= count;    //更新soe新记录数
  
  return dis_update;
}


