
/*��crivain : chinuxer 
SOE ��������      Ԭ��

��Ҫ���������۵�ʱ����ʱ����Ҫר�ģ����ðٰ�����ͣ������Ľ�ѵ����Ϊʱ��Ҫ�����˱��ᷳ
�����ĵ������䷢����������Լ����������ڶ����������ƫ������������ȴҪ���½�����
���ܿ��ѣ��������Ĺ��򣬾����ְ�֡������ڱ����죬��������ʱ���ˡ��Ǽ��������Ѿ���
���ˣ����ϵ�·���Ѿ��ܾ��ˣ����ŵĵ����Ѿ���ס�ˡ��Ӵ��Ժ��й���Ĺ���Ϊ�Ҵ���������
���Ź������е�����������Ҫ�͸��ҵģ������͸��ң�Ҳ�͸�����Ľ�����ֵ��ˡ�	
�����	 */

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

//����¼�¼��soe����
void Soe_enqueue(SOE_Struct *insert)
{
  if(inst_seq >= MAX_SOE_SIZE)
    inst_seq = 0;
  
  memcpy(&SOE_Queue[inst_seq], insert, single_len);
  inst_seq++;
  dis_update++;
}

//�Ƿ����µ�soe��¼
uint8_t Is_new_soe()
{
  return dis_update;
}

//��ȡδ�ϴ���soe��¼��
uint8_t get_soe(SOE_Struct* pSoeBuf, uint8_t count)
{
  uint8_t last = 0;
  
  if(pSoeBuf == NULL)
    return 0xff;
  
  if(inst_seq < dis_update) //�����Ը��·�ת
  {
    last = dis_update-inst_seq; //��ö��к�������ӵ�����
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
  
  dis_update -= count;    //����soe�¼�¼��
  
  return dis_update;
}


