#ifndef __SOE_H
#define __SOE_H

#include <time.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "RTC_Time.h"

typedef struct tag_SOEFLAG
{
  unsigned int state:1;	
  unsigned int p103trig:1;	
  unsigned int PADtrig:1;
  unsigned int termtrig:1;	
  unsigned int reserved:4;	
}soeflag;

typedef struct tag_SOESTRUCT
{	
  uint8_t state;
  uint8_t diseq;
  soeflag flag;
  time_t timetamp;
  uint16_t mscd;
}SOE_Struct;

typedef struct tag_ERRORSTRUCT
{	
  uint32_t Perirh_baseaddr;
  uint32_t Register_1;
  uint32_t Register_2;
  uint32_t Register_3;
  uint32_t Register_4;
  time_t timetamp;
  uint16_t mscd;
}ERROR_Struct;

void soe_engine( uint8_t i, uint8_t state);
void Soe_enqueue(SOE_Struct *insert);

uint8_t Is_new_soe();
uint8_t get_soe(SOE_Struct* pSoeBuf, uint8_t count);


#endif


/******************* 风电工程部chinuxer *****la fin de document****/
