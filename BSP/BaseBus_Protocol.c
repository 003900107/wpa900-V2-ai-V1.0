/*
*     @arg I2C_EVENT_SLAVE_ADDRESS_MATCHED   : EV1
*     @arg I2C_EVENT_SLAVE_BYTE_RECEIVED     : EV2
*     @arg I2C_EVENT_SLAVE_BYTE_TRANSMITTED  : EV3
*     @arg I2C_EVENT_SLAVE_ACK_FAILURE       : EV3-2
*     @arg I2C_EVENT_MASTER_MODE_SELECT      : EV5
*     @arg I2C_EVENT_MASTER_MODE_SELECTED    : EV6
*     @arg I2C_EVENT_MASTER_BYTE_RECEIVED    : EV7
*     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTED : EV8
*     @arg I2C_EVENT_MASTER_MODE_ADDRESS10   : EV9
*     @arg I2C_EVENT_SLAVE_STOP_DETECTED     : EV4

========================================

*/
#include "stm32f10x_i2c.h"
#include "bsp.h"
#include "BaseBus_Protocol.h"
#include "I2CRoutines.h"


#include <string.h>
#include <stdbool.h>

//unsigned char RxBuffer[8];
//#if I2C_METHOD == SINGLEBYTE
//unsigned char TxBuffer[8];
//#endif
//#if MEA_UPDATE_METHOD == MEMBLKCP
//unsigned char TxBuffer[127];
//#endif
uint8_t CommingCall=0;
__IO uint8_t COEF_INTFLAG=0;
__IO uint8_t Times_of_Setcoef=0;
__IO uint8_t SlaveReceptionComplete = 0;

uint8_t Begin_AI_Send = 0;

extern Setting SetCurrent;
extern I2C_InitTypeDef I2C_InitStruct;

float MeaTab[MEANUM];   //遥测值列表
uint16_t DiTab;         //遥信值列表

//uint32_t EV_Word;
//u32 EV[256];

//uint16_t Amp_Coef[10]={1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,};

uint32_t Ang_Coef[10];

#define HEADCHECK(a,b)  ((a==0xA5&&b==0x5A)?1:0)
#define BufferSize  8

#define I2C_PollingWait 0xfff

__IO uint8_t AinQuerry_index;

//uint16_t checksum16(uint8_t *pByte , uint16_t Len )
//{
//  uint16_t sum=0,i=0;
//  while(Len--)
//    sum+=*(pByte+i++); 
//  return sum;
//}

void AiResponse(unsigned char AiSeq)
{
  uint16_t bufLen;
//  
//#if I2C_METHOD == SINGLEBYTE
//  TxBuffer[3]=AiSeq;
//  //TxBuffer[4]=*p++;
//  //TxBuffer[5]=*p++;
//  //TxBuffer[6]=*p++;
//  //TxBuffer[7]=*p;
//  memcpy((u8*)(&TxBuffer[4]),&(MeaTab[AiSeq]),4);
//#endif
  
  TxBuffer[0]=0xA5;
  TxBuffer[1]=0x5A;
  TxBuffer[2]=AI_RES;
  TxBuffer[3]=UartAddr;
  memcpy((uint8_t*)(&TxBuffer[4]), &MeaTab, 96);
  
  bufLen = set_CRC(TxBuffer, 100);
//  CRC_ResetDR();
//  CRCValue =CRC_CalcBlockCRC((uint32_t *)TxBuffer, /*24*/25);        
//  TxBuffer[/*96*/100]=CRCValue&0xFF;
//  TxBuffer[/*97*/101]=(CRCValue>>8)&0xFF;
  
  if(bufLen > 0)
    SendUartData(bufLen);    //uart发送数据
  
  memset(TxBuffer, 0, TX_BUFSIZE);
}

void DiResponse(unsigned char AiSeq)
{
  uint16_t bufLen;
  
  TxBuffer[0]=0xA5;
  TxBuffer[1]=0x5A;
  TxBuffer[2]=AI_DIO;
  TxBuffer[3]=UartAddr;
  memcpy((uint8_t*)(&TxBuffer[4]), &DiTab, 2);
 
  bufLen = set_CRC(TxBuffer, 6);
  if(bufLen > 0)
    SendUartData(bufLen);    //uart发送数据
  
  memset(TxBuffer, 0, TX_BUFSIZE);
}


void SoeResponse(const SOE_Struct SoeBuf)
{
  uint16_t bufLen;
  
  TxBuffer[0]=0xA5;
  TxBuffer[1]=0x5A;
  TxBuffer[2]=AI_COS;
  TxBuffer[3]=UartAddr;  
  bufLen = sizeof(SOE_Struct);
  memcpy((uint8_t*)(&TxBuffer[4]), &SoeBuf, bufLen);
 
  bufLen = set_CRC(TxBuffer, bufLen+4);
  if(bufLen > 0)
    SendUartData(bufLen);    //uart发送数据
  
  memset(TxBuffer, 0, TX_BUFSIZE);
}


//处理通道系数设置
bool AiCoefProcess(unsigned char AiSeq)
{
  uint8_t result, status;
  uint8_t i;
  
  result = status = false;
  
  if( (AiSeq>0)&&(AiSeq<10) )
  {
    memcpy((uint8_t*)(&(SetCurrent.ChannelCoef[AiSeq-1])), &(RxBuffer[4]),2);
    status = true;
  } 
  
  if(10 == AiSeq)
  {
    for(i=0;i<9;i++) 
    {
      SetCurrent.ChannelCoef[i] =1000;
    }
    status = true;
  }
  
  if(status)
    result = DataBase_Write(STORAGE_ROMADDR, (uint32_t *)(&SetCurrent), sizeof(Setting));
  
  return result;     
}

//全通道系数自动校准
bool AiCalibration(void)
{
  u8 status;
  //__IO uint8_t Timeout=0xff;
  
  SetCurrent.ChannelCoef[0] =(uint16_t)(57.74/MeaTab[0]*1000+0.5);
  SetCurrent.ChannelCoef[1] =(uint16_t)(57.74/MeaTab[2]*1000+0.5);
  SetCurrent.ChannelCoef[2] =(uint16_t)(57.74/MeaTab[4]*1000+0.5);
  SetCurrent.ChannelCoef[3] =(uint16_t)(5.000/MeaTab[6]*1000+0.5);
  SetCurrent.ChannelCoef[4] =(uint16_t)(5.000/MeaTab[8]*1000+0.5);
  SetCurrent.ChannelCoef[5] =(uint16_t)(5.000/MeaTab[10]*1000+0.5);
  SetCurrent.ChannelCoef[6] =(uint16_t)(100.0/MeaTab[12]*1000+0.5);
  SetCurrent.ChannelCoef[7] =(uint16_t)(100.0/MeaTab[13]*1000+0.5);
  SetCurrent.ChannelCoef[8] =(uint16_t)(100.0/MeaTab[14]*1000+0.5);
  
  status=DataBase_Write(STORAGE_ROMADDR, (uint32_t *)(&SetCurrent), sizeof(Setting));	
  if(status)
    return true;
  else 
    return false;   	
}

//tyh: 20150629 响应i2c通信数据
void Deal_Readed(void)
{
  AIresponse(AinQuerry_index++%MEANUM) ;
}

//对时报文处理函数
bool TimeCalibration(uint8_t *pos)
{
  u32 tempcounter;
  
  tempcounter=*((uint32_t*)pos);
  
  Time_SetUnixTime(tempcounter);
  
  return true;
}

////////////////////////////////////////
bool BusCalling_Process(uint16_t lenght)
{
  uint8_t pBuf[300];
  uint8_t result;
  
  //  if(Flag_Uart_Recv == 0) 
  //    return 1;
  
  if(Flag_Uart_Recv > RX_BUFSIZE)  //收到串口数据
    return false;
  
  memcpy(pBuf, RxBuffer, lenght);
  Flag_Uart_Recv = 0;
  
  //是否是启动定时发送命令
  if(lenght == 6)
  {
    if((pBuf[0]==0x4c) && (pBuf[1]==0x54))
      Begin_AI_Send = 1;
    
    memset(RxBuffer, 0, lenght);
    return true;
  }   
  
  //常规数据报文
  if(!HEADCHECK(RxBuffer[0], RxBuffer[1]))
    return false;
  
  if(!check_CRC(pBuf, lenght))
    return false;    
  
  //tyh:20150629 处理接收到的数据
  switch(pBuf[2])
  {
  case DO_EXE:  //遥控
    result = DoExecute(pBuf[3]);
    break;    
    
  case AI_COF:  //通道系数校准
    result = AiCoefProcess(pBuf[3]);
    break;
    
  case AI_CALB: //通全道系数自动校准
    result = AiCalibration();
    break;
    
  case AI_TIME: //对时
    result = TimeCalibration(&(pBuf[4]));
    break;    
    
  default:
    result = false;
    break;	          
  }
  
/* //tyh:后续需修改执行过程  hack 
  if(result)  //执行成功，回复'确认'
  {
    
  }
  else  //执行不成功，回复'否认'
  {
    
  }
*/  
  memset(RxBuffer, 0, lenght);
  
  return result;
}



//////////////////////////////
bool check_CRC(uint8_t* pBuf, uint16_t lenght)
{
  uint16_t u32_len, CRCValue, crc;
  
  if(lenght > RX_BUFSIZE)
    return false;
  
  u32_len = (lenght-2)/4; //转换为u32的长度
  
  memcpy(&CRCValue, pBuf+lenght-2, 2);
  
  CRC_ResetDR();
  crc = CRC_CalcBlockCRC((uint32_t *)pBuf, u32_len);
  if(CRCValue != crc)
    return false;
  
  return true;
}

//////////////////////////////
uint16_t set_CRC(uint8_t*pBuf, uint16_t lenght)
{
  uint16_t u32_len, CRCValue, u8_len;
  
  if((lenght%4) != 0)
  {
    u32_len = (lenght/4)+1;
    u8_len = u32_len*4;
  }
  else
  {
    u32_len = lenght/4;
    u8_len = lenght;
  }
  
  if(u8_len > (RX_BUFSIZE-2))
    return 0;
  
  CRC_ResetDR();
  CRCValue = CRC_CalcBlockCRC((uint32_t *)pBuf, u32_len);

  memcpy(pBuf+u8_len, &CRCValue, 2);
  u8_len += 2;
  
  return u8_len;
}


void I2CHW_Maintain(void)
{
//  __IO uint32_t Timeout;
//  __IO uint32_t temp = 0;
//  uint16_t counter;
//  
//  counter = GetBusCount();
//  
//  temp=I2C1->SR2;
//  if(temp&0x0002) //总线BUSY
//  {
//    counter++;
//    SetBusCount(counter);
//  }
//  else
//  {
//    if(counter != 0)
//    {
//      counter = 0;
//      SetErrorCount(0);
//    }
//    GPIO_WriteBit(I2C_RESET_LED,  Bit_SET); 
//  }
//  
//  //tyh:20130730 i2c在指定次数内没有复位成功, 重启AI板
//  if( GetErrorCount() >= I2C_BUS_ERROR_MAX_COUNT )
//  {
//    GPIO_WriteBit(ALARM_LED,  Bit_RESET);
//    Timeout=0x7fff;
//    while(Timeout--); 
//    
//    MeaTab[MEANUM-1]++; //记录当前复位次数
//    DataBase_Write(BACKUPS_ROMADDR, (u32*)(&MeaTab[MEANUM-1]), sizeof(float)); //将复位次数存入flash
//    
//    NVIC_SystemReset();
//  }
//  
//  if(counter > 0x002b) //7s
//  {
//    I2C_Cmd(I2C1,DISABLE);
//    I2C_Cmd(I2C1,ENABLE);
//    I2CHW_Reset();
//    
//    GPIO_WriteBit(I2C_RESET_LED,  Bit_RESET); 
//    
//    //tyh:20130730 总线复位次数加"1"，
//    SetErrorCount( GetErrorCount()+1 );
//    
//    SetBusCount(0);
//  }
}

/*------------------------------End of File------------------------------------------*/
