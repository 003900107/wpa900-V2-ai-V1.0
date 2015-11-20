#ifndef BASEBUSPROTCOL
#define BASEBUSPROTCOL

#include <stdbool.h>

#include "soe.h"

#define DO_EXE  0xB3
#define DO_CHK  0xB2
#define DO_SEL  0xB1

#define AI_QRY  0xC1
#define AI_RES  0XC2
#define AI_SET  0XC4
#define AI_COF  0XC3
#define AI_DIO  0XC5
#define AI_CALB 0XC6
#define AI_TIME 0XCA    //对时
#define AI_COS  0XCB    //遥信变位


#define DMA	1
#define POLLING	2
#define INTERRUPT 3
#define I2C_METHOD DMA

#define MEMBLKCP  1
#define SINGLEBYTE  2
#define MEA_UPDATE_METHOD MEMBLKCP

#define CR1_PE_Set              ((uint16_t)0x0001)
#define CR1_PE_Reset            ((uint16_t)0xFFFE)
#define MEANUM      24
#define PTR_F2I(x) unsigned char*(x)

extern uint8_t Begin_AI_Send;



bool slave_write(void);
bool slave_read(void);
void i2c_buffer_read(unsigned char *pBuffer, unsigned char SlaveAddr);
void i2c_buffer_write(unsigned char *pBuffer, unsigned char SlaveAddr);
void I2CHW_Maintain(void);
void _I2C1_EV_IRQHandler(void);
/////////////////////////////////////
bool BusCalling_Process(uint16_t lenght);

void Deal_Readed(void);
bool AiCoefProcess(unsigned char AiSeq);
bool TimeCalibration(uint8_t *pos);
bool AiCalibration(void);

//tyh:20150803 增加硬件CRC校验函数
bool check_CRC(uint8_t* pBuf, uint16_t lenght);
uint16_t set_CRC(uint8_t*pBuf, uint16_t lenght);

void DiResponse(unsigned char AiSeq);
void AiResponse(unsigned char AiSeq);
void SoeResponse(const SOE_Struct SoeBuf);


#endif
