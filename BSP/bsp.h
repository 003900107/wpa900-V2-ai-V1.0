#ifndef __BSP_H
#define __BSP_H
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "adc_calc.h"
#include "BaseBus_Protocol.h"
#include "Realtimedb.h"						

/* define ------------------------------------------------------------*/
// ADDR
#define ADDR_IO_GPIO  GPIOE
#define ADDR_IO_PIN   GPIO_Pin_5

// SEND CONTROL
#define CONTROL_IO_GPIO  GPIOB
#define CONTROL_IO_PIN1  GPIO_Pin_13
#define CONTROL_IO_PIN2  GPIO_Pin_12

// DO
#define DO_NUM  4
#define DO0  GPIOE, GPIO_Pin_13
#define DO1  GPIOE, GPIO_Pin_12
#define DO2  GPIOE, GPIO_Pin_11
#define DO3  GPIOE, GPIO_Pin_10
#define DO_TMR_INTERVAL 15    //    实际DO脉宽为150ms

// DI
#define DI_NUM  16
#define DI0   GPIOD, GPIO_Pin_7
#define DI1   GPIOD, GPIO_Pin_6
#define DI2   GPIOD, GPIO_Pin_5
#define DI3   GPIOD, GPIO_Pin_4
#define DI4   GPIOD, GPIO_Pin_3
#define DI5   GPIOD, GPIO_Pin_2
#define DI6   GPIOD, GPIO_Pin_1
#define DI7   GPIOD, GPIO_Pin_0
#define DI8   GPIOD, GPIO_Pin_15
#define DI9   GPIOD, GPIO_Pin_14
#define DI10  GPIOD, GPIO_Pin_13
#define DI11  GPIOD, GPIO_Pin_12
#define DI12  GPIOD, GPIO_Pin_11
#define DI13  GPIOD, GPIO_Pin_10
#define DI14  GPIOD, GPIO_Pin_9
#define DI15  GPIOD, GPIO_Pin_8

#define TX_BUFSIZE      256
#define RX_BUFSIZE      300

/* typedef -----------------------------------------------------------*/
//DI状态采集
typedef struct
{
  uint8_t Actcnt;
  uint8_t Retcnt;
  uint8_t Value; 
  //uint8_t SOE_Queue[16];	  
  
}DiStatus_Type;

//继电器出口
typedef struct TagDoStruct
{
  __IO uint8_t status;
  __IO uint32_t DoTimer;
} DoStructure ;

//UART 收发缓冲区
extern uint8_t TxBuffer[TX_BUFSIZE];
extern uint8_t RxBuffer[RX_BUFSIZE];

extern uint16_t Flag_Uart_Send;
extern uint16_t Flag_Uart_Recv;

extern uint8_t UartAddr;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void); 
void ADC_Configuration(void);
void TIM2_Configuration(void);
void I2C_Configuration(void);
#ifdef WATCHDOG
void IWDG_Configuration(void);
void WDGFeeding(void);
#endif
void FREQ_MEA_Init ( void );
void SysTick_Configuration(void);
void Delay(u32 nTime);
void Time_Update(void);
void I2CHW_Reset(void);

////////////////////////////////
uint8_t GetComAddr();     //20151103 添加com地址判断
void COM3_Configuration(uint32_t BaudRate);
void TIM_UART_Config();

void GPIO_DealInputData(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint8_t pos);
void DiScanning();    //DI输入检测

void Di_PostWork(void);   //非电量跳闸判断

bool DoExecute(unsigned char DoSeq);
void DO_Process(void);
void RelayClose(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void RelayOpen(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

bool SendUartData(uint16_t lenght);
bool GetSendSign();

#endif

