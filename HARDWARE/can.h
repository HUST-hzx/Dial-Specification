#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "pid.h"
#include "trigger.h"

void CAN_Config(void);
void CAN_Set(int a,int b,int c,int d);
void trigger_F105_CurSend(short a);
void Can1Receive1(CanRxMsg rx_message1);
void CAN1_Receive0Task(CanRxMsg rx_message);




typedef struct{
	short Angle;
	short RealSpeed;       
}MotorReceive_Typedef;


#define PASS_ID        0x206


#endif

