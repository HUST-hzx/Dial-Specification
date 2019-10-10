#ifndef __FRICTIONWHEEL_H
#define __FRICTIONWHEEL_H

#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"

void FrictionWheel_Config(void);
void FrictionWheel_Set(short speedL);
void Friction_Cal(void);
void Friction_Set(void);
void BigFriction_Set(void);
void Pid_BigFrictSpeed(void);


typedef struct{
	short Current;
	short Angle;
	short RealSpeed;       
}M3508Receive_Typedef;

#endif
