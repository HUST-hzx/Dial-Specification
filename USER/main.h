#ifndef __MAIN_H
#define __MAIN_H

#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))  


//Standard Lib
#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <stdio.h>
#include <stdlib.h>
#include "string.h"

//MyLib
#include "putt.h"
#include "can.h"
#include "usart.h"
#include "pid.h"
#include "frictionwheel.h"
#include "trigger.h"
#include "ZeroCheck.h"
#include "tim4.h"
#include "CRC_algorithm.h"


void System_Config(void);
void Parameter_Config(void);
void delay_ms(int a);


#endif
