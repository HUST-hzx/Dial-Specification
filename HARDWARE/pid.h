#ifndef __PID_H
#define __PID_H

#include "usart.h"
#include "can.h"

typedef struct PID{
		float SetPoint;			
		
		float P;						
		float I;						
		float D;						
		
		float LastError;		
		float PreError;			
		float SumError;			

		float IMax;					
		
		float POut;					
		float IOut;					
		float DOut;					
}PID;			

float PIDCalc(PID *P, float NextPoint);


#endif
