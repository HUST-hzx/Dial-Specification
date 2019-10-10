/**********************************************************************************************************
 * @�ļ�     pid.c
 * @˵��     PID����
 * @�汾  	 V1.0
 * @����     hzx
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"


/**********************************************************************************************************
*�� �� ��: PIDCalc
*����˵��: PID����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float PIDCalc(PID *P, float NextPoint)
{
		float dError,Error;
		Error = P->SetPoint - NextPoint;
		
		P->SumError += Error;
		dError = Error-P->LastError;
		P->PreError = P->LastError;
		P->LastError = Error;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P*Error;
		P->IOut = P->I*P->SumError;
		P->DOut = P->D*dError;
		
		return P->POut+P->IOut+P->DOut;
}





