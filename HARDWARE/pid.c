/**********************************************************************************************************
 * @文件     pid.c
 * @说明     PID计算
 * @版本  	 V1.0
 * @作者     hzx
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"


/**********************************************************************************************************
*函 数 名: PIDCalc
*功能说明: PID计算
*形    参: 无
*返 回 值: 无
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





