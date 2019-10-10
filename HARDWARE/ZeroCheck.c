/**********************************************************************************************************
 * @文件     ZeroCheck.c
 * @说明     过零检测
 * @版本  	 V1.0
 * @作者     hzx
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"



/**********************************************************************************************************
*函 数 名: ZeroCheak
*功能说明: 检测电机转过角度
*形    参: 检测结构体，当前角度
*返 回 值: 转过角度
**********************************************************************************************************/
float ZeroCheak(ZeroCheck_Typedef *Zero,float value)
{
	Zero->Actrual_Angle=value;
	if(Zero->Init_flag == 0)
	{
		Zero->First_Angle = Zero->Actrual_Angle;
		Zero->Init_flag++;
	}
	Zero->PreError=Zero->Actrual_Angle-Zero->Last_Angle;
	Zero->Last_Angle=Zero->Actrual_Angle;
	if(Zero->PreError<-0.5f*Zero->Count_Cycle)
	{
		Zero->PreError=Zero->Count_Cycle+Zero->PreError;
		Zero->Circle++;
	}
	if(Zero->PreError>0.5f*Zero->Count_Cycle)
	{
		Zero->PreError=Zero->PreError-Zero->Count_Cycle;
		Zero->Circle--;
	}
	Zero->SumAngle=Zero->Actrual_Angle+Zero->Circle*Zero->Count_Cycle-Zero->First_Angle;
		return Zero->SumAngle;
}











