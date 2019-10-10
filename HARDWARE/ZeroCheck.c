/**********************************************************************************************************
 * @�ļ�     ZeroCheck.c
 * @˵��     ������
 * @�汾  	 V1.0
 * @����     hzx
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"



/**********************************************************************************************************
*�� �� ��: ZeroCheak
*����˵��: �����ת���Ƕ�
*��    ��: ���ṹ�壬��ǰ�Ƕ�
*�� �� ֵ: ת���Ƕ�
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











