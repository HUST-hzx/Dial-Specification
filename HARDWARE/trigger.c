/**********************************************************************************************************
 * @�ļ�     trigger.c
 * @˵��     ���̵��
 * @�汾  	 V1.0
 * @����     hzx
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/

/*----------------------------------�ṹ��-----------------------------*/
Motor_Typedef Trigger;//��������ṹ��
F105_Typedef idata;
/*----------------------------------�ⲿ����---------------------------*/


/**********************************************************************************************************
*�� �� ��: Trigger_PID
*����˵��: �������˫������
*��    ��: circle��Ϊ��ʱ�������תcircleȦ
	         speed��Ϊ��ʱ�������speed���ٶ���ת(˫������)
					 ����ͬʱ��Ϊ��
*�� �� ֵ: ���͵���ֵ
**********************************************************************************************************/
float Trigger_PID(short circle, short speed)
{
	static float BodanMotorCurrent;
	
	Blocking_Detect(BodanMotorCurrent);
	
	if(circle != 0)
	{
		Trigger.Pos.SetPoint += circle*Trigger.Zero.Count_Cycle;
	}
	if(speed != 0)
	{
		Trigger.Pos.SetPoint += speed;
	}

	Trigger.Speed.SetPoint = PIDCalc(&Trigger.Pos, ZeroCheak(&Trigger.Zero, Trigger.Receive.Angle)); 
	BodanMotorCurrent = PIDCalc(&Trigger.Speed, Trigger.Receive.RealSpeed);
	BodanMotorCurrent = (short)LIMIT_MAX_MIN(BodanMotorCurrent, 7000.0f, -7000.0f);
	
	return BodanMotorCurrent;
}



/**********************************************************************************************************
*�� �� ��: Blocking_Detect
*����˵��: ��ת���
*��    ��: ��ǰ����ֵ
*�� �� ֵ: ��
**********************************************************************************************************/
void Blocking_Detect(float BodanMotorCurrent)
{
	static short blocking_prevent_flag;//��תʱ����
	static short reverse_flag;//��ת
	static short MicroPosition;
	if(ABS(BodanMotorCurrent) > 700 && ABS(Trigger.Receive.RealSpeed) == 0)
	{
		blocking_prevent_flag++;
		if(ABS(BodanMotorCurrent) > 700 && ABS(Trigger.Receive.RealSpeed) == 0 && blocking_prevent_flag >50)
		{
			MicroPosition = -0.3*Trigger.Zero.PreMicroPosition;
		  Trigger.Pos.SetPoint += MicroPosition;
			blocking_prevent_flag = 0;
			reverse_flag = 70;
		}
	}
	else
	{
		MicroPosition = Trigger.Zero.PreMicroPosition;
		blocking_prevent_flag = 0;
	}
	
	  if(reverse_flag > 0)
	{
		Trigger.Pos.SetPoint -= Trigger.Zero.PreMicroPosition/8;
		reverse_flag--;
	}
}


/**********************************************************************************************************
*�� �� ��: Trigger_Init
*����˵��: ��������ṹ�������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Trigger_Init(short a)
{
	Trigger.Zero.PreMicroPosition = a;//תһ����ת��λ��  ������26810(8192*36/11)
	Trigger.Zero.Circle=0;
	Trigger.Zero.First_Angle=0;
	Trigger.Zero.Init_flag=0;
	Trigger.Zero.PreError=0;
	Trigger.Zero.Actrual_Angle=0;
	Trigger.Zero.Last_Angle=0;
	Trigger.Zero.SumAngle=0;
	Trigger.Zero.Count_Cycle=8190;
	
	Trigger.Pos.P=8.0f;
	Trigger.Pos.I=0.2f;
	Trigger.Pos.D=1.5f;
	Trigger.Pos.IMax=1500.0f;
	Trigger.Pos.SetPoint=400.0f;
	
	Trigger.Speed.P=0.2f;
	Trigger.Speed.I=0.05f;
	Trigger.Speed.D=0.0f;
	Trigger.Speed.IMax=500.0f;
	Trigger.Speed.SetPoint=38000.0f;
}



