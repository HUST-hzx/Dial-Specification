/**********************************************************************************************************
 * @文件     trigger.c
 * @说明     拨盘电机
 * @版本  	 V1.0
 * @作者     hzx
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"
/*----------------------------------内部变量---------------------------*/

/*----------------------------------结构体-----------------------------*/
Motor_Typedef Trigger;//拨弹电机结构体
F105_Typedef idata;
/*----------------------------------外部变量---------------------------*/


/**********************************************************************************************************
*函 数 名: Trigger_PID
*功能说明: 拨弹电机双环控制
*形    参: circle不为零时：电机旋转circle圈
	         speed不为零时：电机以speed的速度旋转(双环控制)
					 不能同时不为零
*返 回 值: 发送电流值
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
*函 数 名: Blocking_Detect
*功能说明: 堵转检测
*形    参: 当前电流值
*返 回 值: 无
**********************************************************************************************************/
void Blocking_Detect(float BodanMotorCurrent)
{
	static short blocking_prevent_flag;//堵转时间检测
	static short reverse_flag;//倒转
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
*函 数 名: Trigger_Init
*功能说明: 拨弹电机结构体参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Trigger_Init(short a)
{
	Trigger.Zero.PreMicroPosition = a;//转一格电机转过位置  步兵：26810(8192*36/11)
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



