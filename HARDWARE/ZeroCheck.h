#ifndef __ZEROCHECK_H
#define __ZEROCHECK_H

#include "main.h"


typedef struct
{
	float Circle;//转过圈数
	float Count_Cycle;//计数周期
	float First_Angle;//初始角度
	short Init_flag;//记录初始角度标志位
	float Actrual_Angle;//当前角度测量值
	float Last_Angle;//上次角度测量值
	float PreError;//测量差值
	float SumAngle;
	float PreMicroPosition;
}ZeroCheck_Typedef;


typedef struct
{
	ZeroCheck_Typedef Zero;  //过零检测结构体
	MotorReceive_Typedef Receive;  //电机返回数据
	PID Pos;  //位置环
	PID Speed;  //速度环
}Motor_Typedef;



float ZeroCheak(ZeroCheck_Typedef *Zero,float value);



#endif
