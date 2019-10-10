#ifndef __ZEROCHECK_H
#define __ZEROCHECK_H

#include "main.h"


typedef struct
{
	float Circle;//ת��Ȧ��
	float Count_Cycle;//��������
	float First_Angle;//��ʼ�Ƕ�
	short Init_flag;//��¼��ʼ�Ƕȱ�־λ
	float Actrual_Angle;//��ǰ�ǶȲ���ֵ
	float Last_Angle;//�ϴνǶȲ���ֵ
	float PreError;//������ֵ
	float SumAngle;
	float PreMicroPosition;
}ZeroCheck_Typedef;


typedef struct
{
	ZeroCheck_Typedef Zero;  //������ṹ��
	MotorReceive_Typedef Receive;  //�����������
	PID Pos;  //λ�û�
	PID Speed;  //�ٶȻ�
}Motor_Typedef;



float ZeroCheak(ZeroCheck_Typedef *Zero,float value);



#endif
