#ifndef __BODAN_H
#define __BODAN_H





typedef struct F105{
  short IsShootAble;
  short bulletSpeed_100;
  float Limit_Power_k;
	short RobotRed;
}F105_Typedef;




void Blocking_Detect(float BodanMotorCurrent);
float Trigger_PID(short circle, short speed);
void Trigger_Init(short a);




#endif
