#ifndef __USART_H
#define __USART_H

#define GYRO_BUF_SIZE 18
#define JudgeBufBiggestSize 24

#include "stm32f4xx.h"

void USART_Config(void);
void USART1_Configuration(void);
void RC_Rst(void);
void RemoteReceive(volatile unsigned char rx_buffer[]);
void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

/*遥控器结构体*/
typedef __packed struct
{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned short s1;
		unsigned short s2;
}Remote;
/*鼠标结构体*/
typedef __packed 	struct
{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
}Mouse;
/*键盘结构体*/
typedef __packed struct
{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
  char RCrecvd,RCDisconnectCnt;//RCrecvd为数据接收标志位
}RC_Ctl_t;

typedef struct
{
	char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;
	
	
	int FiredBulletNum;
	uint8_t RobotLevel;
	uint8_t bulletFreq;
	uint8_t cardType;
	uint8_t CardIdx;
	uint16_t remainHP;
	uint16_t maxHP;

	short  HeatTop;
	uint16_t shooterHeat42;
	
	uint16_t LastShooterHeat17;
	short shooterHeat17;
	float bulletSpeed;
	float LastbulletSpeed;
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       //剩余能量
	float bulletSpeed17;
	float bulletSpeed42;
	float LastbulletSpeed17;
}
tGameInfo;



#define RX_UART5_BUFFER 30
#define JudgeBufBiggestSize 24

#endif

