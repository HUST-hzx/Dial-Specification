#include "main.h"
RCC_ClocksTypeDef rcc;



extern int SF_current;



int main(void)
{
  RCC_GetClocksFreq(&rcc);	
	System_Config();
	Parameter_Config();
  while(1)
	{
	}
}

void System_Config()
{
	SysTick_Config(168000);
	CAN_Config();
	USART_Config();
	TIM4_Config();
	FrictionWheel_Config();
	Putt_Init();
}



void Parameter_Config()
{
	FrictionWheel_Set(0);
	delay_ms(4000);
	SF_current=200;    //2205���21m/s(392)  22m/s(410) 27m/s(500) ����:16.4m/s(600) 19.5m/s(700) 27m/s(1050)
	Trigger_Init(26810);
	Pid_BigFrictSpeed();
}


void SysTick_Handler(void)//ba����Ħ����ת��
{
	Trigger_PID(0,0);
	Friction_Cal();
}

void delay_ms(int a)
{
	for(a*=10300;a>0;a--);
//	u32 temp;
//	SysTick->LOAD = 9000*ms;
//	SysTick->VAL = 0x00;  //��ռ�����
//	SysTick->CTRL = 0x01;  //ʹ�ܣ����������޶���
//	do
//	{
//		temp = SysTick->CTRL;  //��ȡ��ǰ������ֵ
//	}while((temp&0x01)&&(!(temp&(1<<16))));  //�ȴ�ʱ�䵽��
//	SysTick->CTRL = 0x00;  //�رռ�����
//	SysTick->VAL = 0x00;  //��ռ�����
}


