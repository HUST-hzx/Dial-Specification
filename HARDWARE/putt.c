/**********************************************************************************************************
 * @�ļ�     putt.c
 * @˵��     ��ŷ�
 * @�汾  	 V1.0
 * @����     hzx
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"

/**********************************************************************************************************
*�� �� ��: Putt_Init
*����˵��: ��ŷ�IO�ڳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Putt_Init()
{
	GPIO_InitTypeDef gpio;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	gpio.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	gpio.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);	
}

