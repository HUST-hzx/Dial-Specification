/**********************************************************************************************************
 * @文件     putt.c
 * @说明     电磁阀
 * @版本  	 V1.0
 * @作者     hzx
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"

/**********************************************************************************************************
*函 数 名: Putt_Init
*功能说明: 电磁阀IO口初始化
*形    参: 无
*返 回 值: 无
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

