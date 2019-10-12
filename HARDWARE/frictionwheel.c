/**********************************************************************************************************
 * @文件     frictionwheel.c
 * @说明     摩擦轮配置
 * @版本  	 V1.0
 * @作者     hzx
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"
uint32_t SRC_Buffer[]={12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23,12,23};


/**********************************************************************************************************
*函 数 名: FrictionWheel_Config
*功能说明: 摩擦轮相关初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void FrictionWheel_Config()//TIM8 PC8 PC9
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);
	
	TIM_TimeBaseInitStruct.TIM_Prescaler=8-1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=35-1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse=12;
	TIM_OCInitStruct.TIM_OCIdleState=TIM_OCIdleState_Set;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC3Init(TIM8,&TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM8,ENABLE);
	TIM_OC4Init(TIM8,&TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM8,ENABLE);
	
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel =DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(TIM8->DMAR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SRC_Buffer;	
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;	
	DMA_InitStructure.DMA_BufferSize = 32;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	
	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	TIM_DMAConfig(TIM8,TIM_DMABase_CCR3,TIM_DMABurstLength_2Transfers);
	TIM_DMACmd(TIM8, TIM_DMA_Update, ENABLE);
	TIM_Cmd(TIM8,ENABLE);
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);
}


void DMA2_Stream1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream1,DMA_IT_TCIF1)==SET)
	{
		TIM_DMACmd(TIM8, TIM_DMA_Update, DISABLE);
		DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,DISABLE);
		DMA_Cmd(DMA2_Stream1, DISABLE);
		TIM_SetCompare3(TIM8,0);
		TIM_SetCompare4(TIM8,0);
		TIM_Cmd(TIM4, ENABLE);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
		DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TCIF1);
	}
}




extern RC_Ctl_t RC_Ctl; 
short SF_current=0;//小摩擦轮电流值
short time1;
short BF_current=2000;//大摩擦轮电流值
short	time3;

/**********************************************************************************************************
*函 数 名: Friction_Cal
*功能说明: 发射机构执行函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Friction_Cal(void)
{
	if(RC_Ctl.rc.s1 == 1)
	{
		BF_current = 0;
		Friction_Set();
		trigger_F105_CurSend(0);
	}
	else if(RC_Ctl.rc.s1 == 2)
	{
		SF_current=0;
		BigFriction_Set();
		FrictionWheel_Set(0);
	}
	if(RC_Ctl.rc.s2 == 1)
	{
//		GPIO_SetBits(GPIOC, GPIO_Pin_4);
//		GPIO_SetBits(GPIOC, GPIO_Pin_5);
		GPIO_SetBits(GPIOA, GPIO_Pin_6);
//		GPIO_SetBits(GPIOA, GPIO_Pin_7);
	}
	else if(RC_Ctl.rc.s2 == 2)
	{
//		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
//		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
		GPIO_ResetBits(GPIOA, GPIO_Pin_6);
//		GPIO_ResetBits(GPIOA, GPIO_Pin_7);
	}
}




extern uint16_t accelerator;
/**********************************************************************************************************
*函 数 名: FrictionWheel_Set
*功能说明: 小摩擦轮转速设置
*形    参: 摩擦轮转速
*返 回 值: 无
**********************************************************************************************************/
void FrictionWheel_Set(short speed)
{
	accelerator=LIMIT_MAX_MIN(speed,2048,0);
}






/**********************************************************************************************************
*函 数 名: Friction_Set
*功能说明: 通过遥控器改变小摩擦轮转速
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Friction_Set(void)
{
	if(time1>1000)
	{
		if(RC_Ctl.rc.ch1 > 1500)
		  SF_current += 100;
	  else if(RC_Ctl.rc.ch1 < 500)
			SF_current -= 100;
		time1 = 0;
	}
	SF_current = LIMIT_MAX_MIN(SF_current, 2000, 0);
	time1++;
	FrictionWheel_Set(SF_current);
}


M3508Receive_Typedef BigFrict[2];
PID PidBigFrictSpeed[2];
/**********************************************************************************************************
*函 数 名: BigFriction_Set
*功能说明: 通过遥控器改变大摩擦轮转速
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BigFriction_Set(void)
{
	short BigFrictCur[2];
	
	if(time3>1000)
	{
		if(RC_Ctl.rc.ch1 > 1500)
		  BF_current += 1000;
	  else if(RC_Ctl.rc.ch1 < 500)
			BF_current -= 1000;
		time3 = 0;
	}
	BF_current = LIMIT_MAX_MIN(BF_current, 16000, 0);
	time3++;
	
	PidBigFrictSpeed[0].SetPoint = BF_current;
	PidBigFrictSpeed[1].SetPoint = -BF_current;
	
	BigFrictCur[0] = (short)LIMIT_MAX_MIN(PIDCalc(&PidBigFrictSpeed[0], BigFrict[0].RealSpeed),16000,-16000);
	BigFrictCur[1] = (short)LIMIT_MAX_MIN(PIDCalc(&PidBigFrictSpeed[1], BigFrict[1].RealSpeed),16000,-16000);
	
	CAN_Set(BigFrictCur[1], BigFrictCur[0], 0, 0);
}


/**********************************************************************************************************
*函 数 名: Pid_BigFrictSpeed
*功能说明: 大摩擦轮PID参数配置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_BigFrictSpeed(void)
{
	PidBigFrictSpeed[0].SetPoint = 0.0f;
	PidBigFrictSpeed[0].P = 25;		
	PidBigFrictSpeed[0].I = 0;
	PidBigFrictSpeed[0].D	= 0;
	PidBigFrictSpeed[0].IMax = 0;
	
	PidBigFrictSpeed[1].SetPoint = 0.0f;
	PidBigFrictSpeed[1].P = 25;		
	PidBigFrictSpeed[1].I = 0;
	PidBigFrictSpeed[1].D	= 0;
	PidBigFrictSpeed[1].IMax = 0;
}


