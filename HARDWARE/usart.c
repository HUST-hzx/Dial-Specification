/**********************************************************************************************************
 * @文件     usart.c
 * @说明     USART初始化
 * @版本  	 V1.0
 * @作者     hzx
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"

volatile unsigned char sbus_rx_buffer[18];
unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
unsigned char JudgeSend[22];
tGameInfo JudgeReceive;
short BulletFre,BulletSpeed;
RC_Ctl_t RC_Ctl; 


float jilu[400];
/**********************************************************************************************************
*函 数 名: USART_Config
*功能说明: USART初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void USART_Config()
{
	{//USART1  遥控器接收
    USART_InitTypeDef usart1;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 
	
		gpio.GPIO_Pin = GPIO_Pin_10;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA,&gpio);

		usart1.USART_BaudRate = 100000;
		usart1.USART_WordLength = USART_WordLength_8b;
		usart1.USART_StopBits = USART_StopBits_1;
		usart1.USART_Parity = USART_Parity_Even;
		usart1.USART_Mode = USART_Mode_Rx;
    usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART1,&usart1);

    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
		USART_Cmd(USART1,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);	

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		{
			DMA_InitTypeDef   dma;
 			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
			DMA_DeInit(DMA2_Stream5);
			dma.DMA_Channel= DMA_Channel_4;
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
			dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
			dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
			dma.DMA_BufferSize = 30;
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Circular;
			dma.DMA_Priority = DMA_Priority_VeryHigh;
			dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
			dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
			dma.DMA_MemoryBurst = DMA_Mode_Normal;
			dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA2_Stream5,&dma);
			DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
			DMA_Cmd(DMA2_Stream5,ENABLE);
		}
	}

  {  //UART4 电脑通信
	  USART_InitTypeDef uart4;
		GPIO_InitTypeDef  gpio;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
    gpio.GPIO_Pin = GPIO_Pin_10;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC,&gpio);
		
		gpio.GPIO_Pin = GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC,&gpio);

		uart4.USART_BaudRate = 115200;
		uart4.USART_WordLength = USART_WordLength_8b;
		uart4.USART_StopBits = USART_StopBits_1;
		uart4.USART_Parity = USART_Parity_No;
		uart4.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(UART4,&uart4);
		
		USART_Cmd(UART4,ENABLE);
	}
	
	{//USART6 裁判系统接收
	  USART_InitTypeDef usart6;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 
	
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC,&gpio);

		usart6.USART_BaudRate = 115200;
		usart6.USART_WordLength = USART_WordLength_8b;
		usart6.USART_StopBits = USART_StopBits_1;
		usart6.USART_Parity = USART_Parity_No;
		usart6.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART6,&usart6);
		
		USART_Cmd(USART6,ENABLE);
    USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);	
		USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
		
    nvic.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;//1
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		{
			DMA_InitTypeDef   dma;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
			DMA_DeInit(DMA2_Stream2);
			dma.DMA_Channel= DMA_Channel_5;
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
			dma.DMA_Memory0BaseAddr = (uint32_t)JudgeReceiveBuffer;
			dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
			dma.DMA_BufferSize = JudgeBufBiggestSize;
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Circular;
			dma.DMA_Priority = DMA_Priority_VeryHigh;
			dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
			dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
			dma.DMA_MemoryBurst = DMA_Mode_Normal;
			dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA2_Stream2,&dma);
			DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
			DMA_Cmd(DMA2_Stream2,ENABLE);
		}
	}
}

/**********************************************************************************************************
*函 数 名: fputc  fgetc
*功能说明: 重定向
*形    参: 
*返 回 值: 
**********************************************************************************************************/
int fputc(int ch,FILE *f)
{
	USART_SendData(USART6,(uint8_t)ch);
	while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
	return (ch);
}

int fgetc(FILE *f)
{
	while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
	return (int)USART_ReceiveData(USART6);
}


/**
  * @brief  串口1 遥控器数据闲时中断
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	static int DATA_LENGTH=0;
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
    (void)USART1->SR;
		(void)USART1->DR;	
	  DMA_Cmd(DMA2_Stream5,DISABLE);
	  DATA_LENGTH=30-DMA_GetCurrDataCounter(DMA2_Stream5);
		 if(DATA_LENGTH==18)
		 {
				RemoteReceive(sbus_rx_buffer);//解码函数
		 }
		DMA_SetCurrDataCounter(DMA2_Stream5,30);	
		DMA_Cmd(DMA2_Stream5,ENABLE);
  }
}


/**********************************************************************************************************
*函 数 名: RC_Rst
*功能说明: 遥控器接收结构体重置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RC_Rst(void)
{
		RC_Ctl.rc.ch0 = 1024;
		RC_Ctl.rc.ch1 = 1024;
		RC_Ctl.rc.ch2 = 1024;
		RC_Ctl.rc.ch3 = 1024;
		RC_Ctl.mouse.x = 0;
		RC_Ctl.mouse.y = 0;
		RC_Ctl.mouse.z = 0;
		RC_Ctl.mouse.press_l = 0;                                                
		RC_Ctl.mouse.press_r = 0;
	
		RC_Ctl.key.w = 0;
		RC_Ctl.key.s = 0;                            
		RC_Ctl.key.a = 0;
		RC_Ctl.key.d = 0;
		RC_Ctl.key.q = 0;
		RC_Ctl.key.e = 0;
		RC_Ctl.key.r = 0;
		RC_Ctl.key.f = 0;
		RC_Ctl.key.shift = 0;
		RC_Ctl.key.ctrl = 0;
	
	  RC_Ctl.rc.s1 = 2;
		RC_Ctl.rc.s2 = 2;
}   


/**********************************************************************************************************
*函 数 名: RemoteReceive
*功能说明: 遥控器解码
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RemoteReceive(volatile unsigned char rx_buffer[])
{
	RC_Ctl.rc.ch0 = (rx_buffer[0]| (rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RC_Ctl.rc.ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	RC_Ctl.rc.ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1 = ((rx_buffer[5] >> 4)& 0x0003); //!< Switch left
	RC_Ctl.rc.s2 = ((rx_buffer[5] >> 6)& 0x0003);
	RC_Ctl.mouse.x = rx_buffer[6] | (rx_buffer[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = rx_buffer[8] | (rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = rx_buffer[10] | (rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.w = rx_buffer[14]&0x01; // KeyBoard value
	RC_Ctl.key.s = (rx_buffer[14]>>1)&0x01;
	RC_Ctl.key.a = (rx_buffer[14]>>2)&0x01;
	RC_Ctl.key.d = (rx_buffer[14]>>3)&0x01;
	RC_Ctl.key.shift =(rx_buffer[14]>>4)&0x01;
	RC_Ctl.key.ctrl = (rx_buffer[14]>>5)&0x01;
	RC_Ctl.key.q = (rx_buffer[14]>>6)&0x01;
	RC_Ctl.key.e = (rx_buffer[14]>>7)&0x01;	
	RC_Ctl.key.r = (rx_buffer[15])&0x01;
	RC_Ctl.key.f = (rx_buffer[15]>>1)&0x01;
	RC_Ctl.key.g = (rx_buffer[15]>>2)&0x01; 
	RC_Ctl.key.z = (rx_buffer[15]>>3)&0x01;
	RC_Ctl.key.x = (rx_buffer[15]>>4)&0x01;
	RC_Ctl.key.c = (rx_buffer[15]>>5)&0x01;
	RC_Ctl.key.v = (rx_buffer[15]>>6)&0x01;
	RC_Ctl.key.b = (rx_buffer[15]>>7)&0x01;
	RC_Ctl.RCrecvd = 1; //数据接收标志位
	RC_Ctl.RCDisconnectCnt = 0;    //断线数据清零	
	if((RC_Ctl.rc.ch0-1024<15)&&(RC_Ctl.rc.ch0-1024>-15)) RC_Ctl.rc.ch0=1024;
	if((RC_Ctl.rc.ch1-1024<15)&&(RC_Ctl.rc.ch1-1024>-15)) RC_Ctl.rc.ch1=1024;
	if((RC_Ctl.rc.ch2-1024<10)&&(RC_Ctl.rc.ch2-1024>-10)) RC_Ctl.rc.ch2=1024;
	if((RC_Ctl.rc.ch3-1024<10)&&(RC_Ctl.rc.ch3-1024>-10)) RC_Ctl.rc.ch3=1024;	
}


void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
	{
		JudgeBuffReceive(JudgeReceiveBuffer,0);
  }
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
}

//void DMA2_Stream7_IRQHandler(void)
//{
//	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
//	{
//  }
//	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
//	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
//}

unsigned char SaveBuffer[68];
int Bullet=0;
uint16_t heat[256];
/**********************************************************************************************************
*函 数 名: JudgeBuffReceive
*功能说明: 裁判系统解码
*形    参: 接收裁判系统数据  字长
*返 回 值: 无
**********************************************************************************************************/
void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen)
{
	uint16_t cmd_id;
	short PackPoint;
	memcpy(&SaveBuffer[JudgeBufBiggestSize],&ReceiveBuffer[0],JudgeBufBiggestSize);
	for(PackPoint=0;PackPoint<JudgeBufBiggestSize;PackPoint++)
	{
		if(SaveBuffer[PackPoint]==0xA5)  //新版 数据帧开头
		{	
			if((Verify_CRC8_Check_Sum(&SaveBuffer[PackPoint],5)==1))//  新版也是 CRC校验  5是CRC
			{
				cmd_id=(SaveBuffer[PackPoint+6])&0xff;
				cmd_id=(cmd_id<<8)|SaveBuffer[PackPoint+5];  //新版   6 7是ID号
				DataLen=SaveBuffer[PackPoint+2]&0xff;
				DataLen=(DataLen<<8)|SaveBuffer[PackPoint+1];//新版  2 3是长度
				if((cmd_id==0x0201)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))  //这儿为什么加上9
				{
					JudgeReceive.RobotLevel=SaveBuffer[PackPoint+8];//   9位是机器人等级
					JudgeReceive.remainHP=(SaveBuffer[PackPoint+10]<<8)|SaveBuffer[PackPoint+9]; // 10,11位机器人当前血量
				}
				if((cmd_id==0x0207)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					if(SaveBuffer[PackPoint+7+0]==1)
					{
						JudgeReceive.bulletFreq= SaveBuffer[PackPoint+7+1];
						memcpy(&JudgeReceive.bulletSpeed,&SaveBuffer[PackPoint+7+2],4);
						Bullet++;
						printf("%fm/s    %d 发/s   第 %d 发    \n", JudgeReceive.bulletSpeed, JudgeReceive.bulletFreq, Bullet);
					}
				}
				if((cmd_id==0x0202)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					memcpy(&JudgeReceive.realChassisOutV,&SaveBuffer[PackPoint+7+0],2);
					memcpy(&JudgeReceive.realChassisOutA,&SaveBuffer[PackPoint+7+2],2);
					memcpy(&JudgeReceive.realChassispower,&SaveBuffer[PackPoint+7+4],4);
					memcpy(&JudgeReceive.remainEnergy,&SaveBuffer[PackPoint+7+8],2);
					memcpy(&JudgeReceive.shooterHeat17,&SaveBuffer[PackPoint+7+10],2);
				}
			}
		}
	}
	memcpy(&SaveBuffer[0],&SaveBuffer[JudgeBufBiggestSize],JudgeBufBiggestSize);
}

