#include "ACTION.h"

ACTION_GL_POS ACTION_GL_POS_DATA;
// 串口3初始化函数
void bsp_UART4_Init(u32 baud_rate)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);   //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);  //使能USART4时钟
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);  //GPIOC10复用为USART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);  //GPIOC11复用为USART4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  //GPIOC11和GPIOC10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  						//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  					// 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  						// 上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  		// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;  				// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	 // 收发模式
	USART_Init(UART4, &USART_InitStructure);  // 初始化串口4
	
	USART_Cmd(UART4, ENABLE);  //使能串口 
    
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  
	NVIC_Init(&NVIC_InitStructure);	

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);  // 开启中断

}

//串口4中断函数
void UART4_IRQHandler(void)
{
	static uint8_t ch;
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;

	static uint8_t count = 0;
	static uint8_t i = 0;

	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		ch = USART_ReceiveData(UART4);

		switch (count)
		{
			case 0:
			{
				if (ch == 0x0d)
					count++;
				else
					count = 0;
			}
			break;
			case 1:
			{
				if (ch == 0x0a)
				{
					i = 0;
					count++;
				}
				else if (ch == 0x0d);
				else
					count = 0;
			}
			break;
			case 2:
			{
				posture.data[i] = ch;
				i++;
				if (i >= 24)
				{
					i = 0;
					count++;
				}
			}
			break;
			case 3:
			{
				if (ch == 0x0a)
					count++;
				else
					count = 0;
			}
			break;
			case 4:
			{
				if (ch == 0x0d)
				{
					//更新传感器数据
					//不直接跟新，进行差分运算
					
					
					
					
					
					
					Update_Action(posture.ActVal);
				}
				count = 0;
			}
			break;
			default:
				count = 0;
				break;
		}
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
}



//更新action全场定位的值
//不直接跟新
void Update_Action(float value[6])
{
	float error;
//储存上一次的值
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
// 记录这次的值
	ACTION_GL_POS_DATA.ANGLE_Z = value[0];  // 有用
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X   = value[3];  // 有用
	ACTION_GL_POS_DATA.POS_Y   = value[4];  // 有用
	ACTION_GL_POS_DATA.W_Z     = value[5];
	
	// 差分运算
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
	
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
	

	// 偏航角直接赋值（逆时针为正，顺时针为负）
  ROBOT_REAL_POS_DATA.POS_YAW = ACTION_GL_POS_DATA.ANGLE_Z - OFFSET_YAW;
	
	//消除机械误差,赋值X、Y
	ROBOT_REAL_POS_DATA.POS_X = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
	ROBOT_REAL_POS_DATA.POS_Y = ACTION_GL_POS_DATA.REAL_Y - INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
	 
}

/*
void Update_Action_gl_position(float value[6])
{
	//储存上一次的值
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

	//记录这次的值
	ACTION_GL_POS_DATA.ANGLE_Z = value[0]; // 有用
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X = value[3]; // 有用x
	ACTION_GL_POS_DATA.POS_Y = value[4]; // 有用y
	ACTION_GL_POS_DATA.W_Z = value[5];

	// 差分运算
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

	//???????
	Robot_Chassis.Angle = -ACTION_GL_POS_DATA.ANGLE_Z;

	//?????????
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);//??????????
	
			
	//??????
//	Robot_Chassis.Position[x] =  ACTION_GL_POS_DATA.REAL_X; 
//	Robot_Chassis.Position[y] =  ACTION_GL_POS_DATA.REAL_Y;
		
	//???????
//	Robot_Chassis.Position[x] =  ACTION_GL_POS_DATA.REAL_X - L*sin(Robot_Chassis.Angle * PI / 180); 
//	Robot_Chassis.Position[y] = (ACTION_GL_POS_DATA.REAL_Y + L*cos(Robot_Chassis.Angle * PI / 180) );
}

*/





void UART_SendString(USART_TypeDef* USARTx, char *DataString)
{
	int i = 0;
	USART_ClearFlag(USARTx,USART_FLAG_TC);										//发送字符前清空标志位（否则缺失字符串的第一个字符）
	while(DataString[i] != '\0')												//字符串结束符
	{
		USART_SendData(USARTx,DataString[i]);									//每次发送字符串的一个字符
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC) == 0);					//等待数据发送成功
		USART_ClearFlag(USARTx,USART_FLAG_TC);									//发送字符后清空标志位
		i++;
	}
}

