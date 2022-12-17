#include "includes.h"
#define UART3_RX_BUFFER_SIZE	256
uint8_t uart3RxBuffer[UART3_RX_BUFFER_SIZE];


/**
 * @brief       串口X初始化
 * @param       无
 * @retval      无
 */
void usart3_init(uint32_t bound)
{
	// 声明结构体
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);  // 使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  // 使能USART2时钟
 
	// 串口5对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  // PC10复用为USART2 TX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  // PC11复用为USART2 RX
	
	// UART5端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 // 速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

  // UART4初始化设置
	USART_InitStructure.USART_BaudRate = bound;                                  // 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                 // 收发模式
  USART_Init(USART3, &USART_InitStructure);
	
  USART_Cmd(USART3, ENABLE);  // 使能串口4
	
	// 配置中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;           // 串口4中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		     // 子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			       // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  // 开启相关中断


}

int laser_test_1;
int laser_test_2;
int USART_LASER_FLAG;
void USART3_IRQHandler(void)                	//串口2中断服务程序
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)   
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);  // 清除标志位
		if(laser_test_1>65000)
		{
			laser_test_1 = laser_test_1-65535;
		}
		if(laser_test_2>65000)
		{
			laser_test_2 = laser_test_2-65535;
		}
		laser_test_2=laser_test_2-5;
	}
}
