#include "includes.h"
//stm32波特率最大为1Mbps
// CAN1初始化 用于M3508
// PA11 -> CANRX
// PA12 -> CANTX
void CAN1_Init(void)
{
	CAN_InitTypeDef        can; 
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;

	// 开时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	// 配置所用IO口
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

	gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &gpio);
	
	// 配置CAN的工作模式
	CAN_DeInit(CAN1);
	CAN_StructInit(&can);
	
	can.CAN_TTCM = DISABLE;//非时间触发通信模式  
	can.CAN_ABOM = DISABLE;//使用该功能可以在节点出错离线后适时的自动恢复，不需要软件干预。
	can.CAN_AWUM = DISABLE;//使用该功能可以在监测到总线活动后自动唤醒
	can.CAN_NART = ENABLE;//DISABLE代表的是使用自动重传的功能，ENABLE是代表不使用自动重传的功能 
	can.CAN_RFLM = DISABLE;//是否锁定FIFO,如果锁定，FIFO溢出会丢弃新数据；如果不锁定，FIFO溢出时，新数据会覆盖旧数据。
  can.CAN_TXFP = DISABLE;//使能时会以存入发送邮箱的顺序进行发送，失能时，以报文ID的优先级发送。
	can.CAN_Mode = CAN_Mode_Normal;//正常模式
	can.CAN_SJW  = CAN_SJW_1tq;//定义位段加长或缩短的上限，在1到4个时间片上调整
	can.CAN_BS1 = CAN_BS1_9tq;//定义采样点的位置，持续长度在1到16个时间片
	can.CAN_BS2 = CAN_BS2_4tq;//定义发送点的位置，持续长度在1到8个时间片
	can.CAN_Prescaler = 3;   //设置CAN波特率为 42/(1+9+4)/3=1Mbps（F4：42000）
	CAN_Init(CAN1, &can);//初始化can1

	// 配置筛选器
	can_filter.CAN_FilterNumber = 0;//过滤器0
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;//32位 
	can_filter.CAN_FilterIdHigh = 0x0000;//32位ID
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;//32位MASK
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	can_filter.CAN_FilterActivation=ENABLE;//激活过滤器0
	CAN_FilterInit(&can_filter);//滤波器初始化
	
	// 配置中断控制器
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;// 主优先级为1
	nvic.NVIC_IRQChannelSubPriority = 0;       // 次优先级为0
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	// 使能CAN中断
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);  // FIFO0接收中断
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);  // 发送中断
}

// CAN1的发送中断函数
void CAN1_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
		{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
     
    }
}

/*WAITING_TEST*/
// CAN1的FIFO0接受中断函数
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg CAN1_RX0_message;  // 临时存放接受数据的结构体
	
  if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
  {
		CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RX0_message);  // 读取数据
		m3508_update_m3508_info_can1(&CAN1_RX0_message);  // M3508电机数据处理
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
  }

}

//can2 初始化用于AK80
// PB12 -> CANRX
// PB13 -> CANTX

void CAN2_Init(void)
{
	CAN_InitTypeDef        can; 
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;

	// 开时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2 , ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  // 使用CAN2时必须开启CAN1时钟
    

	// 配置所用IO口
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &gpio);
	
	// 配置CAN的工作模式
	CAN_DeInit(CAN2);
	CAN_StructInit(&can);
	
	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = ENABLE;
	can.CAN_Mode = CAN_Mode_Normal;
	can.CAN_SJW  = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_9tq;
	can.CAN_BS2 = CAN_BS2_4tq;
	can.CAN_Prescaler = 3;   //设置CAN波特率为 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN2, &can);

	// 配置筛选器
	can_filter.CAN_FilterNumber = 14;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = 0x0000;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
	
	// 配置中断控制器
	nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 3;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	// 使能CAN中断
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);  // FIFO0接收中断
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);  // 发送中断
}


// CAN2的发送中断函数
void CAN2_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
		{
 	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
			
    }
}


/*WAITING_TEST*/
// CAN2的FIFO0接受中断函数
void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg CAN2_RX0_message;  // 临时存放接受数据的结构体
	
  if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
  {
		CAN_Receive(CAN2, CAN_FIFO0, &CAN2_RX0_message);  // 读取数据
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		
		AK80_update_info(&CAN2_RX0_message);//AK80接受
  }
	
}

