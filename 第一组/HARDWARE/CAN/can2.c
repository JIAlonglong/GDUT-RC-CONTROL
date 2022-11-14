#include "can2.h"
// CAN2��ʼ��
// PB12 -> CAN2RX 
// PB13 -> CAN2TX
 void CAN2_Init(void)
{
	CAN_InitTypeDef        can; 
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;

	// ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2 , ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  // ʹ��CAN2ʱ���뿪��CAN1ʱ��
    

	// ��������IO��
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &gpio);
	
	// ����CAN�Ĺ���ģʽ
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
	can.CAN_Prescaler = 3;   //����CAN������Ϊ 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN2, &can);

	// ����ɸѡ��
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
	
	// �����жϿ�����
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
	
	// ʹ��CAN�ж�
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);  // FIFO0�����ж�
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);  // �����ж�
}


// CAN2�ķ����жϺ���
void CAN2_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
		{
 	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
			
    }
}

      
/*WAITING_TEST*/
// CAN2��FIFO0�����жϺ���
void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg CAN2_RX0_message;  // ��ʱ��Ž������ݵĽṹ��
	
  if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
  {
		CAN_Receive(CAN2, CAN_FIFO0, &CAN2_RX0_message);  // ��ȡ����
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		
		AK80_update_info(&CAN2_RX0_message);//AK80����
  }
	
}


