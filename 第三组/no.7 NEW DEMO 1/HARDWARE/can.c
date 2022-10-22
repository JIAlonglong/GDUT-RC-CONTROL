#include "includes.h"
//stm32���������Ϊ1Mbps
// CAN1��ʼ�� ����M3508
// PA11 -> CANRX
// PA12 -> CANTX
void CAN1_Init(void)
{
	CAN_InitTypeDef        can; 
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;

	// ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	// ��������IO��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

	gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &gpio);
	
	// ����CAN�Ĺ���ģʽ
	CAN_DeInit(CAN1);
	CAN_StructInit(&can);
	
	can.CAN_TTCM = DISABLE;//��ʱ�䴥��ͨ��ģʽ  
	can.CAN_ABOM = DISABLE;//ʹ�øù��ܿ����ڽڵ�������ߺ���ʱ���Զ��ָ�������Ҫ�����Ԥ��
	can.CAN_AWUM = DISABLE;//ʹ�øù��ܿ����ڼ�⵽���߻���Զ�����
	can.CAN_NART = ENABLE;//DISABLE�������ʹ���Զ��ش��Ĺ��ܣ�ENABLE�Ǵ���ʹ���Զ��ش��Ĺ��� 
	can.CAN_RFLM = DISABLE;//�Ƿ�����FIFO,���������FIFO����ᶪ�������ݣ������������FIFO���ʱ�������ݻḲ�Ǿ����ݡ�
  can.CAN_TXFP = DISABLE;//ʹ��ʱ���Դ��뷢�������˳����з��ͣ�ʧ��ʱ���Ա���ID�����ȼ����͡�
	can.CAN_Mode = CAN_Mode_Normal;//����ģʽ
	can.CAN_SJW  = CAN_SJW_1tq;//����λ�μӳ������̵����ޣ���1��4��ʱ��Ƭ�ϵ���
	can.CAN_BS1 = CAN_BS1_9tq;//����������λ�ã�����������1��16��ʱ��Ƭ
	can.CAN_BS2 = CAN_BS2_4tq;//���巢�͵��λ�ã�����������1��8��ʱ��Ƭ
	can.CAN_Prescaler = 3;   //����CAN������Ϊ 42/(1+9+4)/3=1Mbps��F4��42000��
	CAN_Init(CAN1, &can);//��ʼ��can1

	// ����ɸѡ��
	can_filter.CAN_FilterNumber = 0;//������0
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;//32λ 
	can_filter.CAN_FilterIdHigh = 0x0000;//32λID
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;//32λMASK
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//������0������FIFO0
	can_filter.CAN_FilterActivation=ENABLE;//���������0
	CAN_FilterInit(&can_filter);//�˲�����ʼ��
	
	// �����жϿ�����
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;// �����ȼ�Ϊ1
	nvic.NVIC_IRQChannelSubPriority = 0;       // �����ȼ�Ϊ0
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	// ʹ��CAN�ж�
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);  // FIFO0�����ж�
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);  // �����ж�
}

// CAN1�ķ����жϺ���
void CAN1_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
		{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
     
    }
}

/*WAITING_TEST*/
// CAN1��FIFO0�����жϺ���
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg CAN1_RX0_message;  // ��ʱ��Ž������ݵĽṹ��
	
  if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
  {
		CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RX0_message);  // ��ȡ����
		m3508_update_m3508_info_can1(&CAN1_RX0_message);  // M3508������ݴ���
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
  }

}

//can2 ��ʼ������AK80
// PB12 -> CANRX
// PB13 -> CANTX

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

