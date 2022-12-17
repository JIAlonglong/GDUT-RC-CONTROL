#include "includes.h"
#define UART3_RX_BUFFER_SIZE	256
uint8_t uart3RxBuffer[UART3_RX_BUFFER_SIZE];


/**
 * @brief       ����X��ʼ��
 * @param       ��
 * @retval      ��
 */
void usart3_init(uint32_t bound)
{
	// �����ṹ��
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);  // ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  // ʹ��USART2ʱ��
 
	// ����5��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);  // PC10����ΪUSART2 TX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);  // PC11����ΪUSART2 RX
	
	// UART5�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 // �ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // �������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // ����
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

  // UART4��ʼ������
	USART_InitStructure.USART_BaudRate = bound;                                  // ����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                 // �շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);
	
  USART_Cmd(USART3, ENABLE);  // ʹ�ܴ���4
	
	// �����ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;           // ����4�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		     // �����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			       // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  // ��������ж�


}

int laser_test_1;
int laser_test_2;
int USART_LASER_FLAG;
void USART3_IRQHandler(void)                	//����2�жϷ������
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)   
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);  // �����־λ
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
