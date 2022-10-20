#include "includeh.h"
#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0

static uint16_t PPM_buf[8]={0};
uint16_t PPM_Databuf[10]={0};
uint8_t ppm_update_flag=0;
uint32_t now_ppm_time_send=0;
//TIM2_IRQHandler
uint16_t Time_Sys[4]={0};
uint16_t Microsecond_Cnt=0;
u32 TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;

 void PPM_Init(void)//�ݶ�����PF7
{
	EXTI_InitTypeDef EXTI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//�������ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ�� 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource7);
	
	//GPIO����-��������
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	//EXIT����-�ж�ģʽ
	EXTI_InitStruct.EXTI_Line 	 = EXTI_Line7;//PF7
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode 	 = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;//������ʽ
	EXTI_Init(&EXTI_InitStruct);
	
	//NVIC����-�ж�����
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�0
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;//��� 
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
}


void EXTI9_5_IRQHandler(void)
{
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;//�õ����������½��ص�ʱ��
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		//ϵͳ����ʱ���ȡ����λus
		last_ppm_time=now_ppm_time;//��ȡ��һ�εĵ�ǰʱ����Ϊ�ϴ�ʱ��
		now_ppm_time_send=now_ppm_time=10000*TIME_ISR_CNT+TIM2->CNT;//us
		ppm_time_delta=now_ppm_time-last_ppm_time;//����õ�һ������ʱ��
		//PPM������ʼ
		if(ppm_ready==1)//�ж�֡����ʱ����ʼ�����µ�һ��PPM
		{
			if(ppm_time_delta>=2200)//֡������ƽ����2ms=2000us�����ڲ����ϰ汾ң������//���ջ����PPM�źŲ���׼�������ֽ����쳣ʱ�����Ը�С��ֵ�������������һ����ʹ����ط��ϰ汾ң����
			{
				//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_ready = 1;
				ppm_sample_cnt=0;//��Ӧ��ͨ��ֵ
				ppm_update_flag=1;
			}
			else if(ppm_time_delta>=900&&ppm_time_delta<=2100)//����PWM������1000-2000us�������趨900-2100��Ӧ����Ϊ�������ݴ�
			{         
				PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//��Ӧͨ��д�뻺���� 
				if(ppm_sample_cnt>=8)//���ν�������0-7��ʾ8��ͨ���������������ʾ10��ͨ���������ֵӦ��Ϊ0-9�������޸�
				{
					memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));
					//ppm_ready=0;
					ppm_sample_cnt=0;
				}
			}
			else  ppm_ready=0;
		}
		else if(ppm_time_delta>=2200)//֡������ƽ����2ms=2000us
		{
			ppm_ready=1;
			ppm_sample_cnt=0;
			ppm_update_flag=0;
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line7); //���LINE10�ϵ��жϱ�־λ 
}	

void TIM2_GET_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM2ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period = 10000; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler= 84-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��2�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void)//10ms
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		LAST_TIME_ISR_CNT=TIME_ISR_CNT;
		TIME_ISR_CNT++;
		Microsecond_Cnt++;
		if(Microsecond_Cnt>=100)
		{
			Microsecond_Cnt=0;
			Time_Sys[Second]++;
			if(Time_Sys[Second]>=60)
			{
				Time_Sys[Second]=0;
				Time_Sys[Minute]++;
				if(Time_Sys[Minute]>=60)
				{
					Time_Sys[Minute]=0;
					Time_Sys[Hour]++;
				}
			}
		}
		Time_Sys[MicroSecond]=Microsecond_Cnt;
		TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
	}
}
