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

 void PPM_Init(void)//暂定采用PF7
{
	EXTI_InitTypeDef EXTI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//开启相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource7);
	
	//GPIO设置-下拉输入
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	//EXIT设置-中断模式
	EXTI_InitStruct.EXTI_Line 	 = EXTI_Line7;//PF7
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode 	 = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;//触发方式
	EXTI_Init(&EXTI_InitStruct);
	
	//NVIC设置-中断配置
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断0
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;//最高 
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
}


void EXTI9_5_IRQHandler(void)
{
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;//得到上升沿与下降沿的时间
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		//系统运行时间获取，单位us
		last_ppm_time=now_ppm_time;//获取上一次的当前时间作为上次时间
		now_ppm_time_send=now_ppm_time=10000*TIME_ISR_CNT+TIM2->CNT;//us
		ppm_time_delta=now_ppm_time-last_ppm_time;//相减得到一个周期时间
		//PPM解析开始
		if(ppm_ready==1)//判断帧结束时，开始解析新的一轮PPM
		{
			if(ppm_time_delta>=2200)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、//接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
			{
				//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_ready = 1;
				ppm_sample_cnt=0;//对应的通道值
				ppm_update_flag=1;
			}
			else if(ppm_time_delta>=900&&ppm_time_delta<=2100)//单个PWM脉宽在1000-2000us，这里设定900-2100，应该是为了提升容错
			{         
				PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//对应通道写入缓冲区 
				if(ppm_sample_cnt>=8)//单次解析结束0-7表示8个通道。我这里可以显示10个通道，故这个值应该为0-9！！待修改
				{
					memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));
					//ppm_ready=0;
					ppm_sample_cnt=0;
				}
			}
			else  ppm_ready=0;
		}
		else if(ppm_time_delta>=2200)//帧结束电平至少2ms=2000us
		{
			ppm_ready=1;
			ppm_sample_cnt=0;
			ppm_update_flag=0;
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line7); //清除LINE10上的中断标志位 
}	

void TIM2_GET_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM2时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = 10000; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler= 84-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器2更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器2
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
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
