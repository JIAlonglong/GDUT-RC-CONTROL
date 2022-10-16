#ifndef __TMOTOR_H
#define __TMOTOR_H

void TIM14_PWM_Init(u32 arr,u32 psc);//初始化定时器14的两个通道
void TIM13_PWM_Init(u32 arr,u32 psc);//初始化定时器13的两个通道
void TIM10_PWM_Init(u32 arr,u32 psc);//初始化定时器14的两个通道
void TIM9_PWM_Init(u32 arr,u32 psc);//初始化定时器14的两个通道
void U8_contorl_1(u32 Compare1);
void U8_contorl_2(u32 Compare1);
void U3_contorl_1(u32 Compare1);
void U3_contorl_2(u32 Compare1);
void TMOTOR_init(void);
#endif
