#ifndef __AIR_H
#define __AIR_H
#include "stm32f4xx.h"
#include "sys.h"
#define ROCK_R_X			PPM_Databuf[0]				//PITCH 1000-1500-2000P
#define ROCK_R_Y			PPM_Databuf[1]				//ROLL 1000-1500-2000
#define ROCK_L_Y			PPM_Databuf[2]				//THR  1000-1500-2000
#define ROCK_L_X			PPM_Databuf[3]				//YAW  1000-1500-2000
//#define AIR_L_SHORT		PPM_Databuf[4]				//AUX4 1000~2000
#define SWA		PPM_Databuf[4]				//AUX3 1000-1500-2000
#define SWB	PPM_Databuf[5]				//AUX2 1000-1500-2000                                  
#define SWC		PPM_Databuf[6]
#define SWD		PPM_Databuf[7]			//AUX1 1000~2000
extern uint16_t PPM_Databuf[10];
extern u32 TIME_ISR_CNT,LAST_TIME_ISR_CNT;
//void TIM2_IRQHandler(void);//10ms
void TIM2_GET_TIM_Init(void);
//void EXTI2_IRQHandler(void);
void PPM_Init(void);// π”√PE2

#endif
