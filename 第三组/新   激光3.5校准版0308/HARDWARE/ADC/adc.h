//#ifndef __ADC_H
//#define __ADC_H	
//#include "sys.h"

//#define ADC_ChANEL_NUMBER 8



//extern unsigned int adc_cache[5][ADC_ChANEL_NUMBER];

//extern unsigned int adc_value[ADC_ChANEL_NUMBER];        //存储每个通道滤波后或去平均后的值

//extern uint16_t adc_threshould_value[ADC_ChANEL_NUMBER];     //存储固定的阈值

//void Adc_Init(void);

//#endif 
#ifndef __ADC_H
#define __ADC_H 			   
#include "sys.h"


#define ADC1_DR_Address    ((uint32_t)0x4001244C) //ADC1这个外设的地址（查参考手册得出）

#define ADCPORT		GPIOA	//定义ADC接口
#define ADC_CH0		GPIO_Pin_0	//定义ADC接口 
#define ADC_CH1		GPIO_Pin_1	//定义ADC接口 
#define ADC_CH2		GPIO_Pin_2	//定义ADC接口 
#define ADC_CH3		GPIO_Pin_3	//定义ADC接口 
#define ADC_CH4		GPIO_Pin_4	//定义ADC接口 
#define ADC_CH5		GPIO_Pin_5	//定义ADC接口 
#define ADC_CH6		GPIO_Pin_6	//定义ADC接口 
#define ADC_CH7		GPIO_Pin_7	//定义ADC接口
#define ADC_ChANEL_NUMBER 7   //定义8个ADC通道

extern vu16 ADC_DMA_IN[8]; //ADC数值存放的变量

extern vu16 ADC_IN[8]; //重新定义一个数组存放ADC数值存放的变量来处理数据

extern float RANGING_DISTANCE ;

extern u32 w ;

	 
void ADC_DMA_Init(void);
void ADC_GPIO_Init(void);
void ADC_Configuration(void);
u16 Get_Adc_Average(u8 ch,u8 times); 
float adc_update_ranging_distance(u8 ch);
void get_adc(void);
#endif
