//#ifndef __ADC_H
//#define __ADC_H	
//#include "sys.h"

//#define ADC_ChANEL_NUMBER 8



//extern unsigned int adc_cache[5][ADC_ChANEL_NUMBER];

//extern unsigned int adc_value[ADC_ChANEL_NUMBER];        //�洢ÿ��ͨ���˲����ȥƽ�����ֵ

//extern uint16_t adc_threshould_value[ADC_ChANEL_NUMBER];     //�洢�̶�����ֵ

//void Adc_Init(void);

//#endif 
#ifndef __ADC_H
#define __ADC_H 			   
#include "sys.h"


#define ADC1_DR_Address    ((uint32_t)0x4001244C) //ADC1�������ĵ�ַ����ο��ֲ�ó���

#define ADCPORT		GPIOA	//����ADC�ӿ�
#define ADC_CH0		GPIO_Pin_0	//����ADC�ӿ� 
#define ADC_CH1		GPIO_Pin_1	//����ADC�ӿ� 
#define ADC_CH2		GPIO_Pin_2	//����ADC�ӿ� 
#define ADC_CH3		GPIO_Pin_3	//����ADC�ӿ� 
#define ADC_CH4		GPIO_Pin_4	//����ADC�ӿ� 
#define ADC_CH5		GPIO_Pin_5	//����ADC�ӿ� 
#define ADC_CH6		GPIO_Pin_6	//����ADC�ӿ� 
#define ADC_CH7		GPIO_Pin_7	//����ADC�ӿ�
#define ADC_ChANEL_NUMBER 7   //����8��ADCͨ��

extern vu16 ADC_DMA_IN[8]; //ADC��ֵ��ŵı���

extern vu16 ADC_IN[8]; //���¶���һ��������ADC��ֵ��ŵı�������������

extern float RANGING_DISTANCE ;

extern u32 w ;

	 
void ADC_DMA_Init(void);
void ADC_GPIO_Init(void);
void ADC_Configuration(void);
u16 Get_Adc_Average(u8 ch,u8 times); 
float adc_update_ranging_distance(u8 ch);
void get_adc(void);
#endif
