#ifndef __CYLINDER_H
#define __CYLINDER_H
#include "stm32f4xx.h"
#define Cylinder_PUSH  GPIO_SetBits(GPIOB,GPIO_Pin_14)					//��������
#define Cylinder_BACK  GPIO_ResetBits(GPIOB ,GPIO_Pin_14)					//��������

#define Cylinder_Close GPIO_SetBits(GPIOB,GPIO_Pin_15)		     //��צ�н�
#define Cylinder_Open  GPIO_ResetBits(GPIOB,GPIO_Pin_15)		      //��צ�ɿ�

void Cylinder_Init(void);

#endif
