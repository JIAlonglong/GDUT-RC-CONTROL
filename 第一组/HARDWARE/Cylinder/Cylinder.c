#include "includeh.h"
void Cylinder_Init(void)
{
	//气缸引脚初始化
	GPIO_InitTypeDef  GPIO_InitStructure;   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  // 普通输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100M
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_InitTypeDef  GPIO_InitStructureA; 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructureA.GPIO_Mode = GPIO_Mode_OUT;  // 普通输出模式
	GPIO_InitStructureA.GPIO_Speed = GPIO_Speed_100MHz;  //100M
	GPIO_InitStructureA.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureA.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructureA.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructureA);
	
	GPIO_InitTypeDef  GPIO_InitStructureB; 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructureB.GPIO_Mode = GPIO_Mode_OUT;  // 普通输出模式
	GPIO_InitStructureB.GPIO_Speed = GPIO_Speed_100MHz;  //100M
	GPIO_InitStructureB.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureB.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructureB.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructureB);	
	
}
