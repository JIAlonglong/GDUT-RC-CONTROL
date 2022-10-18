#ifndef _ACTION_H
#define _ACTION_H


#define INSTALL_ERROR_Y		190.3f
#define INSTALL_ERROR_X		0

#include "includeh.h"

typedef struct ACTION_GL_POS
{
	float ANGLE_Z;
	float ANGLE_X;
	float ANGLE_Y;
	float POS_X;
	float POS_Y;
	float W_Z;

	float LAST_POS_X;
	float LAST_POS_Y;

	float DELTA_POS_X;
	float DELTA_POS_Y;
	
	float REAL_X;
	float REAL_Y;
	
	
} ACTION_GL_POS;

extern ACTION_GL_POS ACTION_GL_POS_DATA;

void bsp_UART4_Init(u32 baud_rate);
void UART4_IRQHandler(void);
void Update_Action(float value[6]);
void UART_SendString(USART_TypeDef* USARTx, char *DataString);

#endif
