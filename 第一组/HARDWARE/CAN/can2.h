#ifndef __CAN2_H
#define __CAN2_H	 
#include "includeh.h"
#define CAN2_RX0_INT_ENABLE	1	//0,��ʹ��;1,ʹ��.			
 
 
  void CAN2_Init(void);
	
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
#endif

