#ifndef __CAN2_H
#define __CAN2_H	 
#include "includeh.h"
#define CAN2_RX0_INT_ENABLE	1	//0,不使能;1,使能.			
 
 
  void CAN2_Init(void);
	
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
#endif

