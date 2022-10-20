#include "includeh.h"




void Update_J(float New_J)
{
 int i =0;
 static union
 {
  float J;
  char data[4];
 }New_set;
 
 New_set.J = New_J;
 
 UART_SendString(UART4, "ACTJ");
 for(i=0;i<4;i++)
 {
  USART_SendData(UART4,New_set.data[i]); 
  while(USART_GetFlagStatus(UART4,USART_FLAG_TC) == 0);     //????????  
 } 
 
 USART_ClearFlag(UART4,USART_FLAG_TC);          //??????????(?????????????)   
 
}

void Update_X(float New_X)      //????ACTION_GL_POS_DATA.REAL_X,?????????
{
 int i =0;
 static union
 {
  float X;
  char data[4];
 }New_set;
 
 New_set.X = New_X;
 
 UART_SendString(UART4, "ACTX");
 for(i=0;i<4;i++)
 {
  USART_SendData(UART4,New_set.data[i]); 
  while(USART_GetFlagStatus(UART4,USART_FLAG_TC) == 0);     //????????  
 } 
 
 USART_ClearFlag(UART4,USART_FLAG_TC);          //??????????(?????????????)   
 
}

void Update_Y(float New_Y)      //????ACTION_GL_POS_DATA.REAL_Y,?????????
{
 int i =0;
 static union
 {
  float Y;
  char data[4];
 }New_set;
 
 New_set.Y = New_Y;
 
 UART_SendString(UART4, "ACTY");
 for(i=0;i<4;i++)
 {
  USART_SendData(UART4,New_set.data[i]); 
  while(USART_GetFlagStatus(UART4,USART_FLAG_TC) == 0);     //????????  
 } 
 
 USART_ClearFlag(UART4,USART_FLAG_TC);          //??????????(?????????????)   
}

void Action_update(float New_X,float New_Y,float New_J)
{
	Update_J(New_J);
	Update_X(New_X);
	Update_Y(New_Y);
				
}

