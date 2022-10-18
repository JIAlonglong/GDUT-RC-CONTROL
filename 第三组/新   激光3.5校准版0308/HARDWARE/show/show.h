#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
#define START 0X11
int transfer(int x);
void oled_show(void);
void USART_Send_String(u8 *p,u16 sendSize);//发送指定字符数组字符串

//void VisionSendData(int X,int Y,int yaw);//发送数据(标定用数据)

unsigned char getCrc8(unsigned char *ptr,unsigned short len);
int VisionReceiveData(int *theta_angle,int *theta_pitch);
void VisionSendData(int X,int Y);

#endif
