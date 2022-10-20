#ifndef  __VISION_H
#define  __VISION_H
#include "includeh.h"
//意义暂不明确
#define START 0X11
unsigned char getCrc8(unsigned char *ptr,unsigned short len);
int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag);
void USART_Send_String(u8 *p,u16 sendSize);
void VisionSendData(short X,short Y,short yaw);
int VisionReceiveData(int *theta_angle,int *theta_pitch);

int Laser_calibration(float x, float y,float action_x,float action_y);
#endif
