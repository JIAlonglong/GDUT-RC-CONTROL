#ifndef __ROS__H_
#define __ROS__H_

#define START   0X11

//数据接收暂存区
extern unsigned char  receiveBuff[32];         
//通信协议常量
extern const unsigned char header[2];
extern const unsigned char ender[2];

//从linux接收并解析数据到参数地址中
extern int usartReceiveOneData(int *p_VXSet,int *p_VYSet,int *p_VZSet,int *p_update_X,int *p_update_Y,int *p_update_Z,unsigned char *p_crtlFlag);
//封装数据，调用USART1_Send_String将数据发送给linux
extern void usartSendData(short VX, short VY,short VZ,short Robot_X,short Robot_Y,short Robot_Z,unsigned char ctrlFlag);
//发送指定字符数组的函数
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//计算八位循环冗余校验，得到校验值，一定程度上验证数据的正确性
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 



#endif 
