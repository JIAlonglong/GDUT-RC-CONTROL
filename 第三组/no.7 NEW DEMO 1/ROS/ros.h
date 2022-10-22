#ifndef __ROS__H_
#define __ROS__H_

#define START   0X11

//���ݽ����ݴ���
extern unsigned char  receiveBuff[32];         
//ͨ��Э�鳣��
extern const unsigned char header[2];
extern const unsigned char ender[2];

//��linux���ղ��������ݵ�������ַ��
extern int usartReceiveOneData(int *p_VXSet,int *p_VYSet,int *p_VZSet,int *p_update_X,int *p_update_Y,int *p_update_Z,unsigned char *p_crtlFlag);
//��װ���ݣ�����USART1_Send_String�����ݷ��͸�linux
extern void usartSendData(short VX, short VY,short VZ,short Robot_X,short Robot_Y,short Robot_Z,unsigned char ctrlFlag);
//����ָ���ַ�����ĺ���
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//�����λѭ������У�飬�õ�У��ֵ��һ���̶�����֤���ݵ���ȷ��
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 



#endif 
