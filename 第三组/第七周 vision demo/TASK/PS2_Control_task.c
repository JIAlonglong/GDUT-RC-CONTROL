#include "includes.h"

void PS2_Control_task(void *pvParameters)
{
	while(1)
	{
//	int RC_Velocity=50,RC_Position=3000;         //����ң�ص��ٶȺ�λ��ֵ
//  int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; //PS2��ر��� 
//	float LY,RX,LX;  //PS2�ֱ����Ʊ���
//	int Yuzhi=2;  		//PS2���Ʒ�����ֵ
//	LX=PS2_LX-128; //��ȡƫ��
//	LY=PS2_LY-128; //��ȡƫ��
//	RX=PS2_RX-128; //��ȡƫ��
//	if(LX>-Yuzhi&&LX<Yuzhi)LX=0; //����С�Ƕȵ�����
//	if(LY>-Yuzhi&&LY<Yuzhi)LY=0; //����С�Ƕȵ�����
//	if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //����С�Ƕȵ�����
//	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=LX*RC_Velocity/200;/��ģ�ù������ﲻ���ٴ�
//	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=-LY*RC_Velocity/200;	
//	ROBOT_TARGET_VELOCITY_DATA.W_RPM=RX*RC_Velocity/200;
	vTaskDelay(5);
  }		
}
