#include "includes.h"

void PS2_Control_task(void *pvParameters)
{
	while(1)
	{
//	int RC_Velocity=50,RC_Position=3000;         //设置遥控的速度和位置值
//  int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; //PS2相关变量 
//	float LY,RX,LX;  //PS2手柄控制变量
//	int Yuzhi=2;  		//PS2控制防抖阈值
//	LX=PS2_LX-128; //获取偏差
//	LY=PS2_LY-128; //获取偏差
//	RX=PS2_RX-128; //获取偏差
//	if(LX>-Yuzhi&&LX<Yuzhi)LX=0; //设置小角度的死区
//	if(LY>-Yuzhi&&LY<Yuzhi)LY=0; //设置小角度的死区
//	if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //设置小角度的死区
//	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=LX*RC_Velocity/200;/航模用过，这里不能再代
//	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=-LY*RC_Velocity/200;	
//	ROBOT_TARGET_VELOCITY_DATA.W_RPM=RX*RC_Velocity/200;
	vTaskDelay(5);
  }		
}
