#include "includes.h"
short v;
short th;
void data_update(void *pvParameters)
{
	while(1)
	{
		usartSendData(v,th,1,1,1,1,rosctrl_flag);
		//usartSendData(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM, ROBOT_TARGET_VELOCITY_DATA.Vy_RPM,ROBOT_TARGET_VELOCITY_DATA.W_RPM,ROBOT_REAL_POS_DATA.POS_X,ROBOT_REAL_POS_DATA.POS_Y,ROBOT_REAL_POS_DATA.POS_YAW,rosctrl_flag);
  	vTaskDelay(13);
	}
}	