#include "includes.h"
int speed_factor;
void Robot_state_task(void *pvParamerters)//机器人状态
{
	while(1)
	{

//		if(AIR_R_SHORT<1500)//进入正常模式
		if(AIR_R_SHORT>1500)//进入自动模式
		{
		/***********以下为规划路径模式*********************************/	
//			if(AIR_L_LONG<1400)//自动到达取环点
//			{
			MOVE_STATE=MOVE_1_CAST_POINT;

//			}
//			if(1400<AIR_L_LONG&AIR_L_LONG<1600)//自动到达射1
//			{
////			MOVE_STATE=MOVE_1_SHOOT;

//			
//			}
//			if(AIR_L_LONG>1600)//射1到重启区
//			{
////			MOVE_STATE=MOVE_1_RESTART;

//			}
		
		}
		vTaskDelay(5);
		
	}
}
