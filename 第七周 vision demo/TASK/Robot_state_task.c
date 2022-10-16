#include "includes.h"
int speed_factor;
void Robot_state_task(void *pvParamerters)//机器人状态
{
	while(1)
	{
		LaserLockPoint(ros_vx , ros_vy ,100,200);//单位cm

//		if(AIR_R_SHORT<1500)//进入正常模式
//		if(AIR_R_SHORT>1500)//进入自动模式
//		{
		/***********以下为规划路径模式*********************************/	
//		 usartReceiveOneData(&ros_vx,&ros_vy,&ros_vz,&new_x,&new_y,&new_z,&rosctrl_flag);
									
//		    if(ros_vx>=100)
//				{
//					ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=-80;
//					ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
////					YawAdjust(90);
//				}
//				else
//				{
//					if(ROBOT_REAL_POS_DATA.POS_YAW!=ros_vz)
//					{
//						ROBOT_TARGET_VELOCITY_DATA.W_RPM=20;
//							ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
//					ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
//						YawAdjust(ros_vy+ROBOT_REAL_POS_DATA.POS_YAW);
					
						move();

				
//				}
			
			
//			if(AIR_L_LONG<1400)//自动到达取环点
//			{
//			MOVE_STATE=MOVE_1_CAST_POINT;

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
		
//		}
//  	 usartSendData(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM, ROBOT_TARGET_VELOCITY_DATA.Vy_RPM,ROBOT_TARGET_VELOCITY_DATA.W_RPM,ROBOT_REAL_POS_DATA.POS_X,ROBOT_REAL_POS_DATA.POS_Y,ROBOT_REAL_POS_DATA.POS_YAW,rosctrl_flag);
//		vTaskDelay(13);		
		vTaskDelay(5);
		
	}
}
