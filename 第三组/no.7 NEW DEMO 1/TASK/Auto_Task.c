#include "includes.h"
extern MOVE_STATE_ITEMS MOVE_STATE ;
float CUR_POS[3] = {0, 0, 0};	// 当前位置x,y,yaw
	float move_time_counter = 0;
void Auto_Task(void *pvParameters)
{
  static portTickType move_xLastWakeTime;
	const portTickType move_xFrequency = pdMS_TO_TICKS(10); // 延时10ms
	move_xLastWakeTime = xTaskGetTickCount(); // 获取当前计数值
	
	while(1)
	{
		switch(MOVE_STATE)
		{
			// 停止
			case MOVE_STOP:
					ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
					ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
					ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
//				LockupPoint(CUR_POS[0], CUR_POS[1], CUR_POS[2]);
				break;
			case MOVE_1_CAST_POINT:
				move_time_counter += 0.01f;
			if(PathPlan(move_time_counter, 8.0, 9+1, X, Y, Yaw))
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;
			case MOVE_1_SHOOT:
				move_time_counter += 0.01f;
			if(moving_point_track(move_time_counter,-4900, 78, 0,800))
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;
			case MOVE_2_SHOOT:
				move_time_counter += 0.01f;
			if(moving_point_track(move_time_counter,-3174, 78, 0,800))
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;
			case MOVE_3_SHOOT:
				move_time_counter += 0.01f;
			if(moving_point_track(move_time_counter,-1425, 78, 0,800))
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;
			case MOVE_1_RESTART:
				move_time_counter += 0.01f;
			if(moving_point_track(move_time_counter,-6227, 75, 0,800))//取环点
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;
			case MOVE_1_SHOOT_CLOSE:
				move_time_counter += 0.01f;
			if(moving_point_track(move_time_counter,-5759, 1028, 0,250))
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;
			case MOVE_2_SHOOT_CLOSE:
				move_time_counter += 0.01f;
			if(moving_point_track(move_time_counter,-3592, 952, 0,250))
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;
			case MOVE_3_SHOOT_CLOSE:
				move_time_counter += 0.01f;
			if(moving_point_track(move_time_counter,-625, 915, 0,250))
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;
			case MOVE_JI:
				move_time_counter += 0.01f;
			if(moving_point_track(move_time_counter,-4037, 915, 0,250))
				{
					move_time_counter = 0;
					MOVE_STATE = MOVE_STOP;
				}
			 break;	
			
			default:
				break;
		}
		
		vTaskDelayUntil(&move_xLastWakeTime, move_xFrequency); // 绝对延时
	
	}

}
