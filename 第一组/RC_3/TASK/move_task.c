#include "includeh.h"
extern MOVE_STATE_ITEMS MOVE_STATE ;
float CUR_POS[3] = {0, 0, 0};	// 当前位置x,y,yaw
	float move_time_counter = 0;

  int fine_turning_state=0;
	int unautomatic=0;
	int is_MOVE_SHOOTING_1=0;
	int is_MOVE_SHOOTING_2=0;
	int  is_MOVE_RING=0;
	

void Move_task(void *pvParameters)
{
  static portTickType move_xLastWakeTime;
	const portTickType move_xFrequency = pdMS_TO_TICKS(10); // 延时10ms
	move_xLastWakeTime = xTaskGetTickCount(); // 获取当前计数值
//	float move_time_counter = 0;
	
	
	
	while(1)
	{
		if(SWA<1500)
		{
		if(SWC>=1400 && SWC<=1600)
		{
	ACTION_GL_POS_DATA.REAL_X = 0;
	ACTION_GL_POS_DATA.REAL_Y = 0;
		}
		
		
//		if(SWB>=1400 && SWB<=1600)
//		Action_update(0,0,0);
	switch(MOVE_STATE)
	  {
		case MOVE_STOP:
			if(SWD>1500&&SWA<1500)
			{
				MOVE_STATE= MOVE_RING;
			}
			fine_turning_state=0;//不可微调
					
			ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
		  ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
		  ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
		break;
			
		case MOVE_RING:	
		  fine_turning_state=0;//不可微调					
			move_time_counter += 0.01f;
		  if(PathPlan(move_time_counter, 10, 9+1, X, Y, Yaw))
		  {
			  move_time_counter = 0;
			  MOVE_STATE = RING_STOP;	
        is_MOVE_RING=1;
    			
		  }
		break;
		
		case RING_STOP:
		if(SWD>1500)
		MOVE_STATE= MOVE_SHOOTING_1;
		 fine_turning_state=1;//可微调
		break;
		
		case MOVE_SHOOTING_1:
		fine_turning_state=0;//不可微调		
			move_time_counter += 0.01f;
			if(PathPlan(move_time_counter, 3, 2+1, X2, Y2, Yaw2))
		{
			
			move_time_counter = 0;
			
			
			
			MOVE_STATE = SHOOTING_STOP;		
		}
			
			
				
			break;
		case SHOOTING_STOP:
				fine_turning_state=1;//可微调
		/*
			if(Laser_calibration(2000, 3450,2000,3500)==0)
			{
			MOVE_STATE = MOVE_STOP;		
			}
			
			*/
				
		
		break;
		
		case MOVE_SHOOTING_2:
			//while(Laser_calibration(7350, 3000,0,0)==0)	{}
		break;
		
		
		default:
			break;
			
	  }
		
			vTaskDelayUntil(&move_xLastWakeTime, move_xFrequency); // 绝对延时
	
	}	
}
	
}
