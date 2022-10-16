#include "includes.h"

void move_task(void *pvParameters)
{
	while(1)
	{	if(AIR_R_SHORT<1500)//进入正常模式
		{
					 
		// 速度分解
		
			//     vy=(ROCK_L_Y_Processed/10-150.0)*speed_factor;     
				 if(ROCK_L_Y>1460&&ROCK_L_Y<1540)   ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
				 else if(ROCK_L_Y>=1540) ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1540)*1.5;
				 else if(ROCK_L_Y<=1460) ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1460)*1.5;
				 
		//     vx=(ROCK_L_X_Processed/10-150.0)*speed_factor;
				 if(ROCK_L_X>1460&&ROCK_L_X<1540)   ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
				 else if(ROCK_L_X>=1540) ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1540)*1.5;
				 else if(ROCK_L_X<=1460) ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1460)*1.5;
				 
		//     vw=-(ROCK_R_X_Processed/10-150.0)*speed_factor*1.5;
				 if(ROCK_R_X>1400&&ROCK_R_X<1600)   ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
				 else if(ROCK_R_X>=1600) ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_X-1600)*1.5;
				 else if(ROCK_R_X<=1400) ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_X-1400)*1.5;	
					
				 
				 
			
		}
		move();
		vTaskDelay(5);
	}
	
	


}