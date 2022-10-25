#include "includes.h"
int speed_factor;
void Robot_state_task(void *pvParamerters)//机器人状态
{
	while(1)
	{
		switch(ROBOT_SHOOTING)
		{
			case SHOOT_WAITING:
				UP_ARM_NOW_MOTION=&UP_DOWN3;
//				U8_contorl_1(1000);
				U8_contorl_2(0);
				AK80_Speed_Control(AK80_ID1,0);
				AK80_Speed_Control(AK80_ID2,0);	
				break;
			case SHOOT_INIT:
			
				UP_ARM_NOW_MOTION=&UP_INIT;
//				U8_contorl_1(1050);
				U8_contorl_2(0);//可以
				AK80_Speed_Control(AK80_ID1,0);
				AK80_Speed_Control(AK80_ID2,0);		
				break;
			
			
			case ROBOT_SHOOT_OUR:
			
				UP_ARM_NOW_MOTION=&UP_ON1;
//				U8_contorl_1(0);
				U8_contorl_2(1500);
				AK80_Speed_Control(AK80_ID1,0);
				AK80_Speed_Control(AK80_ID2,0);	
				break;
			
		 
			
			case ROBOT_SHOOT_HIGH:
			
				UP_ARM_NOW_MOTION=&UP_ON2;
//				U8_contorl_1(0);
				U8_contorl_2(1500);
				AK80_Speed_Control(AK80_ID1,0);
				AK80_Speed_Control(AK80_ID2,0);				
				break;
			
			
			case ROBOT_SHOOT_TH:
			
				UP_ARM_NOW_MOTION=&UP_ON3;
//				U8_contorl_1(0);
				U8_contorl_2(1500);
				AK80_Speed_Control(AK80_ID1,0);
				AK80_Speed_Control(AK80_ID2,0);				
				break;
				
			
			default:
			break;
		
		}


		vTaskDelay(5);
		}
		
}

//时间控制在2ms
void Motor_Control(void *pvParameters)
{
	while(1)
	{
		RPM_MOTOR_PLANNING();//电机的速度规划
		vTaskDelay(5);  
	}
}

