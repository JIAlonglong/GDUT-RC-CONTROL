#include "includes.h"
int speed_factor;
int up_flag=1;
void Robot_state_task(void *pvParamerters)//机器人状态
{
	while(1)
	{
		switch(ROBOT_SHOOTING)
		{
			case SHOOT_WAITING:
			{
				UP_ARM_NOW_MOTION=&UP_DOWN3;
				U8_contorl_2(1000);
				Servo_contorl(1000);
				set_pick_transate_speed(0);
				set_pick_speed(0);
				YAW_ARM_NOW_MOTION=&YAW_INIT;
				break;
			}
			case SHOOT_INIT:
			{
				UP_ARM_NOW_MOTION=&UP_DOWN3;
				U8_contorl_2(1000);
				Servo_contorl(1500);
				set_pick_transate_speed(0);
				set_pick_speed(0);
				YAW_ARM_NOW_MOTION=&YAW_MOVE_1;	
				break;
			}
			
			case ROBOT_PICK:
			
				{
				UP_ARM_NOW_MOTION=&UP_DOWN3;
				U8_contorl_2(1000);
				set_pick_transate_speed(500);
				set_pick_speed(500);
				YAW_ARM_NOW_MOTION=&YAW_MOVE_1;	
				break;
				}
		 
			
			case  ROBOT_SHOTH://包含special
			{
				UP_ARM_NOW_MOTION=&UP_ON1;
				U8_contorl_2(1700);
				set_pick_transate_speed(0);
				set_pick_speed(0);
				YAW_ARM_NOW_MOTION=&YAW_MOVE_1;	
				break;
				}
			
			
			case ROBOT_MIDDLE:
			{
				UP_ARM_NOW_MOTION=&UP_ON2;
				U8_contorl_2(1700);
				set_pick_transate_speed(0);
				set_pick_speed(0);
				YAW_ARM_NOW_MOTION=&YAW_MOVE_1;
				break;
			case ROBOT_HIGH:
			{
				UP_ARM_NOW_MOTION=&UP_ON3;
				U8_contorl_2(1700);
				set_pick_transate_speed(0);
				set_pick_speed(0);
				YAW_ARM_NOW_MOTION=&YAW_MOVE_1;
				break;
			}
			
			default:
			break;
		
		}


		vTaskDelay(5);
		}
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

