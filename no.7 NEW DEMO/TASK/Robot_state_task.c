#include "includes.h"
int speed_factor;
void Robot_state_task(void *pvParamerters)//������״̬
{
	while(1)
	{
		switch(ROBOT_SHOOTING)
		{
			case SHOOT_WAITING:
			

			
				break;
			case SHOOT_INIT:
			
				UP_ARM_NOW_MOTION=&UP_INIT;
				U8_contorl_1(0);
				U8_contorl_2(0);
			
				break;
			
			
			case ROBOT_SHOOT_OUR:
			
				UP_ARM_NOW_MOTION=&UP_ON;
				U8_contorl_1(0);
				U8_contorl_2(0);
			
				break;
			
		 
			
			case ROBOT_SHOOT_HIGH:
			
				UP_ARM_NOW_MOTION=&UP_ON;
				U8_contorl_1(0);
				U8_contorl_2(0);
			
				break;
			
			
			case ROBOT_SHOOT_TH:
			
				UP_ARM_NOW_MOTION=&UP_ON;
				U8_contorl_1(0);
				U8_contorl_2(0);
			
				break;
			
			default:
			break;
		
		}


		vTaskDelay(5);
		}
		
}

//ʱ�������2ms
void Motor_Control(void *pvParameters)
{
	while(1)
	{
		RPM_MOTOR_PLANNING();//������ٶȹ滮
		vTaskDelay(5);  
	}
}

