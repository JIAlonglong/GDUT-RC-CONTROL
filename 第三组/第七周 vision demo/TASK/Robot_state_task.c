#include "includes.h"
int speed_factor;
void Robot_state_task(void *pvParamerters)//������״̬
{
	while(1)
	{
		LaserLockPoint(ros_vx , ros_vy ,100,200);//��λcm

//		if(AIR_R_SHORT<1500)//��������ģʽ
//		if(AIR_R_SHORT>1500)//�����Զ�ģʽ
//		{
		/***********����Ϊ�滮·��ģʽ*********************************/	
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
			
			
//			if(AIR_L_LONG<1400)//�Զ�����ȡ����
//			{
//			MOVE_STATE=MOVE_1_CAST_POINT;

//			}
//			if(1400<AIR_L_LONG&AIR_L_LONG<1600)//�Զ�������1
//			{
////			MOVE_STATE=MOVE_1_SHOOT;

//			
//			}
//			if(AIR_L_LONG>1600)//��1��������
//			{
////			MOVE_STATE=MOVE_1_RESTART;

//			}
		
//		}
//  	 usartSendData(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM, ROBOT_TARGET_VELOCITY_DATA.Vy_RPM,ROBOT_TARGET_VELOCITY_DATA.W_RPM,ROBOT_REAL_POS_DATA.POS_X,ROBOT_REAL_POS_DATA.POS_Y,ROBOT_REAL_POS_DATA.POS_YAW,rosctrl_flag);
//		vTaskDelay(13);		
		vTaskDelay(5);
		
	}
}
