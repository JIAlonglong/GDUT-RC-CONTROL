#include "includes.h"

void move_task(void *pvParameters)
{
	while(1)
	{	if(AIR_R_SHORT<1500&&mode_flag==0)//��������ģʽ
		{
				if(ROCK_L_Y==0&&ROCK_L_X==0&&ROCK_R_X==0)//��ȫ��
				{
						ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
						ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
						ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
					 
				}
		       else     
				{
					if(AIR_L_LONG>950&&AIR_L_LONG<1250&&AIR_R_LONG>950&&AIR_R_LONG<1250)//��ߵ��͸ߵ�
								{
									Free_Control();//ң�����ɿ���
								}
					if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>800&&AIR_R_LONG<1200)//���е��͸ߵ�
			          {
									int ctrl_flag=0;
			            if(ctrl_flag==0)
									{
										MOVE_STATE=MOVE_1_SHOOT;//����1���价��/point to point
										ctrl_flag+=1;
									}
									else
                  {
										MOVE_STATE=MOVE_STOP;
										if(MOVE_STATE==MOVE_STOP)
										{
											//�������
										}
									}
			          }
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>1460&&AIR_R_LONG<1540)//���е����е�
			          {
									int ctrl_flag=0;
			            if(ctrl_flag==0)
									{
										MOVE_STATE=MOVE_2_SHOOT;//����2���价��/point to point
										ctrl_flag+=1;
									}
									else
                  {
										MOVE_STATE=MOVE_STOP;
										if(MOVE_STATE==MOVE_STOP)
										{
											//�������
										}
									}	
			          }
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>1850&&AIR_R_LONG<2100)//���е��͵͵�
			          {
									int ctrl_flag=0;
			            if(ctrl_flag==0)
									{
										MOVE_STATE=MOVE_3_SHOOT;//����3���价��/point to point
										ctrl_flag+=1;
									}
									else
                  {
										MOVE_STATE=MOVE_STOP;
										if(MOVE_STATE==MOVE_STOP)
										{
											//�������
										}
									}
			          }
			        if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>800&&AIR_R_LONG<1200)//��͵��͸ߵ�
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_OUR;//���ҷ�����
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP)
									{
										LaserLockPoint(ros_vx , ros_vy ,100,200);//��λcm
									}
								}
					 if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>1460&&AIR_R_LONG<1540)//��͵����е�
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_HIGH;//���м�����
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP)
									{
										LaserLockPoint(ros_vx , ros_vy ,100,200);//��λcm
									}
								}
					  if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>1850&&AIR_R_LONG<2100)//��͵��͵͵�
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_TH;//��Է�����
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP)
									{
										LaserLockPoint(ros_vx , ros_vy ,100,200);//��λcm
									}
								}
				      if(ROCK_R_Y>1850)//���ұ��͸�
				      {
						ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
						ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
						ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
					    //PUSH
				      }
					 

			     }	
		}
		if(AIR_R_SHORT>1500&&mode_flag==0)//�����Զ�ģʽ
		{
			int ctrl_flag=0;
			if(ctrl_flag==0)
			{
				MOVE_STATE=MOVE_1_CAST_POINT;
				ctrl_flag+=1;
			}
			else
			{
				MOVE_STATE=MOVE_STOP;
						if(MOVE_STATE==MOVE_STOP)
						{
							//�������
						}
			}
			
		}
		if(mode_flag==2)//ROS�ص���
		{
			
			ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=ros_vx;
			ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=ros_vy;
			ROBOT_TARGET_VELOCITY_DATA.W_RPM=ros_vz;//��Ϊ�Ƕȣ���
			
		}
		usartSendData(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM, ROBOT_TARGET_VELOCITY_DATA.Vy_RPM,ROBOT_TARGET_VELOCITY_DATA.W_RPM,ROBOT_REAL_POS_DATA.POS_X,ROBOT_REAL_POS_DATA.POS_Y,ROBOT_REAL_POS_DATA.POS_YAW,rosctrl_flag);
		vTaskDelay(13);
		move();
		vTaskDelay(5);
	}
	
	


}