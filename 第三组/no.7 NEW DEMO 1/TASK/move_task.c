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
					 //ʵ��							
//			      if(1)//���ұ��͸�
//				      {
//						ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
//						ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
//						ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
//					    //PUSH PUSH UPDATE ANGLE
//						PUSH(0,-4000,10000,9000,0,0.4,0.5);//void PUSH(float start,float end,float speedmax,float speedstart,float speedend,float ac,float de)
//				      }
				}
		       else     
				{
					if(AIR_L_LONG>950&&AIR_L_LONG<1250&&AIR_R_LONG>950&&AIR_R_LONG<1250)//��ߵ��͸ߵ�
								{
										ROBOT_SHOOTING=SHOOT_WAITING;
										Free_Control();//ң�����ɿ���
								}
					if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_L_X>800&&ROCK_L_X<1200)//���е��������
			          {
										ROBOT_SHOOTING=SHOOT_INIT;
										MOVE_STATE=MOVE_1_RESTART;//ȡ����
			          }
					  if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_L_Y>1850&&ROCK_L_Y<2150)//���е��������
			          {
										ROBOT_SHOOTING=SHOOT_INIT;
										MOVE_STATE=MOVE_1_SHOOT;
			          }
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_L_X>1850&&ROCK_L_X<2150)//���е��������
			          {
										ROBOT_SHOOTING=SHOOT_INIT;
										MOVE_STATE=MOVE_2_SHOOT;
			          }
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_L_Y>850&&ROCK_L_Y<1250)//���е��������
			          {
										ROBOT_SHOOTING=SHOOT_INIT;
										MOVE_STATE=MOVE_3_SHOOT;
			          }
			        if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>800&&AIR_R_LONG<1200)//��͵��͸ߵ�
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_OUR;//���ҷ�����
										if(ROCK_R_Y>1850)//���ұ��͸�
									{
											ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
													//PUSH PUSH UPDATE ANGLE
											PUSH(0,-2000,5000,4000,0,0.4,0.5);//��Χ��1000-10000
									}
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP)
									{
										if(LaserLockPoint(ros_vx , ros_vy ,100,200))//��λcm
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}
									}
								}
					 if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>1460&&AIR_R_LONG<1540)//��͵����е�
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_HIGH;//���м�����
										if(ROCK_R_Y>1850)//���ұ��͸�
										{
									ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
									ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
									ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
										//PUSH PUSH UPDATE ANGLE
									PUSH(0,-2000,5000,4000,0,0.4,0.5);//��Χ��1000-10000
										}
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP)
									{
										if(LaserLockPoint(ros_vx , ros_vy ,100,200))//��λcm
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}
									}
								}
					  if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>1850&&AIR_R_LONG<2100)//��͵��͵͵�
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_TH;//��Է�����
										if(ROCK_R_Y>1850)//���ұ��͸�
										{
									ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
									ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
									ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
										//PUSH PUSH UPDATE ANGLE
									PUSH(0,-2000,5000,4000,0,0.4,0.5);//��Χ��1000-10000
										}
						  
									//LOCK LockupPoint
										if(LaserLockPoint(ros_vx , ros_vy ,100,200))//��λcm
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}
										if(ROCK_L_Y>1850)
										{
											ROBOT_SHOOTING=SHOOT_WAITING;
										}
								}
					 

			     }	
		}
		if(AIR_R_SHORT>1500&&mode_flag==0)//�����Զ�ģʽ
		{
				MOVE_STATE=MOVE_STOP;
						if(MOVE_STATE==MOVE_STOP)
						{
							//�������
						}
			
			
		}
		if(mode_flag==2)//ROS�ص���
		{
			
			ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=ros_vx;
			ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=ros_vy;
			ROBOT_TARGET_VELOCITY_DATA.W_RPM=ros_vz;//��Ϊ�Ƕȣ���
			
		}
		move();
		vTaskDelay(5);
	}
	
	


}

