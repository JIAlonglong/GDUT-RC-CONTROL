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
					if(AIR_L_LONG>950&&AIR_L_LONG<1250&&AIR_R_LONG>950&&AIR_R_LONG<1250&&MOVE_STATE==MOVE_STOP)//��ߵ��͸ߵ�
								{
									ROBOT_SHOOTING=SHOOT_WAITING;
										MOVE_STATE=MOVE_STOP;
										Free_Control();//ң�����ɿ���
								}
					if(AIR_L_SHORT>1850&&AIR_L_SHORT<2100)//��߶̸���
			          {
										ROBOT_SHOOTING=SHOOT_INIT;//�������չ�� �񻷻�������
			          }
					 if(AIR_R_SHORT>1850&&AIR_R_SHORT<2100)//�ұ߶̸���
			          {
										ROBOT_SHOOTING=ROBOT_PICK;//��								
								}
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>950&&AIR_R_LONG<1100)//���е����ұ�һ��
			          {
										ROBOT_SHOOTING=ROBOT_SHOTH;		//�����
									if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}									
										if(ROCK_R_Y>1800)//���ұ��͸�
										{
											M3508_PICK_TRANSATE.REAL_INFO.REAL_ANGLE=0;
											ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
										//PUSH PUSH UPDATE ANGLE
											PUSH(0,-1450,9000,8000,0,0.4,0.5);//��Χ��1000-10000
										}
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP&&ROCK_L_Y<1600)
									{
										if(LaserLockPoint(ros_vx , ros_vy ,100,100))//��λcm
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}
								  }
								}
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>1460&&AIR_R_LONG<1540)//���е����ұ߶���
			          {
										ROBOT_SHOOTING=ROBOT_MIDDLE; // ������
                   if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}									
										if(ROCK_R_Y>1800)//���ұ��͸�
										{
											M3508_PICK_TRANSATE.REAL_INFO.REAL_ANGLE=0;
											ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
										//PUSH PUSH UPDATE ANGLE
											PUSH(0,-1450,9000,8000,0,0.4,0.5);//��Χ��1000-10000
										}
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP&&ROCK_L_Y<1600)
									{
										if(LaserLockPoint(ros_vx , ros_vy ,100,100))//��λcm
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}									
								  }
								}
						if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>1850&&AIR_R_LONG<2100)//���е����ұ�����
			          {
										ROBOT_SHOOTING=ROBOT_HIGH;   //����� 
									if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}									
										if(ROCK_R_Y>1800)//���ұ��͸�
										{
											M3508_PICK_TRANSATE.REAL_INFO.REAL_ANGLE=0;
											ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
										//PUSH PUSH UPDATE ANGLE
											PUSH(0,-1450,9000,8000,0,0.4,0.5);//��Χ��1000-10000
										}
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP&&ROCK_L_Y<1600)
									{
										if(LaserLockPoint(ros_vx , ros_vy ,100,100))//��λcm
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}
			             }
				
								 }
			     }	
		}
		if(AIR_R_SHORT>1500&&mode_flag==0)//�����Զ�ģʽ
		{
				MOVE_STATE=MOVE_1_CAST_POINT;
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
	
	





