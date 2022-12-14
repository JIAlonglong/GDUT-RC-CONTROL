#include "includes.h"

void move_task(void *pvParameters)
{
	while(1)
	{	if(AIR_R_SHORT<1500&&mode_flag==0)//进入正常模式
		{
				if(ROCK_L_Y==0&&ROCK_L_X==0&&ROCK_R_X==0)//安全锁
				{
						ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
						ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
						ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
				}
		       else     
				{
					if(AIR_L_LONG>950&&AIR_L_LONG<1250&&AIR_R_LONG>950&&AIR_R_LONG<1250&&MOVE_STATE==MOVE_STOP)//打高档和高档
								{
									ROBOT_SHOOTING=SHOOT_WAITING;
										MOVE_STATE=MOVE_STOP;
										Free_Control();//遥控自由控制
								}
					if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_L_X>950&&ROCK_L_X<1100)//打中档和左边左
			          {
										ROBOT_SHOOTING=SHOOT_WAITING;
										MOVE_STATE=MOVE_1_RESTART;//取环点
			          }
					  if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_L_Y>1850&&ROCK_L_Y<2150)//打中档和左边上
			          {
										MOVE_STATE=MOVE_1_SHOOT;
											v=0;
											th=0;									
								}
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_L_X>1850&&ROCK_L_X<2150)//打中档和左边右
			          {
										MOVE_STATE=MOVE_2_SHOOT;
										v=0;
										th=0;
								}
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_L_Y>950&&ROCK_L_Y<1100)//打中档和左边下
			          {
										MOVE_STATE=MOVE_3_SHOOT;
											v=0;
											th=0;
								}
						if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_R_X>950&&ROCK_R_X<1100)//打中档和右边左 up==4(发射都是打中档)
			          {
										MOVE_STATE=MOVE_JI;//取环点
											up_flag=4;
											v=1;
											th=1;
			          }
						if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_R_Y>1850&&ROCK_R_Y<2150)//打中档和右边上 up==1 (发射都是打中档)
			          {
										MOVE_STATE=MOVE_1_SHOOT_CLOSE;
											up_flag=1;
											v=1;
											th=0;
			          }
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_R_X>1850&&ROCK_R_X<2150)//打中档和右边右 up==5 (发射都是打中档)
			          {
										MOVE_STATE=MOVE_2_SHOOT_CLOSE;
											up_flag=5;
											v=1;
											th=0;
			          }
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&ROCK_R_Y>950&&ROCK_R_Y<1100)//打中档和右边下 up==6 (发射都是打中档)
			          {
										MOVE_STATE=MOVE_3_SHOOT_CLOSE;
											up_flag=6;
											v=1;
											th=1;
			          }
			        if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>800&&AIR_R_LONG<1200)//打低档和高档
			          {
									MOVE_STATE==MOVE_STOP;
			            ROBOT_SHOOTING=ROBOT_SHOOT_OUR;//射我方柱子(GET)
									if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}
										if(ROCK_R_Y>1800)//拉右边油杆
									{
										M3508_TRANSATE.REAL_INFO.REAL_ANGLE=0;
											ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
													//PUSH PUSH UPDATE ANGLE
											PUSH(0,-1480,5000,4000,0,0.4,0.5);//范围：1000-10000
									}
									//LOCK LockupPoint
								if(MOVE_STATE==MOVE_STOP&&ROCK_L_Y<1600)
							{
										if(LaserLockPoint(ros_vx , ros_vy ,100,100))//单位cm   106
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}
									}			
								}
					 if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>1460&&AIR_R_LONG<1540)//打低档和中档
			          {
									MOVE_STATE==MOVE_STOP;
			            ROBOT_SHOOTING=ROBOT_SHOOT_HIGH;//射中间柱子(62 GET)
									if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}									
										if(ROCK_R_Y>1800)//拉右边油杆
										{
											M3508_TRANSATE.REAL_INFO.REAL_ANGLE=0;
											ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
										//PUSH PUSH UPDATE ANGLE
											PUSH(0,-1450,9000,8000,0,0.4,0.5);//范围：1000-10000
										}
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP&&ROCK_L_Y<1600)
									{
										if(LaserLockPoint(ros_vx , ros_vy ,100,100))//单位cm
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}
									}	
								}
					  if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>1850&&AIR_R_LONG<2100)//打低档和低档
			          {
									MOVE_STATE==MOVE_STOP;
			            ROBOT_SHOOTING=ROBOT_SHOOT_TH;//射对方柱子
									if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}
										if(ROCK_R_Y>1800)//拉右边油杆
										{
										M3508_TRANSATE.REAL_INFO.REAL_ANGLE=0;
											ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
											ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
										//PUSH PUSH UPDATE ANGLE
											PUSH(0,-1450,9000,8000,0,0.7,0.3);//范围：1000-10000
										}
						  
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP&&ROCK_L_Y<1600)
									{
										if(LaserLockPoint(ros_vx , ros_vy ,100,100))//单位cm
										{
											//Jiguang_Action_Update(0.0,0.0);
										
										}
									}
								}
					 

			     }	
		}
		if(AIR_R_SHORT>1500&&mode_flag==0)//进入自动模式
		{
				MOVE_STATE=MOVE_1_CAST_POINT;
						if(MOVE_STATE==MOVE_STOP)
						{
							//激光矫正
						}
			
			
		}
		if(mode_flag==2)//ROS控底盘
		{
			
			ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=ros_vx;
			ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=ros_vy;
			ROBOT_TARGET_VELOCITY_DATA.W_RPM=ros_vz;//改为角度？？
			
		}
		move();
		vTaskDelay(5);
	}

}
	
	





