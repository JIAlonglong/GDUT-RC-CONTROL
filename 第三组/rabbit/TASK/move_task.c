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
					if(AIR_L_SHORT>1850&&AIR_L_SHORT<2100)//左边短杆下
			          {
										ROBOT_SHOOTING=SHOOT_INIT;//发射机构展开 捡环机构落下
			          }
					 if(AIR_R_SHORT>1850&&AIR_R_SHORT<2100)//右边短杆下
			          {
										ROBOT_SHOOTING=ROBOT_PICK;//捡环								
								}
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>950&&AIR_R_LONG<1100)//打中档和右边一档
			          {
										ROBOT_SHOOTING=ROBOT_SHOTH;		//射低柱
									if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}									
										if(ROCK_R_Y>1800)//拉右边油杆
										{
											M3508_PICK_TRANSATE.REAL_INFO.REAL_ANGLE=0;
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
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>1460&&AIR_R_LONG<1540)//打中档和右边二档
			          {
										ROBOT_SHOOTING=ROBOT_MIDDLE; // 射中柱
                   if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}									
										if(ROCK_R_Y>1800)//拉右边油杆
										{
											M3508_PICK_TRANSATE.REAL_INFO.REAL_ANGLE=0;
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
						if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>1850&&AIR_R_LONG<2100)//打中档和右边三档
			          {
										ROBOT_SHOOTING=ROBOT_HIGH;   //射高柱 
									if(MOVE_STATE==MOVE_STOP)
									{
											Free_Control_Limit();
									}									
										if(ROCK_R_Y>1800)//拉右边油杆
										{
											M3508_PICK_TRANSATE.REAL_INFO.REAL_ANGLE=0;
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
	
	





