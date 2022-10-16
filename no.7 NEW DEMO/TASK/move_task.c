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
					if(AIR_L_LONG>950&&AIR_L_LONG<1250&&AIR_R_LONG>950&&AIR_R_LONG<1250)//打高档和高档
								{
									Free_Control();//遥控自由控制
								}
					if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>800&&AIR_R_LONG<1200)//打中档和高档
			          {
									int ctrl_flag=0;
			            if(ctrl_flag==0)
									{
										MOVE_STATE=MOVE_1_SHOOT;//到达1号射环点/point to point
										ctrl_flag+=1;
									}
									else
                  {
										MOVE_STATE=MOVE_STOP;
										if(MOVE_STATE==MOVE_STOP)
										{
											//激光矫正
										}
									}
			          }
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>1460&&AIR_R_LONG<1540)//打中档和中档
			          {
									int ctrl_flag=0;
			            if(ctrl_flag==0)
									{
										MOVE_STATE=MOVE_2_SHOOT;//到达2号射环点/point to point
										ctrl_flag+=1;
									}
									else
                  {
										MOVE_STATE=MOVE_STOP;
										if(MOVE_STATE==MOVE_STOP)
										{
											//激光矫正
										}
									}	
			          }
					 if(AIR_L_LONG>1460&&AIR_L_LONG<1540&&AIR_R_LONG>1850&&AIR_R_LONG<2100)//打中档和低档
			          {
									int ctrl_flag=0;
			            if(ctrl_flag==0)
									{
										MOVE_STATE=MOVE_3_SHOOT;//到达3号射环点/point to point
										ctrl_flag+=1;
									}
									else
                  {
										MOVE_STATE=MOVE_STOP;
										if(MOVE_STATE==MOVE_STOP)
										{
											//激光矫正
										}
									}
			          }
			        if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>800&&AIR_R_LONG<1200)//打低档和高档
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_OUR;//射我方柱子
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP)
									{
										LaserLockPoint(ros_vx , ros_vy ,100,200);//单位cm
									}
								}
					 if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>1460&&AIR_R_LONG<1540)//打低档和中档
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_HIGH;//射中间柱子
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP)
									{
										LaserLockPoint(ros_vx , ros_vy ,100,200);//单位cm
									}
								}
					  if(AIR_L_LONG<2100&&AIR_L_LONG>1850&&AIR_R_LONG>1850&&AIR_R_LONG<2100)//打低档和低档
			          {
			            ROBOT_SHOOTING=ROBOT_SHOOT_TH;//射对方柱子
									//LOCK LockupPoint
									if(MOVE_STATE==MOVE_STOP)
									{
										LaserLockPoint(ros_vx , ros_vy ,100,200);//单位cm
									}
								}
				      if(ROCK_R_Y>1850)//拉右边油杆
				      {
						ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
						ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
						ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
					    //PUSH
				      }
					 

			     }	
		}
		if(AIR_R_SHORT>1500&&mode_flag==0)//进入自动模式
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
							//激光矫正
						}
			}
			
		}
		if(mode_flag==2)//ROS控底盘
		{
			
			ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=ros_vx;
			ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=ros_vy;
			ROBOT_TARGET_VELOCITY_DATA.W_RPM=ros_vz;//改为角度？？
			
		}
		usartSendData(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM, ROBOT_TARGET_VELOCITY_DATA.Vy_RPM,ROBOT_TARGET_VELOCITY_DATA.W_RPM,ROBOT_REAL_POS_DATA.POS_X,ROBOT_REAL_POS_DATA.POS_Y,ROBOT_REAL_POS_DATA.POS_YAW,rosctrl_flag);
		vTaskDelay(13);
		move();
		vTaskDelay(5);
	}
	
	


}