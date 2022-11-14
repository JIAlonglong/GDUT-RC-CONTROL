#include "includeh.h"
//#include "MOTO.h"
//ChassisDrive任务
extern SHOOTING_STATE SHOOT_STATE ;
extern short testSend1   ;
extern short testSend2   ;
extern short testSend3   ;
extern unsigned char testSend4 ;
extern int ladar_distance;
extern int ladar_yaw;

extern int a1;
extern int b1;
extern unsigned char crtlFlag;
extern int unautomatic;
extern int fine_turning_state;
int ok=0;

void ChassisDrive_task(void *pvParameters)
{	
	
	while(1)
 
	{
		
		
		   if(SWA>1500&&SWD<1500)
		
		{
		if(ROCK_L_X>1460&&ROCK_L_X<1540)
			ROCK_L_X=1500;
			ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1500)*2;
		 if(ROCK_L_Y>1460&&ROCK_L_Y<1540)
			ROCK_L_Y=1500; 
		 ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1500)*2;
		 if(ROCK_R_X>1460&&ROCK_R_X<1540)
			ROCK_R_X=1500; 
		 ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_X-1500)*2;
		 
	   if(ROCK_R_Y<1300)M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_ANGLE+=10;	
		 if(ROCK_R_Y>1700)M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_ANGLE-=10;	
			
		
		ROBOT_REAL_POS_DATA.POS_YAW=0;
			fine_turning_state=0;
		}
    if(fine_turning_state==1)
		{
		if(ROCK_L_X>1460&&ROCK_L_X<1540)
			ROCK_L_X=1500;
			ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1500)/5;
		 if(ROCK_L_Y>1460&&ROCK_L_Y<1540)
			ROCK_L_Y=1500; 
		 ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1500)/5;
		 if(ROCK_R_X>1460&&ROCK_R_X<1540)
			ROCK_R_X=1500; 
		 ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_X-1500)/5;
		 
	   if(ROCK_R_Y<1300)M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_ANGLE+=5;	
		 if(ROCK_R_Y>1700)M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_ANGLE-=5;	
			
		
		ROBOT_REAL_POS_DATA.POS_YAW=0;
		}
	//雷达锁定 
	
		if(SWA>1500&&(SWB<1200)&&ok==0)
		{SHOOT_STATE=SHOOTING_1;
	    if(LaserLockPoint(ladar_distance ,ladar_yaw,600,500)&&(-1<ladar_yaw&&ladar_yaw<1))
			{
					testSend1=0;
				testSend2=0;
				testSend3=0;
				ok=1;
				fine_turning_state=1;		
//				SHOOT_STATE=SHOOTING_1;
			}
			
		}
	if(SWA>1500&&SWB>1600&&SWC<1500&&ok==0)
	{
	  if(LaserLockPoint(ladar_distance ,ladar_yaw,3100,500))
			{
				SHOOT_STATE=SHOOTING_2;
				fine_turning_state=1;
				testSend1=1;
				testSend2=1;
				testSend3=1;
				ok=1;
			}
			
			
	}
	
	if(SWA>1500&&SWB>1600&&SWC>1500&&ok==0)
	{
	  if(LaserLockPoint(ladar_distance ,ladar_yaw,3100,500))
			{
				SHOOT_STATE=SHOOTING_3;
				fine_turning_state=1;
				testSend1=1;
				testSend2=1;
				testSend3=1;
				ok=1;
			}
			
			
	}
		if(SWB>1200&&SWB<1700)
		{
			ok=0;
		}
		
			
//	 usartReceiveOneData(&a1,&b1,&crtlFlag);
//	 usartSendData(testSend1,testSend2,testSend3,testSend4);

		//三轮世界坐标速度分解
		world_3wheel(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM,
									ROBOT_TARGET_VELOCITY_DATA.Vy_RPM, 
									ROBOT_TARGET_VELOCITY_DATA.W_RPM,
									ROBOT_REAL_POS_DATA.POS_YAW);
		

//			CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM=1000;
		

		// 位置式PID
 //   PID_position_PID_calculation(&M3508_CAST_MOTOR_PID_RPM,M3508_CHASSIS_MOTOR_REAL_INFO[3].REAL_ANGLE,M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_ANGLE)  ;

		PID_position_PID_calculation(&M3508_CAST_MOTOR_PID_RPM,M3508_CHASSIS_MOTOR_REAL_INFO[3].REAL_ANGLE,M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_ANGLE)  ;
		// 进行PID计算
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[0], M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM, CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM );
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[1], M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM, CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM);
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[2], M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM, CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM);
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[3], M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM, M3508_CAST_MOTOR_PID_RPM.output);
		
		// 设置电流
		M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[0].output;
		M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[1].output;
		M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[2].output;
		M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[3].output;
		
		// 发送数据
		chassis_m3508_send_motor_currents();
		 AK80_Speed_Control(AK80_ID1,MIT_DRIVER_REAL_INFO[0].TARGET_SPEED);
			delay_ms(1);
		AK80_Speed_Control(AK80_ID2,MIT_DRIVER_REAL_INFO[1].TARGET_SPEED);
				delay_ms(1);
		AK80_Speed_Control(AK80_ID3,MIT_DRIVER_REAL_INFO[2].TARGET_SPEED);
				delay_ms(1);
		AK80_Speed_Control(AK80_ID4,MIT_DRIVER_REAL_INFO[3].TARGET_SPEED);
				delay_ms(1);
		
usartSendData(testSend1 ,testSend2 ,testSend3 ,0x05);
		
		vTaskDelay(10);
	}

}
