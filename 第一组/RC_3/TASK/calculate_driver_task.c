#include "includeh.h"
//#include "MOTO.h"
//ChassisDrive任务
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

void ChassisDrive_task(void *pvParameters)
{	
	while(1)
	{

		   if(SWA>1500&&SWD<1500)
		// 速度分解
		{
		ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1500)*2;
		ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1500)*2;
		ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_Y-1500)*2;
			
		ROBOT_REAL_POS_DATA.POS_YAW=0;
			fine_turning_state=0;
		}
    if(fine_turning_state==1)
		{
		ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1500)/50;
		ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1500)/50;
		ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_Y-1500)/10;
			
		ROBOT_REAL_POS_DATA.POS_YAW=0;
		}
		if(SWA>1500&&SWD>1500)
		{
	 LaserLockPoint(ladar_distance ,ladar_yaw,800,500);
		}
		
//	 usartReceiveOneData(&a1,&b1,&crtlFlag);
//	 usartSendData(testSend1,testSend2,testSend3,testSend4);

		//三轮世界坐标
		world_3wheel(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM,
									ROBOT_TARGET_VELOCITY_DATA.Vy_RPM, 
									ROBOT_TARGET_VELOCITY_DATA.W_RPM,
									ROBOT_REAL_POS_DATA.POS_YAW);
		

//			CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM=1000;
		

		// 位置式PID
// PID_position_PID_calculation(&M3508_CAST_MOTOR_PID_RPM,M3508_CHASSIS_MOTOR_REAL_INFO[0].REAL_ANGLE,M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT)  ;
		// 进行PID计算
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[0], M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM, CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM );
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[1], M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM, CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM);
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[2], M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM, CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM);
//		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[3], M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM, CHASSIS_MOTOR_TARGET_RPM.MOTOR4_RPM);
		
		// 设置电流
		M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[0].output;
		M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[1].output;
		M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[2].output;
//		M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[3].output;
		
		// 发送数据
		chassis_m3508_send_motor_currents();
		
		vTaskDelay(5);
	}

}
