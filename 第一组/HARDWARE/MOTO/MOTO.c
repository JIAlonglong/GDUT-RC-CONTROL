#include "includeh.h"




M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[4] = {0}; // 1-4分别对应顺时针方向的底盘电机
M3508_REAL_INFO M3508_CAST_MOTOR_REAL_INFO 			 = {0};	// 射箭机构电机

PID M3508_CHASSIS_MOTOR_PID_RPM[4];	// 1-4底盘电机
PID M3508_CAST_MOTOR_PID_RPM;				



//M3580初始化
void M3508_Motor_Init(void)
{
	// 底盘电机PID初始化
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[0], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[1], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[2], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[3], 10.0, 1.0, 0.0, 16384, 16384, -1);
	
	// 抬升机构电机PID初始化
	PID_parameter_init(&M3508_CAST_MOTOR_PID_RPM , 50, 0, 1, 7000, 7000, 10);
}


// 利用电机通过CAN反馈的数据更新m3508的状态信息
// 接受频率：1kHz
void m3508_update_m3508_info(CanRxMsg *msg)
{
	switch(msg -> StdId)  // 检测标准ID
	{
    case M3508_CHASSIS_MOTOR_ID_1:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_2:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[1].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_3:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[2].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[2].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;	
		
		case M3508_CHASSIS_MOTOR_ID_4:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[3].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[3].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
		
		case M3508_CAST_MOTOR_ID:
		{ 
			M3508_CAST_MOTOR_REAL_INFO.ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CAST_MOTOR_REAL_INFO.RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CAST_MOTOR_REAL_INFO.CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;

		default: break;
	}
}



//发送电流
void chassis_m3508_send_motor_currents(void)
{
	CanTxMsg tx_message_1;

	// 配置控制段
	tx_message_1.IDE = CAN_Id_Standard;
	tx_message_1.RTR = CAN_RTR_Data;
	tx_message_1.DLC = 0x08;
	
	// 配置仲裁段和数据段	
	tx_message_1.StdId = 0x200;  // 用于ID为 1 2 3 4 的电机
	tx_message_1.Data[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	tx_message_1.Data[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT;
	tx_message_1.Data[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	tx_message_1.Data[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT;
	tx_message_1.Data[4] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	tx_message_1.Data[5] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT;
	tx_message_1.Data[6] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	tx_message_1.Data[7] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT;

	CAN_Transmit(CAN1, &tx_message_1);
}

//控制电流
void shoot_m3508_send_motor_currents(void)
{
	CanTxMsg tx_message_2;
	
	// 配置控制段
	tx_message_2.IDE = CAN_Id_Standard;//标准ID
	tx_message_2.RTR = CAN_RTR_Data; //数据帧
	tx_message_2.DLC = 0x08;
	
	// 配置仲裁段和数据段
	tx_message_2.StdId = 0x1FF;  // 用于ID为 5 6 7 8 的电机
	tx_message_2.Data[0] = (uint8_t)(M3508_CAST_MOTOR_REAL_INFO.TARGET_CURRENT >> 8);
	tx_message_2.Data[1] = (uint8_t) M3508_CAST_MOTOR_REAL_INFO.TARGET_CURRENT;	
	
	CAN_Transmit(CAN1, &tx_message_2);
}


//M3508电机角度积分
// M3508电机角度积分
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR)
{
	float delta_pos = 0;
	
	// 记录第一次进入时的数据
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// 计算变化的角度
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
			}
		}
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
		}
		
		// 滤波
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分	
	}
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
		}
		
		// 滤波
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分
	}

	// 存储角度值 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}
/**
* @brief  抬升机构目标角度
* @note		
* @param  target_angle目标角度
* @retval 未完成，返回1；完成，返回0
PID point_traker_x_pid;
PID point_traker_y_pid;
PID point_traker_yaw_pid;
*/
int	CAST_ANGLE(float target_angle)
{
	if((M3508_CHASSIS_MOTOR_REAL_INFO[3].REAL_ANGLE<(target_angle-0.5))||(M3508_CHASSIS_MOTOR_REAL_INFO[3].REAL_ANGLE>(target_angle+0.5)))
	{
			M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_ANGLE=target_angle;
		return 1;
	}
	else return 0;
}
     
///********特殊的、经过修改的M3508路径积分****************************************************/
//void M3508AngleIntegral_special_up(M3508_REAL_INFO *M3508_MOTOR)  
//{
//	float delta_pos = 0;
//	
//	// 记录第一次进入时的数据
//	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG||Robot_State.UP==BOTTOM||Robot_State.UP==TOP)
//	{
//		delta_pos = 0;
//		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
//		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
//		return;
//	}
//	
//	// 计算变化的角度
//	if(M3508_MOTOR->RPM >= 0)
//	{
//		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
//		{
//			if(abs(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波
//			{
//				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
//				delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
//			}
//		}
//		else
//		{
//			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
//			delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
//		}
//		
//		// 滤波
//		if(delta_pos > 0) 
//			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分	
//	}
//	else
//	{
//		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
//		{
//			if(abs(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
//			{
//				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
//				delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
//			}
//		}	
//		else
//		{
//			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
//			delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
//		}
//		
//		// 滤波
//		if(delta_pos < 0) 
//			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分
//	}

//	// 存储角度值
//	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
//}


