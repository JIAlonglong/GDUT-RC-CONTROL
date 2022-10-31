#include "includes.h"

/*********************
 velocity 机器人的速度 
 speed    电机轴rpm 
 RPM      电机转子rpm
 *********************/
int start=0;
int end=0;
M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[3] = {0};

PID M3508_CHASSIS_MOTOR_PID_RPM[3];	// 3个M3508电机
PID M3508_CAST_MOTOR_PID_RPM;	


//抬升机构和云台机构
int16_t UP_MOTOR_TARGET_RPM ;     // 抬升电机目标速度 
int16_t YAW_MOTOR_TARGET_RPM ;    // 云台电机目标速度
int16_t TRANSATE_MOTOR_TARGET_RPM;

M3508_CLASS M3508_UP;//抬升电机初始化 CANid为5//是一个类里面有 	M3508_REAL_INFO REAL_INFO; PID MOTOR_PID;	int TARGET_RPM;		int SET_CURRENT;
M3508_CLASS M3508_YAW;//云台电机初始化 CANid为6
M3508_CLASS M3508_TRANSATE;//传递电机初始化 CANid为7


ARM_VELOCITY_PLANNING  *UP_ARM_NOW_MOTION;		 // 指向抬升当前动作
//								  开始位置  结束位置    开始的速度(RPM 绝对值)  最大的速度	 末尾的速度   加速路程的比例 减速路程的比例
ARM_VELOCITY_PLANNING   UP_INIT={0};//实验用
ARM_VELOCITY_PLANNING   UP_ON1	 ={0,        -4000,          2500,                3500,            0,          0.4,         0.3};
ARM_VELOCITY_PLANNING   UP_ON2	 ={-4000,        -8000,          2500,                3500,            0,          0.4,         0.3};
ARM_VELOCITY_PLANNING   UP_ON3	 ={-8000,       -10000,         2500,             3500,            0,          0.4,         0.2};
ARM_VELOCITY_PLANNING   UP_DOWN3={-10000,           0,         2500,             3500,            0,          0.2,         0.3};
	

ARM_VELOCITY_PLANNING  *YAW_ARM_NOW_MOTION;		 // 指向云台当前动作
//								  开始位置  结束位置    开始的速度(RPM 绝对值)  最大的速度	 末尾的速度   加速路程的比例 减速路程的比例	
ARM_VELOCITY_PLANNING   YAW_INIT={0};
ARM_VELOCITY_PLANNING   YAW_MOVE_HANDLE={0};	
ARM_VELOCITY_PLANNING   YAW_MOVE_AUTO={0};
ARM_VELOCITY_PLANNING   YAW_MOVE_1={0};
ARM_VELOCITY_PLANNING   YAW_MOVE_2={0};
ARM_VELOCITY_PLANNING   YAW_MOVE_3={0};	

TRANSATE_VELOCITY_PLANNING *TRANSATE_NOW_MOTION; //指向传递电机的当前动作
//							总路程	 开始位置  结束位置    开始的速度(RPM 绝对值)  最大的速度	 末尾的速度   加速路程的比例 减速路程的比例	
TRANSATE_VELOCITY_PLANNING TRANSATE_INIT={0};
TRANSATE_VELOCITY_PLANNING TRANSATE_1={2000,0 , 0	,0		,0		,0		,0		,0};//0 , 2000		,1200		,5000		,0		,0.2		,0.4
/*float distance;       //距离
  float startSpeed;    //开始调速时的初始速度
  float currentSpeed;   //当前速度
  float targetSpeed;    //目标速度
  float stepSpeed;     //加速度
  float speedMax;     //最大速度
  float speedMin;     //最小速度
  uint32_t aTimes;     //调速时间
  uint32_t maxTimes;    //调速跨度
	float  p_add;    //加速的占比
	float  p_decrease; //减速的占比
*/
CurveObjectType TRANSATE={2000,0,0,500,30,600,0,1000,0,0.3,0.7};
// M3508初始化
void M3508_Motor_Init(void)
{
	// 三轮底盘电机速度环PID初始化
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[0], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[1], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[2], 10.0, 1.0, 0.0, 16384, 16384, -1);	
	
	//要角度控制的电机
	PID_parameter_init(&M3508_UP.MOTOR_PID, 10.0, 1.0, 0.0, 16384, 16384,-0.1);
	PID_parameter_init(&M3508_YAW.MOTOR_PID, 10.0, 1.0, 0.0, 16384, 16384,-0.1);
	//需要s型规划的电机
	PID_parameter_init(&M3508_TRANSATE.MOTOR_PID, 10.0, 1.0, 0.0, 16384, 16384,-0.1);
	
	/****LADRC****/
	//三轮底盘
	LADRC_Init(&ADRC_M3508_CHASIS[0],0.005,20,100,400,0.5,16384, -1);
	LADRC_Init(&ADRC_M3508_CHASIS[1],0.005,20,100,400,0.5,16384,  -1);
	LADRC_Init(&ADRC_M3508_CHASIS[2],0.005,20,100,400,0.5,16384,  -1);
	//要角度控制的电机
	LADRC_Init(&ADRC_M3508_UP,0.005,20,100,400,0.5,16384, -1);
	//需要s型规划的电机
	LADRC_Init(&ADRC_M3508_TRANSATE,0.005,20,100,400,0.5,16384, -1);
	
}


//can中断处理过程
// 利用电机通过CAN反馈的数据更新m3508的状态信息
// 接受频率：1kHz
void m3508_update_m3508_info_can1(CanRxMsg *msg)
{
	switch(msg -> StdId)  // 检测标准ID
	{
    case M3508_CHASSIS_MOTOR_ID_1://A轮子
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
			M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[0]);
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_2://B轮子
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[1].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
			M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[1]);
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_3://C轮子
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[2].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[2].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
			M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[2]);
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_4://D轮子
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[3].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[3].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
			M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[3]);
		}; break;

		case M3508_UP_MOTOR_ID_5://抬升机构
		{
			M3508_UP.REAL_INFO.ANGLE    =(msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_UP.REAL_INFO.RPM      =(msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_UP.REAL_INFO.CURRENT  =(msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
			M3508AngleIntegral(&M3508_UP.REAL_INFO);
			
		};break;
		case M3508_YAW_MOTOR_ID_6://云台机构
		{
			M3508_YAW.REAL_INFO.ANGLE    =(msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_YAW.REAL_INFO.RPM      =(msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_YAW.REAL_INFO.CURRENT  =(msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
			M3508AngleIntegral(&M3508_YAW.REAL_INFO);
		};break;
		case M3508_TRANSATE_MOTOR_ID_7://传输机构
		{
			M3508_TRANSATE.REAL_INFO.ANGLE    =(msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_TRANSATE.REAL_INFO.RPM      =(msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_TRANSATE.REAL_INFO.CURRENT  =(msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
			M3508AngleIntegral(&M3508_TRANSATE.REAL_INFO);
		};break;	
		default: break;
	}

}


// 设置3508的电流大小
void chassis_m3508_send_motor_currents_can1(void)
{
	CanTxMsg tx_message_1;

	// 配置控制段
	tx_message_1.IDE = CAN_Id_Standard;
	tx_message_1.RTR = CAN_RTR_Data;
	tx_message_1.DLC = 0x08;//0x02代表一个电机
	
	// 配置仲裁段和数据段	
	tx_message_1.StdId = 0x200;  // 用于ID为 1 2 3  的电机
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

void chassis_m3508_m2006_send_motor_currents_can1(void)
{
	CanTxMsg tx_message_2;
	
	// 配置控制段
	tx_message_2.IDE = CAN_Id_Standard;
	tx_message_2.RTR = CAN_RTR_Data;
	tx_message_2.DLC = 0x08;
	
	// 配置仲裁段和数据段
	tx_message_2.StdId = 0x1FF;  // 用于ID为 5 6 7 8 的电机(不论是3508还是2006)
	tx_message_2.Data[0] = (uint8_t)(M3508_UP.REAL_INFO.TARGET_CURRENT >> 8);
	tx_message_2.Data[1] = (uint8_t) M3508_UP.REAL_INFO.TARGET_CURRENT;
  tx_message_2.Data[2] = (uint8_t)(M3508_YAW.REAL_INFO.TARGET_CURRENT >> 8);
  tx_message_2.Data[3] = (uint8_t) M3508_YAW.REAL_INFO.TARGET_CURRENT;
  tx_message_2.Data[4] = (uint8_t)(M3508_TRANSATE.REAL_INFO.TARGET_CURRENT >> 8);
  tx_message_2.Data[5] = (uint8_t)(M3508_TRANSATE.REAL_INFO.TARGET_CURRENT);
	
	CAN_Transmit(CAN1, &tx_message_2);
}

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
int up_finished;
// 规划抬升机构应有的RPM
//								*ARM_NOW_MOTION 	M3508_ARM_MOTOR_REAL_INFO.REAL_ANGLE[处理过的真实角度]
void ad_plan_arm_motor_RPM_UP(ARM_VELOCITY_PLANNING motion, 							float pos			)	
{
	float Ssu;   //总路程
	float Sac;   //加速路程
	float Sde;   //减速路程
	float Sco;   //匀速路程
	float Aac;   //加速加速度
	float Ade;   //减速加速度
	float S;     //当前路程
	up_finished=0;
	// 如果所配数据有误，则不执行速度规划		
	if((motion.Rac > 1) || (motion.Rac < 0) ||		//加速路程的比例
		 (motion.Rde > 1) || (motion.Rde < 0) ||	//减速路程的比例
		 (motion.Vmax < motion.Vstart) )			//最大的速度<开始的速度 
	{
		UP_MOTOR_TARGET_RPM = 0;  // 令抬升机构不运动
		return;
	}
	// 匀速模式
	if(motion.Pstart == motion.Pend)	//开始位置=结束位置
	{
		UP_MOTOR_TARGET_RPM = motion.Vstart * motion.Vmax;	//开始的速度*最大的速度
		return;
	}
	
	// 计算一些变量
	Ssu = ABS(motion.Pend - motion.Pstart); 	//总路程   
	Sac = Ssu * motion.Rac;		//加速路程 =	总路程 * 加速路程的比例
	Sde = Ssu * motion.Rde;		//减速路程 =	总路程 * 减速路程的比例
	Sco = Ssu - Sac - Sde;		//匀速路程 = 总路程 - 加速路程 - 减速路程
	Aac = (motion.Vmax * motion.Vmax - motion.Vstart * motion.Vstart) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade = (motion.Vend * motion.Vend -   motion.Vmax *   motion.Vmax) / (2.0f * Sde);	
	
	// 过滤异常情况
	if(((motion.Pend > motion.Pstart) && (pos < motion.Pstart)) ||		//[(结束位置 > 开始位置) && (处理过的真实角度pos <开始位置)]	||
		 ((motion.Pend < motion.Pstart) && (pos > motion.Pstart)))		//	[(结束位置 < 开始位置) && (处理过的真实角度pos >开始位置)]
	{
		UP_MOTOR_TARGET_RPM = motion.Vstart;	//TARGET_RPM = 开始的速度
	}
	else if(((motion.Pend > motion.Pstart) && (pos > motion.Pend)) ||
		      ((motion.Pend < motion.Pstart) && (pos < motion.Pend)))
	{
		UP_MOTOR_TARGET_RPM = motion.Vend;	//TARGET_RPM = 末尾的速度
	}
	else
	{
		S = ABS(pos - motion.Pstart);      //开始位置
		
		// 规划RPM
		if     (S < Sac)       UP_MOTOR_TARGET_RPM = sqrt(2.0f * Aac * S + motion.Vstart * motion.Vstart);               // 加速阶段
		else if(S < (Sac+Sco)) UP_MOTOR_TARGET_RPM = motion.Vmax;                                                        // 匀速阶段
		else                   UP_MOTOR_TARGET_RPM = sqrt(motion.Vend * motion.Vend - 2.0f * Ade * ABS(Ssu - S));  // 减速阶段
	}
	
	// 分配合适的正负号
	if(motion.Pend < motion.Pstart) UP_MOTOR_TARGET_RPM = -UP_MOTOR_TARGET_RPM;
	//pid
	PID_incremental_PID_calculation(&M3508_UP.MOTOR_PID, M3508_UP.REAL_INFO.RPM ,UP_MOTOR_TARGET_RPM);
	M3508_UP.REAL_INFO.TARGET_CURRENT = M3508_UP.MOTOR_PID.output;
	
	if(ABS(S-Ssu)<0.0001){up_finished=1;}
	//LADRC
//	LADRC_Loop(&ADRC_M3508_UP,M3508_UP.REAL_INFO.RPM ,UP_MOTOR_TARGET_RPM);
//	M3508_UP.REAL_INFO.TARGET_CURRENT=ADRC_M3508_UP.u;
}

// 规划云台电机应有的RPM
//								*ARM_NOW_MOTION 	M3508_ARM_MOTOR_REAL_INFO.REAL_ANGLE[处理过的真实角度]
void ad_plan_arm_motor_RPM_YAW(ARM_VELOCITY_PLANNING motion, 							float pos			)	
{
	float Ssu;   //总路程
	float Sac;   //加速路程
	float Sde;   //减速路程
	float Sco;   //匀速路程
	float Aac;   //加速加速度
	float Ade;   //减速加速度
	float S;     //当前路程
	
	// 如果所配数据有误，则不执行速度规划		
	if((motion.Rac > 1) || (motion.Rac < 0) ||		//加速路程的比例
		 (motion.Rde > 1) || (motion.Rde < 0) ||	//减速路程的比例
		 (motion.Vmax < motion.Vstart) )			//最大的速度<开始的速度 
	{
		YAW_MOTOR_TARGET_RPM = 0;  // 令夹爪不运动
		return;
	}
	// 匀速模式
	if(motion.Pstart == motion.Pend)	//开始位置=结束位置
	{
		YAW_MOTOR_TARGET_RPM = motion.Vstart * motion.Vmax;	//开始的速度*最大的速度
		return;
	}
	
	// 计算一些变量
	Ssu = ABS(motion.Pend - motion.Pstart); 	//总路程   
	Sac = Ssu * motion.Rac;		//加速路程 =	总路程 * 加速路程的比例
	Sde = Ssu * motion.Rde;		//减速路程 =	总路程 * 减速路程的比例
	Sco = Ssu - Sac - Sde;		//匀速路程 = 总路程 - 加速路程 - 减速路程
	Aac = (motion.Vmax * motion.Vmax - motion.Vstart * motion.Vstart) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade = (motion.Vend * motion.Vend -   motion.Vmax *   motion.Vmax) / (2.0f * Sde);	
	
	// 过滤异常情况
	if(((motion.Pend > motion.Pstart) && (pos < motion.Pstart)) ||		//[(结束位置 > 开始位置) && (处理过的真实角度pos <开始位置)]	||
		 ((motion.Pend < motion.Pstart) && (pos > motion.Pstart)))		//	[(结束位置 < 开始位置) && (处理过的真实角度pos >开始位置)]
	{
		YAW_MOTOR_TARGET_RPM = motion.Vstart;	//TARGET_RPM = 开始的速度
	}
	else if(((motion.Pend > motion.Pstart) && (pos > motion.Pend)) ||
		      ((motion.Pend < motion.Pstart) && (pos < motion.Pend)))
	{
		YAW_MOTOR_TARGET_RPM = motion.Vend;	//TARGET_RPM = 末尾的速度
	}
	else
	{
		S = ABS(pos - motion.Pstart);      //开始位置
		
		// 规划RPM
		if     (S < Sac)       YAW_MOTOR_TARGET_RPM = sqrt(2.0f * Aac * S + motion.Vstart * motion.Vstart);               // 加速阶段
		else if(S < (Sac+Sco)) YAW_MOTOR_TARGET_RPM = motion.Vmax;                                                        // 匀速阶段
		else                   YAW_MOTOR_TARGET_RPM = sqrt(motion.Vend * motion.Vend - 2.0f * Ade * ABS(Ssu - S));  // 减速阶段
	}
	
	// 分配合适的正负号
	if(motion.Pend < motion.Pstart) YAW_MOTOR_TARGET_RPM = -YAW_MOTOR_TARGET_RPM;
	//pid
//	PID_incremental_PID_calculation(&M3508_YAW.MOTOR_PID, M3508_YAW.REAL_INFO.RPM ,YAW_MOTOR_TARGET_RPM);
//	M3508_YAW.REAL_INFO.TARGET_CURRENT = M3508_YAW.MOTOR_PID.output;
	//LADRC
//	LADRC_Loop(&ADRC_M3508_YAW,M3508_YAW.REAL_INFO.RPM ,YAW_MOTOR_TARGET_RPM);
//	M3508_YAW.REAL_INFO.TARGET_CURRENT=ADRC_M3508_YAW.u;
}

// 规划传递电机应有的RPM
//								*ARM_NOW_MOTION 	M3508_ARM_MOTOR_REAL_INFO.REAL_ANGLE[处理过的真实角度]
int transate_finished;
void ad_plan_arm_motor_RPM_TRANSATE1(TRANSATE_VELOCITY_PLANNING motion, 							float pos			)	
{
	float Ssu;   //总路程
	float Sac;   //加速路程
	float Sde;   //减速路程
	float Sco;   //匀速路程
	float Aac;   //加速加速度
	float Ade;   //减速加速度
	float S;     //当前路程
	
	transate_finished=0;
	// 如果所配数据有误，则不执行速度规划		
	if((motion.Rac > 1) || (motion.Rac < 0) ||		//加速路程的比例
		 (motion.Rde > 1) || (motion.Rde < 0) ||	//减速路程的比例
		 (motion.Vmax < motion.Vstart) )			//最大的速度<开始的速度 
	{
		TRANSATE_MOTOR_TARGET_RPM = 0;  // 令抬升机构不运动
		return ;
	}
	// 匀速模式
	if(motion.Pstart == motion.Pend)	//开始位置=结束位置
	{
		TRANSATE_MOTOR_TARGET_RPM = motion.Vstart * motion.Vmax;	//开始的速度*最大的速度
		return ;
	}
	
	// 计算一些变量
	Ssu = ABS(motion.Pend - motion.Pstart); 	//总路程   
	Sac = Ssu * motion.Rac;		//加速路程 =	总路程 * 加速路程的比例
	Sde = Ssu * motion.Rde;		//减速路程 =	总路程 * 减速路程的比例
	Sco = Ssu - Sac - Sde;		//匀速路程 = 总路程 - 加速路程 - 减速路程
	Aac = (motion.Vmax * motion.Vmax - motion.Vstart * motion.Vstart) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade = (motion.Vend * motion.Vend -   motion.Vmax *   motion.Vmax) / (2.0f * Sde);	
	
	// 过滤异常情况
	if(((motion.Pend > motion.Pstart) && (pos < motion.Pstart)) ||		//[(结束位置 > 开始位置) && (处理过的真实角度pos <开始位置)]	||
		 ((motion.Pend < motion.Pstart) && (pos > motion.Pstart)))		//	[(结束位置 < 开始位置) && (处理过的真实角度pos >开始位置)]
	{
		TRANSATE_MOTOR_TARGET_RPM = motion.Vstart;	//TARGET_RPM = 开始的速度
	}
	else if(((motion.Pend > motion.Pstart) && (pos > motion.Pend)) ||
		      ((motion.Pend < motion.Pstart) && (pos < motion.Pend)))
	{
		TRANSATE_MOTOR_TARGET_RPM = motion.Vend;	//TARGET_RPM = 末尾的速度
	}
	else
	{
		S = ABS(pos - motion.Pstart);      //开始位置
		
		// 规划RPM
		if     (S < Sac)       TRANSATE_MOTOR_TARGET_RPM = sqrt(2.0f * Aac * S + motion.Vstart * motion.Vstart);               // 加速阶段
		else if(S < (Sac+Sco)) TRANSATE_MOTOR_TARGET_RPM = motion.Vmax;                                                        // 匀速阶段
		else                   TRANSATE_MOTOR_TARGET_RPM = sqrt(motion.Vend * motion.Vend - 2.0f * Ade * ABS(Ssu - S));  // 减速阶段
	}
	
	// 分配合适的正负号
	if(motion.Pend < motion.Pstart) TRANSATE_MOTOR_TARGET_RPM = -TRANSATE_MOTOR_TARGET_RPM;
	//pid
	PID_incremental_PID_calculation(&M3508_TRANSATE.MOTOR_PID, M3508_TRANSATE.REAL_INFO.RPM ,TRANSATE_MOTOR_TARGET_RPM);
	M3508_TRANSATE.REAL_INFO.TARGET_CURRENT = M3508_TRANSATE.MOTOR_PID.output;
	
	if(ABS(S-Ssu)<0.0001){transate_finished=1;}
	
	//如果链条堵转(测试)
	else if(ABS(S-Ssu)>5&&TRANSATE_MOTOR_TARGET_RPM<5){PUSH(Ssu,motion.Pend,motion.Vmax,motion.Vstart,motion.Vend,motion.Rac,motion.Rde);}//void PUSH(float start,float end,float speedmax,float speedstart,float speedend,float ac,float de)
//	//s型曲线规划的参数设置
//	curve.speedMax=8000;
//	curve.aTimes=1000;
//	curve.p_add=0.1;
//	curve.speedMin=100;
//	curve.stepSpeed=40;
//	MotorVelocityCurve(&curve,&M3508_TRANSATE.MOTOR_PID);
//	M3508_TRANSATE.REAL_INFO.TARGET_CURRENT = M3508_TRANSATE.MOTOR_PID.output;
	//LADRC
//	LADRC_Loop(&ADRC_M3508_TRANSATE,M3508_TRANSATE.REAL_INFO.RPM ,TRANSATE_MOTOR_TARGET_RPM);
//	M3508_TRANSATE.REAL_INFO.TARGET_CURRENT=ADRC_M3508_TRANSATE.u;
}
void set_speed(float speed)
{
	TRANSATE_MOTOR_TARGET_RPM = speed;
	PID_incremental_PID_calculation(&M3508_TRANSATE.MOTOR_PID, M3508_TRANSATE.REAL_INFO.RPM ,TRANSATE_MOTOR_TARGET_RPM);
	M3508_TRANSATE.REAL_INFO.TARGET_CURRENT = M3508_TRANSATE.MOTOR_PID.output;
	chassis_m3508_m2006_send_motor_currents_can1();
}
	

