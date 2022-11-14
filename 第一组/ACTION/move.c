#include "move.h"
#include "math.h"

MOVE_STATE_ITEMS MOVE_STATE = MOVE_STOP;
SHOOTING_STATE SHOOT_STATE = STOP_SHOOTING;
PID yaw_pid;
PID yaw_pid_ladar;
PID point_X_pid;
PID point_Y_pid;
PID laser_X_pid;
PID laser_Y_pid;
PID point_traker_x_pid;
PID point_traker_y_pid;
PID point_traker_yaw_pid;
PID point_traker_ladar_y_pid;
PID point_traker_ladar_x_pid;
/**
* @brief  Move_Init初始化
* @note		移动相关PID初始化
* @param
* @retval 
*/
void MoveInit(void)
{
	PID_parameter_init(&point_X_pid, 0.025, 0.001, 0.33, 0.3, 0.05, -1);
	PID_parameter_init(&point_Y_pid, 0.025, 0.001, 0.33, 0.3, 0.05, -1);
	//yawadjust
	PID_parameter_init(&yaw_pid, 40,5, 0.1, 2000, 0, 1);
	//激光
	PID_parameter_init(&laser_X_pid, 3,0, 0.5, 100, 0, 10);
	PID_parameter_init(&laser_Y_pid, 3,0, 0.5, 100, 0, 10);
	//点对点追踪
	PID_parameter_init(&point_traker_x_pid, 3,0, 0.1, 1000, 0, 10);
	PID_parameter_init(&point_traker_y_pid, 3,0, 0.1, 1000, 0, 10); 
	PID_parameter_init(&point_traker_yaw_pid, 10,0, 0.1,500, 0, 1);
	PID_parameter_init(&point_traker_ladar_y_pid,1,0, 0.1, 500, 0, 5);
	PID_parameter_init(&point_traker_ladar_x_pid, 2,0, 0.1, 1000, 0, 5);
	PID_parameter_init(&yaw_pid_ladar, 20,0, 0.1, 500, 0, 1);
}

float kp_x = 7;
float kd_x = 0.1;	//0.00011
float kp_y = 7;
float kd_y = 0.1;	//0.00011
float kp_yaw = 1;
float kd_yaw = 0.1;
float error_X;float error_Y;	// 世界X、Y偏差
float error_x;float error_y;	// 本体x、y偏差
float error_Yaw;							// 偏航角偏差
float now_yaw;								// 当前弧度制偏航角
float u_output;								// 本体坐标x方向速度输出
float v_output;								// 本体坐标y方向速度输出
float w_ouput;								// 角速度输出


/**
* @brief  PDController跟踪器
* @note		跟踪规划好的路径
* @param  target_point:单位时间要跟踪的点（需先规划好速度），robot_now_pos:机器人当前世界坐标下的位置
* @retval 
*/
/*
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
	// 计算误差
	error_X = target_point.X - robot_now_pos.POS_X;
	error_Y = target_point.Y - robot_now_pos.POS_Y;
	error_Yaw = target_point.Yaw - robot_now_pos.POS_YAW;
	//角度制转换为弧度制
	now_yaw = robot_now_pos.POS_YAW * PI / 180.0f;
	// 换算到本体坐标
	error_x =  cos(now_yaw) * error_X + sin(now_yaw) * error_Y;
	error_y = -sin(now_yaw) * error_X + cos(now_yaw) * error_Y;
	
	// 计算速度
	w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
	u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
																		target_point.V_y  * sin(now_yaw) + \
																		w_ouput * error_y * cos(now_yaw) - \
																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
	v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
																		target_point.V_y  * cos(now_yaw) - \
																		w_ouput * error_y * sin(now_yaw) - \
																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);
																		 
	// 换算为世界坐标系下的速度
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = (u_output * cos(now_yaw) - v_output * sin(now_yaw));
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = u_output * sin(now_yaw) + v_output * cos(now_yaw);
	ROBOT_TARGET_VELOCITY_DATA.W_RPM  = -w_ouput;
}

*/
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
	// 计算误差
	error_X = target_point.X - robot_now_pos.POS_X;
	error_Y = target_point.Y - robot_now_pos.POS_Y;
	error_Yaw = target_point.Yaw - robot_now_pos.POS_YAW;


	/*
	// 计算速度
	w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
	u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
																		target_point.V_y  * sin(now_yaw) + \
																		w_ouput * error_y * cos(now_yaw) - \
																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
	v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
																		target_point.V_y  * cos(now_yaw) - \
																		w_ouput * error_y * sin(now_yaw) - \
																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);
																		 
	// 换算为世界坐标系下的速度
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = (u_output * cos(now_yaw) - v_output * sin(now_yaw));
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = u_output * sin(now_yaw) + v_output * cos(now_yaw);
	ROBOT_TARGET_VELOCITY_DATA.W_RPM  = -w_ouput;
	*/
	
	PID_position_PID_calculation_by_error(&point_traker_x_pid, error_X);
	PID_position_PID_calculation_by_error(&point_traker_y_pid, error_Y);
	PID_position_PID_calculation_by_error(&point_traker_yaw_pid, error_Yaw);
	
	
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = point_traker_x_pid.output;
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = point_traker_y_pid.output;
	ROBOT_TARGET_VELOCITY_DATA.W_RPM  = -point_traker_yaw_pid.output;
}

void PDController_ladar(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
	// 计算误差
	error_X = target_point.X - robot_now_pos.POS_X;
	error_Y = target_point.Y - robot_now_pos.POS_Y;
	error_Yaw = target_point.Yaw - robot_now_pos.POS_YAW;


	/*
	// 计算速度
	w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
	u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
																		target_point.V_y  * sin(now_yaw) + \
																		w_ouput * error_y * cos(now_yaw) - \
																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
	v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
																		target_point.V_y  * cos(now_yaw) - \
																		w_ouput * error_y * sin(now_yaw) - \
																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);
																		 
	// 换算为世界坐标系下的速度
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = (u_output * cos(now_yaw) - v_output * sin(now_yaw));
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = u_output * sin(now_yaw) + v_output * cos(now_yaw);
	ROBOT_TARGET_VELOCITY_DATA.W_RPM  = -w_ouput;
	*/
	
	PID_position_PID_calculation_by_error(&point_traker_ladar_y_pid, error_X);
	PID_position_PID_calculation_by_error(&point_traker_ladar_x_pid, error_Y);
	PID_position_PID_calculation_by_error(&yaw_pid_ladar, error_Yaw);
	
	
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = point_traker_x_pid.output;
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = point_traker_y_pid.output;
	ROBOT_TARGET_VELOCITY_DATA.W_RPM  = -point_traker_yaw_pid.output;
}

int k;
float t;
float f1s;float f2s;float f3s;float f4s;
float last_X;float last_Y;float last_Yaw;
float Sx_error;float Sy_error;
float Hz;
int first_time_flag = 1;
PATH_TYPEDEF now_path_point;
PATH_TYPEDEF ladar_now_path_point;



/**
* @brief  PathPlan规划+跟踪
* @note		三次B样条规划，误差直接赋值，到达终点返回1，否则返回0
* @param  t_real:真实经过的时间，t_target:目标总时间，num:控制点数目+1，X、Y:控制点数组
* @retval 
*/
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
{   	
	k = (int)(t_real * num / t_target);	// 第k段
	t = t_real - k * t_target / num;		// 第k段时间
  t = t * num / t_target;							// 归一化

	// 位置样条函数
	f1s = (1 - t) * (1 - t) * (1 - t) / 6;
	f2s = (3 * t * t * t - 6 * t * t + 4) / 6;
	f3s = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
	f4s = (t * t * t) / 6;
	
	// 计算目标跟踪点
	now_path_point.X = X[k] * f1s + X[k+1] * f2s + X[k+2] * f3s + X[k+3] * f4s;
	now_path_point.Y = Y[k] * f1s + Y[k+1] * f2s + Y[k+2] * f3s + Y[k+3] * f4s;
	now_path_point.Yaw = Yaw[k] * f1s + Yaw[k+1] * f2s + Yaw[k+2] * f3s + Yaw[k+3] * f4s;
	if(first_time_flag)
	{
		now_path_point.V_x = 0;
		now_path_point.V_y = 0;
		now_path_point.W = 0;
		first_time_flag = 0;
		Hz = 1 / t_real;
	}
	else
	{
		now_path_point.V_x = (now_path_point.X - last_X) * Hz;
		now_path_point.V_y = (now_path_point.Y - last_Y) * Hz;
		now_path_point.W = (now_path_point.Yaw - last_Yaw) * Hz;
	}
	
	// PD跟踪器
	PDController(now_path_point, ROBOT_REAL_POS_DATA);
	
	
	// 保留本次值
	last_X = now_path_point.X;
	last_Y = now_path_point.Y;
	last_Yaw = now_path_point.Yaw;

	// 到达终点
	if(t_real > t_target)
	{
		ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = 0;
		ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = 0;
		first_time_flag = 1;
		return 1;
	} 
	return 0;
}
/**
* @brief  点追踪
* @note		直线移动到目标点
* @param  Target_x,Target_y,Target_yaw目标点和目标偏航角
* @retval 未完成，返回1；完成，返回0
PID point_traker_x_pid;
PID point_traker_y_pid;
PID point_traker_yaw_pid;
*/

int PointTracking(float Target_x,float Target_y,float Target_yaw)
{
	float x_error,y_error,yaw_error;
	x_error=Target_x-ROBOT_REAL_POS_DATA.POS_X;
	y_error=Target_y-ROBOT_REAL_POS_DATA.POS_Y;
	yaw_error=Target_yaw-ROBOT_REAL_POS_DATA.POS_YAW;
	
	
	// 留死区
	if(ABS((x_error) > 1&&ABS(y_error) > 1)&&ABS(yaw_error) > 0.5)
	{
	PID_position_PID_calculation_by_error(&point_traker_x_pid, x_error);
	PID_position_PID_calculation_by_error(&point_traker_y_pid, y_error);
	PID_position_PID_calculation_by_error(&point_traker_yaw_pid, yaw_error);
	
	
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = point_traker_x_pid.output;
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = point_traker_y_pid.output;
	ROBOT_TARGET_VELOCITY_DATA.W_RPM  = point_traker_yaw_pid.output;
	return 1;
	}
	else
	{
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = 0;
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = 0;
	ROBOT_TARGET_VELOCITY_DATA.W_RPM = 0;
	return 0;
	}
	
}





/**
* @brief  YawAdjust偏航角控制
* @note		将偏航角控制在目标角度
* @param  Target_angle:要限制的值
* @retval 未完成，返回1；完成，返回0
*/
int YawAdjust(float Target_angle)
{
   float error;
   
	 // 计算误差
   if(ROBOT_REAL_POS_DATA.POS_YAW*Target_angle >= 0)
   {
      error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
   }
   else
   {
		 if(ABS(ROBOT_REAL_POS_DATA.POS_YAW)+ABS(Target_angle) <= 180) error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
		 else 
		 {
				AngleLimit(&error);
		 }
   }
   
   // 直接利用PID输出角速度
   PID_position_PID_calculation_by_error(&yaw_pid, error);
   
	 ROBOT_TARGET_VELOCITY_DATA.W_RPM = -yaw_pid.output;	// 底盘角速度 单位：rad/s
	 if(ABS(error)<1)return 0;
	 else 
	 {
		 return 1;
	 }
	 
}	

/**
* @brief  YawAdjust偏航角控制
* @note		将偏航角控制在目标角度
* @param  Target_angle:要限制的值
* @retval 未完成，返回1；完成，返回0
*/
int YawAdjust_lasar(float Target_angle)
{
   float error;
   
	 // 计算误差
   if(ROBOT_REAL_POS_DATA.POS_YAW*Target_angle >= 0)
   {
      error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
   }
   else
   {
		 if(ABS(ROBOT_REAL_POS_DATA.POS_YAW)+ABS(Target_angle) <= 180) error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
		 else 
		 {
				AngleLimit(&error);
		 }
   }
   
   // 直接利用PID输出角速度
   PID_position_PID_calculation_by_error(&yaw_pid_ladar, error);
   
	 ROBOT_TARGET_VELOCITY_DATA.W_RPM = -yaw_pid.output;	// 底盘角速度 单位：rad/s
	 if(ABS(error)<1)return 0;
	 else 
	 {
		 return 1;
	 }
	 
}	

/**
* @brief  AngleLimit角度限幅
* @note		将角度限制在-180°到180°
* @param  angle:要限制的值
* @retval 
*/
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}

/**
* @brief  LaserLockPoint锁定车
* @note		利用雷达的距离和角度数据直接算出要锁车的点
* @param  distance_robot:与柱子的距离，thetha:与柱子的角度值，distance_object:柱子与目标点的距离(在一个固定的圆上)
* @retval 
*/
//雷达逼进函数 
int LaserLockPoint(int distance_robot , int thetha ,int distance_object,float V_max)//单位：cm
{
	int distance_object1=distance_object;
	int distance_robot1=distance_robot;
	int true_distance=distance_robot1-distance_object1;//误差
				
	if((distance_robot1<=(distance_object1+20)&&distance_robot1>=(distance_object1-20))||distance_robot1==0||distance_robot>distance_object+1000)
	{return 1;

	}
	else
		
	{
		near_pillar(true_distance, thetha,V_max);
		return 0;
	}
}
//点跟踪
void near_pillar(float POS,float POS_YAW,float V_max)
{
	  
	
	
		 
	  //计算误差
	 
   PID_position_PID_calculation_by_error(&point_traker_ladar_y_pid,POS);
	 PID_position_PID_calculation_by_error(&yaw_pid_ladar,POS_YAW);
	 point_traker_ladar_y_pid.outputmax = ABS(V_max);
	
	
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM =point_traker_ladar_y_pid.output ;   
	ROBOT_TARGET_VELOCITY_DATA.W_RPM=-yaw_pid_ladar.output;
		
}
