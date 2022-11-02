#include "includes.h"

MOVE_STATE_ITEMS MOVE_STATE;
PID yaw_pid;
PID yaw_pid_ladar;
PID point_X_pid;
PID point_Y_pid;
PID laser_X_pid;
PID laser_Y_pid;
PID point_pid;
PID arc_pid;
PID point_traker_x_pid;
PID point_traker_y_pid;
PID point_traker_yaw_pid;
PID point_traker_ladar_y_pid;
PID point_traker_ladar_x_pid;


/**
* @brief  Move_Init��ʼ��
* @note		�ƶ����PID��ʼ��
* @param
* @retval 
*/
void MoveInit(void)
{
  PID_parameter_init(&point_X_pid, 0.025, 0.001, 0.33, 0.3, 0.05, -1);
	PID_parameter_init(&point_Y_pid, 0.025, 0.001, 0.33, 0.3, 0.05, -1);
	//yawadjust
	PID_parameter_init(&yaw_pid, 50,0, 0.1, 1000, 0, 1);
	
	//����
	PID_parameter_init(&laser_X_pid, 3,0, 0.5, 100, 0, 10);
	PID_parameter_init(&laser_Y_pid, 3,0, 0.5, 100, 0, 10);
	
	
	//��Ե�׷��
	PID_parameter_init(&point_traker_x_pid, 3,0, 0.1, 500, 0, 10);
	PID_parameter_init(&point_traker_y_pid, 3,0, 0.1, 500, 0, 10);
	PID_parameter_init(&point_traker_yaw_pid, 10,0, 0.1, 500, 0, 1);
	PID_parameter_init(&point_traker_ladar_y_pid, 0.5,0, 0.1, 1000, 0, 5);
	PID_parameter_init(&point_traker_ladar_x_pid, 2,0, 0.1, 1000, 0, 5);
	PID_parameter_init(&yaw_pid_ladar, 50,0, 0.1, 1000, 0, 1);
	
	PID_parameter_init(&point_pid, 1.5,0.002, 1.0, 0, 0, -1); // ��pid max1.5  p1.5 i0.002 d1
	PID_parameter_init(&arc_pid, 1.5,0.002, 1.0, 0.1, 0, 10);    //Բ������pid     p 10? i0.01  ����50��
}




/**
* @brief  AngleLimit�Ƕ��޷�
* @note		���Ƕ�������-180�㵽180��
* @param  angle:Ҫ���Ƶ�ֵ
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
* @brief  YawAdjustƫ���ǿ���
* @note		��ƫ���ǿ�����Ŀ��Ƕ�
* @param  Target_angle:Ҫ���Ƶ�ֵ
* @retval 
*/
int YawAdjust(float Target_angle)
{
   float error;

	 // �������
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
   
   // ֱ������PID������ٶ�
   PID_position_PID_calculation_by_error(&yaw_pid, error);
   ROBOT_TARGET_VELOCITY_DATA.W_RPM = yaw_pid.output;	// ���̽��ٶ� ��λ��rad/s
	 
	  if(ABS(error)<1)return 0;
	 else 
	 {
		 return 1;
	 }
}	




/**
* @brief  LockupPoint������
* @note		����������ĳһ����
* @param  POS_X:Ҫ���Ƶ�Xֵ��POS_Y:Ҫ���Ƶ�Yֵ��POS_YAW:Ҫ���Ƶ�ƫ����
* @retval 
*/
void LockupPoint(float POS_X, float POS_Y, float POS_YAW)
{ 
	YawAdjust(POS_YAW);
	
	PID_position_PID_calculation(&point_X_pid, ROBOT_REAL_POS_DATA.POS_X, POS_X);
	PID_position_PID_calculation(&point_Y_pid, ROBOT_REAL_POS_DATA.POS_Y, POS_Y);
	
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = point_X_pid.output;
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = point_Y_pid.output;
}

//�����
int moving_point_track(float real_time,float POS_X, float POS_Y, float POS_YAW,float V_max)
{
		YawAdjust(POS_YAW);
	 
	  float error;
	  //�������
	error = sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));  // �������
	point_pid.outputmax = ABS(V_max);
  PID_position_PID_calculation_by_error(&point_pid, error);

	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM =-point_pid.output * 1.0f*(ROBOT_REAL_POS_DATA.POS_X - POS_X) / error;
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = -point_pid.output * 1.0f*(ROBOT_REAL_POS_DATA.POS_Y - POS_Y) / error;
	
	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X)<10&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y)<10)
	{
		return 1;
	}
		return 0;
}


/**
* @brief  LaserLockPoint������
* @note		�����״�ľ���ͽǶ�����ֱ�����Ҫ�����ĵ�
* @param  distance_robot:�����ӵľ��룬thetha:�����ӵĽǶ�ֵ��distance_object:������Ŀ���ľ���(��һ���̶���Բ��)
* @retval 
*/
int LaserLockPoint(int distance_robot , int thetha ,int distance_object,float V_max)//��λ��cm
{
	int distance_object1=distance_object*10;
	int distance_robot1=distance_robot*10;
	int true_distance=distance_robot1-distance_object1;//ע�ⵥλ���� cm->mm
	if(distance_robot==0)//�������״ﷶΧ
	{ 
		Free_Control_Limit();//С��Χң��
	}
	else	
	{
//		float POS_X=ROBOT_REAL_POS_DATA.POS_X-true_distance*sin(thetha+ROBOT_REAL_POS_DATA.POS_YAW);
//		float POS_Y=ROBOT_REAL_POS_DATA.POS_Y+true_distance*cos(thetha+ROBOT_REAL_POS_DATA.POS_YAW);
//		near_pillar(POS_X, POS_Y, thetha+ROBOT_REAL_POS_DATA.POS_YAW,V_max);
		if(ABS(distance_robot1-distance_object1)<10&&ABS(thetha)<0.5)//�����1cm
				return 1;
		else
			{
					near_pillar(true_distance, thetha,V_max);
					return 0;
			}
	}
}

//��������
void near_pillar(float distance,float POS_YAW,float V_max)
{
	  
	 //�������
	 
   PID_position_PID_calculation_by_error(&point_traker_ladar_y_pid,distance);
	 PID_position_PID_calculation_by_error(&yaw_pid_ladar,POS_YAW);
	 point_traker_ladar_y_pid.outputmax = ABS(V_max);
	
	
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM =point_traker_ladar_y_pid.output ;   
	ROBOT_TARGET_VELOCITY_DATA.W_RPM=yaw_pid_ladar.output;
		
}


float kp_x = 7;
float kd_x = 0;	//0.00011
float kp_y = 6;
float kd_y = 0;	//0.00011
float kp_yaw = 1;
float kd_yaw = 0;
float error_X;float error_Y;	// ����X��Yƫ��
float error_x;float error_y;	// ����x��yƫ��
float error_Yaw;							// ƫ����ƫ��
float now_yaw;								// ��ǰ������ƫ����
float u_output;								// ��������x�����ٶ����
float v_output;								// ��������y�����ٶ����
float w_ouput;								// ���ٶ����


/**
* @brief  PDController������
* @note		���ٹ滮�õ�·��
* @param  target_point:��λʱ��Ҫ���ٵĵ㣨���ȹ滮���ٶȣ���robot_now_pos:�����˵�ǰ���������µ�λ��
* @retval 
*/
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
	// �������
	error_X = target_point.X - robot_now_pos.POS_X;
	error_Y = target_point.Y - robot_now_pos.POS_Y;
	error_Yaw = target_point.Yaw - robot_now_pos.POS_YAW;
	//�Ƕ���ת��Ϊ������
	now_yaw = robot_now_pos.POS_YAW * PI / 180.0f;
	// ���㵽��������
	error_x =  cos(now_yaw) * error_X + sin(now_yaw) * error_Y;
	error_y = -sin(now_yaw) * error_X + cos(now_yaw) * error_Y;
	
	// �����ٶ�
	w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
	u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
																		target_point.V_y  * sin(now_yaw) + \
																		w_ouput * error_y * cos(now_yaw) - \
																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
	v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
																		target_point.V_y  * cos(now_yaw) - \
																		w_ouput * error_y * sin(now_yaw) - \
																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);
																		 
	// ����Ϊ��������ϵ�µ��ٶ�
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = u_output * cos(now_yaw) - v_output * sin(now_yaw);
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = u_output * sin(now_yaw) + v_output * cos(now_yaw);
	ROBOT_TARGET_VELOCITY_DATA.W_RPM  = -w_ouput;
}

int k;
float t;
float f1s;float f2s;float f3s;float f4s;
float last_X;float last_Y;float last_Yaw;
float Sx_error;float Sy_error;
float Hz;
int first_time_flag = 1;
PATH_TYPEDEF now_path_point;

/**
* @brief  PathPlan�滮+����
* @note		����B�����滮�����ֱ�Ӹ�ֵ�������յ㷵��1�����򷵻�0
* @param  t_real:��ʵ������ʱ�䣬t_target:Ŀ����ʱ�䣬num:���Ƶ���Ŀ+1��X��Y:���Ƶ�����
* @retval 
*/
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
{   	
	k = (int)(t_real * num / t_target);	// ��k��
	t = t_real - k * t_target / num;		// ��k��ʱ��
  t = t * num / t_target;							// ��һ��

	// λ����������
	f1s = (1 - t) * (1 - t) * (1 - t) / 6;
	f2s = (3 * t * t * t - 6 * t * t + 4) / 6;
	f3s = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
	f4s = (t * t * t) / 6;
	
	// ����Ŀ����ٵ�
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
	
	// PD������
	PDController(now_path_point, ROBOT_REAL_POS_DATA);
	
	// ��������ֵ
	last_X = now_path_point.X;
	last_Y = now_path_point.Y;
	last_Yaw = now_path_point.Yaw;

	// �����յ�
	if(t_real > t_target)
	{
		ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = 0;
		ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = 0;
		first_time_flag = 1;
		return 1;
	} 
	return 0;
}





