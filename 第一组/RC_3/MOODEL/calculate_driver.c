#include "calculate_driver.h"

ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
CHASSIS_MOTOR_RPM CHASSIS_MOTOR_TARGET_RPM;

CurveObjectType curve;


// 4轮机器人坐标系逆运动学
// 世界坐标系
// W：正值-逆时针 负值-顺时针
void Robot_4wheels(float Vx_RPM, float Vy_RPM, float W_RPM)
{
	CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM = ( COS45 * Vy_RPM + SIN45 * Vx_RPM + Robot_R*W_RPM) * MS_TO_RPM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM = ( COS45 * Vy_RPM - SIN45 * Vx_RPM + Robot_R*W_RPM) * MS_TO_RPM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM = (-COS45 * Vy_RPM - SIN45 * Vx_RPM + Robot_R*W_RPM) * MS_TO_RPM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR4_RPM = (-COS45 * Vy_RPM + SIN45 * Vx_RPM + Robot_R*W_RPM) * MS_TO_RPM;
}

// 4轮世界坐标系逆运动学  
// theta为机器人坐标系x轴与世界坐标系x轴夹角 单位：度
// W：正值-逆时针 负值-顺时针
void World_4wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta)
{
	theta = PI * theta / 180.0f;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM = ( cos(theta-PI/4.0f) * Vy_RPM - sin(theta-PI/4.0f) * Vx_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM = ( cos(theta+PI/4.0f) * Vy_RPM - sin(theta+PI/4.0f) * Vx_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM = (-cos(theta-PI/4.0f) * Vy_RPM + sin(theta-PI/4.0f) * Vx_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR4_RPM = (-cos(theta+PI/4.0f) * Vy_RPM + sin(theta+PI/4.0f) * Vx_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
}

// 3轮世界坐标系逆运动学——角为前反向
// theta为机器人坐标系x轴与世界坐标系x轴夹角 单位：度
// W：正值-逆时针 负值-顺时针 
void World_3wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta)
{
	theta = PI * theta / 180.0f;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM = ( cos(theta) * Vy_RPM - sin(theta) * Vx_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM = (-cos(theta+PI/3.0f) * Vy_RPM - sin(theta+PI/3.0f) * Vx_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM = (-sin(theta+PI/6.0f) * Vy_RPM + cos(theta+PI/6.0f) * Vx_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
}

//三轮世界坐标运动学逆解——边为前方向
void world_3wheel(float Vx_RPM, float Vy_RPM, float W_RPM, float theta)
{
	theta = PI * theta / 180.0f;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM = ( -cos(theta) * Vx_RPM -sin(theta) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM = (+sin(theta+PI/6.0f) * Vx_RPM - cos(theta+PI/6.0f) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM = (+cos(theta+PI/3.0f) * Vx_RPM + sin(theta+PI/3.0f) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
}


//曲线规划
void MotorVelocityCurve(CurveObjectType *curve,PID *M3508)
{
	
 int flag=0;
 int time_now =0;
 int  time_acc = curve->p_add * curve->aTimes;		//加速时间
int time_con = curve->aTimes-time_acc;//匀速时间。
    //控制不能超速。
	if(curve->currentSpeed>curve->speedMax)
  {
    curve->currentSpeed=curve->speedMax;
  }
	//控制不能超速
//	 if(curve->targetSpeed<curve->speedMin)
//  {
//    curve->targetSpeed=curve->speedMin;
//  }
	if((M3508->output>curve->speedMin)&&flag!=1)
	//加速阶段
	{
		while(time_now < time_acc)
	{
		time_now++;
		curve->currentSpeed+=curve->stepSpeed;
		vTaskDelay(1);
	}
	}
	while(time_now > time_acc)
	{
		flag=1;
		
		if(M3508->output>curve->speedMax)
		{
		time_now++;
		curve->currentSpeed=curve->speedMax;}
		if(M3508->output<curve->speedMax)
			curve->currentSpeed=M3508->output;
		vTaskDelay(1);
	}
if(M3508->output<curve->speedMin)
	flag=0;
if(time_now>curve->aTimes)
{
time_now=0;
}
}



