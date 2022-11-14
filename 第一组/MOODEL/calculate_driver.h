#ifndef __CALCULATE_DRIVER_H
#define __CALCULATE_DRIVER_H

#include "includeh.h"
//���̵�����ٶ�
typedef struct CHASSIS_MOTOR_RPM
{
	float MOTOR1_RPM;
	float MOTOR2_RPM;
	float MOTOR3_RPM;
	float MOTOR4_RPM;
}CHASSIS_MOTOR_RPM;

// ���������ٶ�
typedef struct ROBOT_TARGET_VELOCITY
{
	float Vx;
	float Vy;
	float W;
	float Vx_RPM;
	float Vy_RPM;
	float W_RPM;
}ROBOT_TARGET_VELOCITY;

extern CHASSIS_MOTOR_RPM CHASSIS_MOTOR_TARGET_RPM;
extern ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
extern CurveObjectType curve;

void Robot_4wheels(float Vx_RPM, float Vy_RPM, float W_RPM);
void World_4wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta);
void World_3wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta);
void world_3wheel(float Vx_RPM, float Vy_RPM, float W_RPM, float theta);
void MotorVelocityCurve(CurveObjectType *curve,PID *M3508);

#endif
