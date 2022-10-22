#ifndef __MOVE_H
#define __MOVE_H

#include "action.h"
#include "pid.h"

#define x  0
#define y  1
#define w  2


typedef struct
{
	float X;
	float Y;
	float Yaw;
	float V_x;
	float V_y;
	float W;
}PATH_TYPEDEF;


typedef enum
{
	MOVE_STOP,
	
	// ȡ����
	MOVE_1_CAST_POINT,
	
	// �价1
	MOVE_1_SHOOT,
	
	// �价2
	MOVE_2_SHOOT,
	
	// �价3
	MOVE_3_SHOOT,
	
	// ������
	MOVE_1_RESTART
	
}MOVE_STATE_ITEMS;

typedef struct TRCK_PIONTS
{
	float X;           //������
	float Y;             
	float T;           //����ֱ�߾���ʱ��   MS
	float theate;
	float Sl;           //�õ굼��
	float V;           //�ٶ�             M/S
}TRCK_PIONTS;

//�������ݽṹ��
typedef struct ROBOT_CHASSIS
{

	float World_V[3]; // X , Y , W
	float Robot_V[3];
	float Position[2];
	float Motor_RPM[3];
	float expect_angle ;
	float Angle;
} ROBOT_CHASSIS;
typedef struct PATH_FOLLOW
{
	u16 TIME_LAST; //�ϴε�ʱ��
	u16 TIME_pass; //������ʱ��
	u8 COUNT;	   //����
} PATH_FOLLOW;


extern ROBOT_CHASSIS Robot_Chassis;

void MoveInit(void);
int YawAdjust(float Target_angle);
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos);
int PathPlan(float t_real, float t_target, int num, float *X, float *Y, float *Yaw);
void LockupPoint(float POS_X, float POS_Y, float POS_YAW);
int LaserLockPoint(int distance_robot , int thetha ,int distance_object,float V_max);//��λ��cm
int moving_point_track(float real_time,float POS_X, float POS_Y, float POS_YAW,float V_max);
void near_pillar(float distance,float POS_YAW,float V_max);
int PointTracking(float Target_x,float Target_y,float Target_yaw);
float Caculate_K(float dx, float dy);
void POINT_FOLLOW(TRCK_PIONTS *PIONTS, TRCK_PIONTS *Last_PIONTS);
float Vector_Unitization(float k, u8 num);
void PATH_TRACKING(TRCK_PIONTS *pionts, uint8_t num);

void model_ChassisRoute_Init(void);

extern PID yaw_pid;
extern PID point_X_pid;
extern PID point_Y_pid;
extern PID laser_X_pid;
extern PID laser_Y_pid;
extern PID point_traker_x_pid;
extern PID point_traker_y_pid;
extern PID point_traker_yaw_pid;
extern PID point_traker_ladar_y_pid;
extern PID point_traker_ladar_x_pid;
extern PID yaw_pid_ladar;

extern MOVE_STATE_ITEMS MOVE_STATE;


#endif
