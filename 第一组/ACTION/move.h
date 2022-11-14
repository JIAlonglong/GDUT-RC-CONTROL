#ifndef __MOVE_H
#define __MOVE_H
#include "includeh.h"

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
	
	// 取环点
	MOVE_RING,
	
	
	// 取箭点
	RING_STOP,
		MOVE_SHOOTING_1,
	SHOOTING_STOP,
		MOVE_SHOOTING_2,
	
	// 启动区
	MOVE_RESTART
	
}MOVE_STATE_ITEMS;

typedef enum
{
	SHOOTING_1,
	
	// 取环点
	SHOOTING_2,
	
	
	// 取箭点
	SHOOTING_3,
	SHOOTING_4,
	SHOOTING_5,
	SHOOTING_6,
	
	// 启动区
	STOP_SHOOTING
	
}SHOOTING_STATE;


void AngleLimit(float *angle);
int YawAdjust(float Target_angle);
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw);
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos);
void MoveInit(void);
int PointTracking(float Target_x,float Target_y,float Target_yaw);
void near_pillar(float POS,float POS_YAW,float V_max);
int LaserLockPoint(int distance_robot , int thetha ,int distance_object,float V_max);
int ladar_track(int distance_robot , int thetha ,int distance_object,float V_max)	;
void PDController_ladar(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos);
void moving_point_track(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos,float vmax);
#endif
