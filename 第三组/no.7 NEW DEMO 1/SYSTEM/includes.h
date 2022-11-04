#ifndef _INCLUDES_H
#define _INCLUDES_H
/*****************************************************************
               这里用来引入所需的头文件
*****************************************************************/

//FreeRTOS系统需要的头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"

//通信头文件
#include "can.h"
#include "pstwo.h"
#include "bsp_air.h"
#include "ros.h"
#include "laser.h"
#include "imu_location.h"
#include "jiguang.h"

//电机配置
#include "pid.h"
#include "m3508.h"
#include "TMOTOR.h"
#include "MIT.h"

//robot底盘运动
#include "control.h"
#include "action.h"
#include "move.h"
#include "path.h"

//机器人状态
#include "FSM.h"

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"

//算法
#include "LADRC.h"

/*****************************************************************
               这里用来定义所有的宏定义
****************************************************************/
#define PI 								 3.14159265358979f
#define COS45              0.70710678f
#define SIN45              0.70710678f
#define ABS(x)      ((x)>0? (x):(-(x)))
#define MIN(x,y)    ((x)<(y)? (x):(y) )

// debug条件编译
#define USE_DATASCOPE			0
#define USE_DEBUG					0
#define USE_PATH					1

// M3508电机编号(标识符0x200)
#define M3508_CHASSIS_MOTOR_ID_1      0x201
#define M3508_CHASSIS_MOTOR_ID_2      0x202
#define M3508_CHASSIS_MOTOR_ID_3      0x203
#define M3508_CHASSIS_MOTOR_ID_4      0x204

// M3508电机编号（角度控制）
#define M3508_UP_MOTOR_ID_5           0x205
#define M3508_YAW_MOTOR_ID_6          0x206
#define M3508_TRANSATE_MOTOR_ID_7     0x207

//三全向轮底盘的参数
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)

// Chassis Config
#define WHEEL_R            0.076f	                  //轮子半径(单位：m)
#define Robot_R            0.406f                  	//车轮到中心距离(单位：m)
#define M3508_RM_To_MS     (PI*WHEEL_R)/570.0f      //转速与速度的转换 (单位：m/s) 
#define M3508_MS_To_RM     1.0f/(PI*WHEEL_R)      //速度与转速的转换 (单位：m/s)  
#define MS_TO_RPM          21*60/(PI*WHEEL_R*2)     //轮子直径152mm，电机减速比1:21，轮子一圈pi*152mm
#define RM_transition_MS (PI * WHEEL_R) / 570.0f //转速与速度的转换
#define MS_transition_RM 1.0f / (PI * WHEEL_R) //速度与转速的转换
																										// 计算公式：1/（pi*轮子直径）*减速比*60
/***************************************
				ROBOT_STATE
****************************************/
void MotorVelocityCurve_task(void);//电机限速


/***************************************
				ROBOT_ACTION
****************************************/

/***************************************
				ROBOT_TRACE
****************************************/

/***************************************
			路径状态 Ph_flag
****************************************/

/*****************************************************************
               这里用来定义所有的状态机
****************************************************************/
typedef struct CHASSIS_MOTOR_RPM
{
	float MOTOR1_RPM;
	float MOTOR2_RPM;
	float MOTOR3_RPM;
}CHASSIS_MOTOR_RPM;

// 底盘期望速度
typedef struct ROBOT_TARGET_VELOCITY
{
	float Vx;
	float Vy;
	float W;
	float Vx_RPM;
	float Vy_RPM;
	float W_RPM;
}ROBOT_TARGET_VELOCITY;


extern struct CHASSIS_MOTOR_RPM CHASSIS_MOTOR_TARGET_RPM;
extern struct ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
extern struct PID M3508_CHASSIS_MOTOR_PID_RPM[3];	// 1-3底盘电机
extern struct PID M3508_CAST_MOTOR_PID_RPM;	
extern CurveObjectType curve;
extern struct M3508_CLASS M3508_UP;
extern struct M3508_CLASS M3508_YAW;
extern struct M3508_CLASS M3508_TRANSATE;
/*****************************************************************
               这里用来定义所有的全局变量
****************************************************************/
extern float Velocity_A,Velocity_B,Velocity_C;
extern int manual_move_mode;//(mode=0:世界坐标；mode=1:局部坐标)
extern int up_flag;
//小车三个方向的速度
extern float VX,VY;
extern int VW;
//ros
extern int ros_vx,ros_vy,ros_vz,new_x,new_y,new_z;
extern unsigned char rosctrl_flag;
extern short v;
extern short th;
extern int mode_flag;



/*****************************************************************
               这里用来定义所有的FREERTOS任务
****************************************************************/
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t Robot_state_task_Handler;
extern TaskHandle_t Auto_Task_Handler;
extern TaskHandle_t move_task_Handler;
extern TaskHandle_t Motor_Control_Handler;
extern TaskHandle_t data_update_Handler;

void start_task(void *pvParameters);
void Robot_state_task(void *pvParamerters);
void Auto_Task(void *pvParameters);
void move_task(void *pvParameters);
void Motor_Control(void *pvParameters);
void data_update(void *pvParameters);
#endif
