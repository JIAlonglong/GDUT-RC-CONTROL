#ifndef _INCLUDES_H
#define _INCLUDES_H
/*****************************************************************
               �����������������ͷ�ļ�
*****************************************************************/

//FreeRTOSϵͳ��Ҫ��ͷ�ļ�
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"

//ͨ��ͷ�ļ�
#include "can.h"
#include "pstwo.h"
#include "bsp_air.h"
#include "ros.h"
#include "laser.h"
#include "imu_location.h"
#include "jiguang.h"

//�������
#include "pid.h"
#include "m3508.h"
#include "TMOTOR.h"
#include "MIT.h"

//robot�����˶�
#include "control.h"
#include "action.h"
#include "move.h"
#include "path.h"

//������״̬
#include "FSM.h"

//C library function related header file
//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"

//�㷨
#include "LADRC.h"

/*****************************************************************
               ���������������еĺ궨��
****************************************************************/
#define PI 								 3.14159265358979f
#define COS45              0.70710678f
#define SIN45              0.70710678f
#define ABS(x)      ((x)>0? (x):(-(x)))
#define MIN(x,y)    ((x)<(y)? (x):(y) )

// debug��������
#define USE_DATASCOPE			0
#define USE_DEBUG					0
#define USE_PATH					1

// M3508������(��ʶ��0x200)
#define M3508_CHASSIS_MOTOR_ID_1      0x201
#define M3508_CHASSIS_MOTOR_ID_2      0x202
#define M3508_CHASSIS_MOTOR_ID_3      0x203
#define M3508_CHASSIS_MOTOR_ID_4      0x204

// M3508�����ţ��Ƕȿ��ƣ�
#define M3508_UP_MOTOR_ID_5           0x205
#define M3508_YAW_MOTOR_ID_6          0x206
#define M3508_TRANSATE_MOTOR_ID_7     0x207

//��ȫ���ֵ��̵Ĳ���
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)

// Chassis Config
#define WHEEL_R            0.076f	                  //���Ӱ뾶(��λ��m)
#define Robot_R            0.406f                  	//���ֵ����ľ���(��λ��m)
#define M3508_RM_To_MS     (PI*WHEEL_R)/570.0f      //ת�����ٶȵ�ת�� (��λ��m/s) 
#define M3508_MS_To_RM     1.0f/(PI*WHEEL_R)      //�ٶ���ת�ٵ�ת�� (��λ��m/s)  
#define MS_TO_RPM          21*60/(PI*WHEEL_R*2)     //����ֱ��152mm��������ٱ�1:21������һȦpi*152mm
#define RM_transition_MS (PI * WHEEL_R) / 570.0f //ת�����ٶȵ�ת��
#define MS_transition_RM 1.0f / (PI * WHEEL_R) //�ٶ���ת�ٵ�ת��
																										// ���㹫ʽ��1/��pi*����ֱ����*���ٱ�*60
/***************************************
				ROBOT_STATE
****************************************/
void MotorVelocityCurve_task(void);//�������


/***************************************
				ROBOT_ACTION
****************************************/

/***************************************
				ROBOT_TRACE
****************************************/

/***************************************
			·��״̬ Ph_flag
****************************************/

/*****************************************************************
               ���������������е�״̬��
****************************************************************/
typedef struct CHASSIS_MOTOR_RPM
{
	float MOTOR1_RPM;
	float MOTOR2_RPM;
	float MOTOR3_RPM;
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


extern struct CHASSIS_MOTOR_RPM CHASSIS_MOTOR_TARGET_RPM;
extern struct ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
extern struct PID M3508_CHASSIS_MOTOR_PID_RPM[3];	// 1-3���̵��
extern struct PID M3508_CAST_MOTOR_PID_RPM;	
extern CurveObjectType curve;
extern struct M3508_CLASS M3508_UP;
extern struct M3508_CLASS M3508_YAW;
extern struct M3508_CLASS M3508_TRANSATE;
/*****************************************************************
               ���������������е�ȫ�ֱ���
****************************************************************/
extern float Velocity_A,Velocity_B,Velocity_C;
extern int manual_move_mode;//(mode=0:�������ꣻmode=1:�ֲ�����)
extern int up_flag;
//С������������ٶ�
extern float VX,VY;
extern int VW;
//ros
extern int ros_vx,ros_vy,ros_vz,new_x,new_y,new_z;
extern unsigned char rosctrl_flag;
extern short v;
extern short th;
extern int mode_flag;



/*****************************************************************
               ���������������е�FREERTOS����
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
