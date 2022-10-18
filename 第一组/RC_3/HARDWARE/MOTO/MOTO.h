#ifndef __MOTOR_H
#define __MOTOR_H
#include "can.h"
#include "PID.h"


// M3508电机编号
#define M3508_CHASSIS_MOTOR_ID_1      0x201
#define M3508_CHASSIS_MOTOR_ID_2      0x202
#define M3508_CHASSIS_MOTOR_ID_3      0x203
#define M3508_CHASSIS_MOTOR_ID_4      0x204
#define M3508_CAST_MOTOR_ID	        	0x205

typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   		        //采样角度						
	int16_t  RPM;					//速度值			
	int16_t  CURRENT;     //电流值
	int16_t  TARGET_CURRENT;//目标电流值
	
	// 角度积分时用到下面变量
	float		 REAL_ANGLE;         //处理过的真实角度（必须用float）
	u8			 FIRST_ANGLE_INTEGRAL_FLAG;  //?
	uint16_t LAST_ANGLE;   //?
}M3508_REAL_INFO;


//用于曲线规划的结构体
/* 定义电机速度曲线对象 */
typedef struct CurveObject {
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
  
}CurveObjectType;



extern struct PID M3508_CAST_MOTOR_PID_RPM;				// 射箭机构电机
extern struct PID M3508_CHASSIS_MOTOR_PID_RPM[4];	// 1-4底盘电机
extern struct M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[4]; 

void M3508_Motor_Init(void);
void m3508_update_m3508_info(CanRxMsg *msg);
void chassis_m3508_send_motor_currents(void);
void shoot_m3508_send_motor_currents(void);
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR);

#endif

