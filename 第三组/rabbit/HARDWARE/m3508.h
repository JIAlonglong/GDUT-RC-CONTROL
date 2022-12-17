#ifndef __M3508_H
#define __M3508_H
#include "stm32f4xx.h"
extern int start,end;
extern int transate_finished;
extern int up_finished;
extern 	float angle_1;
// M3508返回的电机真实信息
typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   		        //采样角度						
	int16_t  RPM;
  int16_t  TARGET_RPM;	
	int16_t  CURRENT;
	int16_t  TARGET_CURRENT;
	
	// 角度积分时用到下面变量
	float		 REAL_ANGLE;         //处理过的真实角度（必须用float）
	u8			 FIRST_ANGLE_INTEGRAL_FLAG;
	uint16_t LAST_ANGLE;
	
	
}M3508_REAL_INFO;

//用于曲线规划的结构体
/* 定义电机速度曲线对象 */
typedef struct CurveObjectType {
	float distance;       //距离
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

typedef struct M3508_CLASS
{
	M3508_REAL_INFO REAL_INFO; 
	PID MOTOR_PID;	
	int TARGET_RPM;								
	int SET_CURRENT;
	int TARGET_ANGLE;
}M3508_CLASS;

typedef struct ARM_VELOCITY_PLANNING //速度规划
{
	float Pstart;        //开始位置
	float Pend;          //结束位置
	float Vstart;        //开始的速度           // 单位：RPM 绝对值
	float Vmax;          //最大的速度
	float Vend;          //末尾的速度
	float Rac;           //加速路程的比例
	float Rde;           //减速路程的比例
}ARM_VELOCITY_PLANNING;

typedef struct TRANSATE_VELOCITY_PLANNING //速度规划
{
	float Distance;
	float Pstart;        //开始位置
	float Pend;          //结束位置
	float Vstart;        //开始的速度           // 单位：RPM 绝对值
	float Vmax;          //最大的速度
	float Vend;          //末尾的速度
	float Rac;           //加速路程的比例
	float Rde;           //减速路程的比例
}TRANSATE_VELOCITY_PLANNING;

extern struct ARM_VELOCITY_PLANNING  *UP_ARM_NOW_MOTION;		 // 指向抬升当前动作
extern struct ARM_VELOCITY_PLANNING   UP_INIT;//抬升机构初始化
extern struct ARM_VELOCITY_PLANNING   UP_ON1;//抬升
extern struct ARM_VELOCITY_PLANNING   UP_ON2;//抬升
extern struct ARM_VELOCITY_PLANNING   UP_ON3;//抬升
extern struct ARM_VELOCITY_PLANNING   UP_ON4;
extern struct ARM_VELOCITY_PLANNING   UP_ON5;
extern struct ARM_VELOCITY_PLANNING   UP_ON6;
extern struct ARM_VELOCITY_PLANNING   UP_DOWN3;//下降

extern struct ARM_VELOCITY_PLANNING  *YAW_ARM_NOW_MOTION;		 // 指向云台当前动作
extern struct ARM_VELOCITY_PLANNING   YAW_INIT;//云台机构初始化
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_HANDLE;//云台机构手动转  VRB
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_AUTO;//云台机构自动转(vision版)
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_1;//云台机构固定点转1
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_2;//云台机构固定点转2
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_3;//云台机构固定点转3

extern struct TRANSATE_VELOCITY_PLANNING *TRANSATE_NOW_MOTION; //传递环的电机（思路一）（规定距离）
																															//	思路二 开始和结束的位置累加上次的位置值
extern struct TRANSATE_VELOCITY_PLANNING TRANSATE_INIT;
extern struct TRANSATE_VELOCITY_PLANNING TRANSATE_1;
extern struct CurveObjectType TRANSATE;

extern int16_t UP_MOTOR_TARGET_RPM ;    // 抬升电机目标速度
extern int16_t YAW_MOTOR_TARGET_RPM ;    // 转向电机目标速度
extern int16_t TRANSATE_MOTOR_TARGET_RPM;
extern int16_t PICK_TRANSATE_MOTOR_TARGET_RPM;

void M3508_Motor_Init(void);
void m3508_update_m3508_info_can1(CanRxMsg *msg);
void m3508_update_m3508_info_can2(CanRxMsg *msg);
void chassis_m3508_send_motor_currents_can1(void);
void chassis_m3508_m2006_send_motor_currents_can1(void);
void chassis_m3508_send_motor_currents_can2(void);
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR);
void ad_plan_arm_motor_RPM_YAW(ARM_VELOCITY_PLANNING motion, 							float pos			)	;// 规划云台电机应有的RPM
void ad_plan_arm_motor_RPM_UP(ARM_VELOCITY_PLANNING motion, 							float pos			);// 规划抬升机构应有的RPM
void ad_plan_arm_motor_RPM_TRANSATE1(TRANSATE_VELOCITY_PLANNING motion, 							float pos			)	;


extern struct M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[4];//底盘
extern struct M3508_REAL_INFO M3508_UP_MOTOR_REAL_INFO[1];//抬升机构
extern struct M3508_REAL_INFO M3508_YAW_MOTOR_REAL_INFO[1];//云台机构
extern struct M3508_REAL_INFO M3508_TRANSATE_MOTOR_REAL_INFO[1];//传输机构

extern struct PID M3508_CHASSIS_MOTOR_PID_RPM[4];    			 // 驱动M3508电机
extern struct PID M3508_UP_MOTOR_PID_RPM[1];//抬升机构
extern struct PID M3508_RAIL_MOTOR_PID_RPM[1];//云台机构
extern struct PID M3508_TRANSATE_MOTOR_PID_RPM[1];//传输机构
extern struct PID M3508_UP_NORMAL;

extern struct M3508_CLASS M3508_UP;//抬升电机初始化 CANid为5//是一个类里面有 	M3508_REAL_INFO REAL_INFO; PID MOTOR_PID;	int TARGET_RPM;		int SET_CURRENT;
extern struct M3508_CLASS M3508_RAIL;//云台电机初始化 CANid为6
extern struct M3508_CLASS M3508_TRANSATE;//传递电机初始化 CANid为7
extern struct M3508_CLASS M3508_PICK_TRANSATE;//
extern struct M3508_CLASS M3508_PICK;//
#endif
