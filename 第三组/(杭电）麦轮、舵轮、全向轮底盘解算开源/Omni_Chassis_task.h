#ifndef CHASSIStaskH
#define CHASSIStaskH
#include "main.h"

#define LENGTH_A 200 //底盘长的一半
#define LENGTH_B 166 //底盘宽的一半
typedef enum
{
    CHASSIS_FOLLOW_GIMBAL = 0,	//底盘跟随云盘行走
    CHASSIS_GYROSCOPE = 1,			//小陀螺模式
    CHASSIS_NORMAL   = 2,//底盘不跟随云台行走
    CHASSIS_CORGI    = 3,//扭屁股模式
    CHASSIS_ROSHAN   = 4,//打符模式
    CHASSIS_SLOW     = 5,//补弹低速模式
    CHASSIS_SZUPUP   = 6,//爬坡模式
    CHASSIS_MISS     = 7,//自动闪避模式
    CHASSIS_PISA     = 8,//45°模式
} eChassisAction;
extern eChassisAction actChassis;

typedef struct
{
    float vx;
    float vy;
    float vw;
} Chassis_Speed;

void Chassis_open_init(void);
void CHASSIS_InitArgument(void);
void Omni_calc(Chassis_Speed *speed, int16_t* out_speed);
void Omni_angle_calc(Chassis_Speed *speed, float* out_angle) ;
void Omni_Set_Motor_Speed(int16_t*out_speed,Motortype* Motor );
void Absolute_Cal(Chassis_Speed* absolute_speed, float angle )	;
float Find_Y_AnglePNY(void);
float Find_min_Angle(int16_t angle1,fp32 angle2);
void RemoteControlChassis(void);
void CHASSIS_Single_Loop_Out(void); //底盘电机输出

/**
  * @brief  角度回环 浮点
  * @param  void
  * @retval 角度值，最大角度值
  * @attention 
  */
//void AngleLoop_f (float* angle ,float max){
//	while((*angle<-(max/2)) ||(*angle>(max/2)))
//	{
//		if(*angle<-(max/2))
//		{
//			*angle+=max;
//		}
//		else if(*angle>(max/2))
//		{
//			*angle-=max;
//		}
//  }
//}

//void AngleLoop_int (int16_t* angle ,int16_t max){
//	while((*angle<-(max/2)) ||(*angle>(max/2)))
//	{
//		if(*angle<-(max/2))
//		{
//			*angle+=max;
//		}
//		else if(*angle>(max/2))
//		{
//			*angle-=max;
//		}
//  }
//}

#endif



