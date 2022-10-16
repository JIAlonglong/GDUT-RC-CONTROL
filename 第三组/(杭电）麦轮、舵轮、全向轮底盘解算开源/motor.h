#ifndef bsp_motor_h
#define bsp_motor_h

#include "main.h"
#include "pid.h"
#include "type.h"

extern Motortype Chassis_Motor[8];

extern Motortype Ammunition_Motor;

extern Motortype Gimbal_MotorYaw;
extern Motortype Gimbal_MotorPitch;

void Motor_Init(Motortype*motor,int ID,float pid1[3],const float outmax1,const float imax1,
                 float pid2[3],const float outmax2,const float imax2);
void Motor_Init2(Motortype*motor,int ID,float pid1[3],const float outmax1,const float imax1,
                 float pid2[3],const float outmax2,const float imax2);
#endif
