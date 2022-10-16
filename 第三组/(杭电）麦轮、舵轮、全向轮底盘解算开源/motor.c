 /*
 * motor.c
 * Date		    Author		Notes
 * 2019.11.25	Tongw    
 */
#include "motor.h"

//3508:1-4
Motortype Chassis_Motor[8];
Motortype Ammunition_Motor;
Motortype Gimbal_MotorYaw;
Motortype Gimbal_MotorPitch;

void Motor_Init(Motortype*motor,int ID,float pid1[3],const float outmax1,const float imax1,
                 float pid2[3],const float outmax2,const float imax2)//3508 ËÙ¶È
{
    motor->ID=ID;
    motor->motor_value=&moto_CAN[ID-1];

    pid_init(&(motor->Motor_PID_Position));
    pid_init(&(motor->Motor_PID_Speed));
    //max_out    max_iout   I_Separation   Dead_Zone  gama    angle_max     angle_min
    (motor->Motor_PID_Position).f_param_init(&(motor->Motor_PID_Position),PID_DELTA,     pid1,outmax1,   imax1,     3e38,          0,        0,      8192,         0 );
    (motor->Motor_PID_Speed).f_param_init(&(motor->Motor_PID_Speed),      PID_POSITION,  pid2,outmax2,   imax2,     3e38,          0,        0.1,    0,            0 );
}

void Motor_Init2(Motortype*motor,int ID,float pid1[3],const float outmax1,const float imax1,
                 float pid2[3],const float outmax2,const float imax2)//6020 ½Ç¶È
{
    motor->ID=ID;
    motor->motor_value=&moto_CAN2[ID-1];

    pid_init(&(motor->Motor_PID_Position));
    pid_init(&(motor->Motor_PID_Speed));
    //max_out    max_iout   I_Separation   Dead_Zone  gama    angle_max     angle_min
    (motor->Motor_PID_Position).f_param_init(&(motor->Motor_PID_Position),PID_POSITION,  pid1,outmax1,   imax1,     3e38,          0,        0,      8192,         0 );
    (motor->Motor_PID_Speed).f_param_init(&(motor->Motor_PID_Speed),      PID_POSITION,  pid2,outmax2,   imax2,     3e38,          0,        0.1,    0,            0 );
}
