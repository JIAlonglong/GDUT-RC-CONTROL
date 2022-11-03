#ifndef _CONTROL_H
#define _CONTROL_H
void MotorVelocityCurve(CurveObjectType *curve,PID *M3508,M3508_REAL_INFO *M3508_REAL);
void World_3wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta);
void move(void);
void Free_Control(void);
void Free_Control_Limit(void);
void free_up(void);
void RPM_MOTOR_PLANNING(void);
void PUSH(float start,float end,float speedmax,float speedstart,float speedend,float ac,float de);
#endif
