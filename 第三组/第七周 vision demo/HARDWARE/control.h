#ifndef _CONTROL_H
#define _CONTROL_H
void MotorVelocityCurve(CurveObjectType *curve,PID *M3508);
void World_3wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta);
void move(void);
#endif
