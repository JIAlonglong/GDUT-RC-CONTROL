#include "includes.h"
void MotorVelocityCurve_task(void)
{
	curve.speedMax=4000;
	curve.aTimes=1000;
	curve.p_add=0.1;
	curve.speedMin=100;
	curve.stepSpeed=40;
 MotorVelocityCurve(&curve,&M3508_CHASSIS_MOTOR_PID_RPM[0]);
 MotorVelocityCurve(&curve,&M3508_CHASSIS_MOTOR_PID_RPM[1]);
 MotorVelocityCurve(&curve,&M3508_CHASSIS_MOTOR_PID_RPM[2]);
 MotorVelocityCurve(&curve,&M3508_CHASSIS_MOTOR_PID_RPM[3]);	
	
vTaskDelay(5);
}
