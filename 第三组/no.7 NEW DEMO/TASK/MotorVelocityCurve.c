#include "includes.h"
void MotorVelocityCurve_task(void)
{
	curve.speedMax=4000;
	curve.aTimes=1000;
	curve.p_add=0.1;
	curve.speedMin=100;
	curve.stepSpeed=40;
vTaskDelay(5);
}
