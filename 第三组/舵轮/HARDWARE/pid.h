#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

typedef struct PID 
{
  float  Proportion;         //Kp
  float  Integral;           //KI
  float  Derivative;         //KD  
	float  PrevError;          //  Error[-2]
  float  LastError;          //  Error[-1]  
	float  Error;
	float  DError;
  float  SumError;           //  Sums of Errors  
	float  Integralmax;        //KI的最大值
	float  output;             //设置电流的大小
	float  outputmax;          //电流限幅
	float  errormax;           //误差限幅
	u8 first_flag;
	float  deadzone;           //死区
}PID;

void PID_parameter_init(PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone);
float PID_abs_limit(float a, float ABS_MAX);
void PID_incremental_PID_calculation(PID *pp,  float CurrentPoint, float NextPoint);
void PID_incremental_PID_calculation_by_error(PID *pp, float error);
void PID_position_PID_calculation(PID *pp, float CurrentPoint, float NextPoint);
void PID_position_PID_calculation_by_error(PID *pp, float error);
void PID_reset_PID(PID *pp);


#endif
