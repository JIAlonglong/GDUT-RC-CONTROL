#include "includes.h"


// ��ʼ��PID����
// λ��ʽ�л����޷� ����ʽû�л����޷�
// λ��ʽ�г�ʼ���� ����ʽû�г�ʼ����
// λ��ʽ������ʽ������������ ������Ϊ�������������ܲ�����
void PID_parameter_init(PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone)  
{  
		pp->Integralmax = Integralmax;
	  pp->outputmax = outputmax;
	  pp->Proportion = Kp;
	  pp->Integral   = Ki;
	  pp->Derivative = Kd;
    pp->DError = pp->Error = pp->SumError = pp->output = pp->LastError = pp->PrevError = pp->errormax = 0.0f;
		pp->first_flag = 1;
		pp->deadzone = deadzone;
}  


// ����PID
void PID_reset_PID(PID *pp)
{
    pp->DError = pp->Error = pp->SumError = pp->output = pp->LastError = pp->PrevError = pp->errormax = 0.0f; 
		pp->first_flag = 1;
}


// �Ա������з�Χ����
float PID_abs_limit(float a, float ABS_MAX)
{
    if(a > ABS_MAX)
        a = ABS_MAX;
		
    if(a < -ABS_MAX)
        a = -ABS_MAX;
		return a;
}

// ����ʽPID
void PID_incremental_PID_calculation(PID *pp, float CurrentPoint, float NextPoint)  
{  
	pp->Error =  NextPoint - CurrentPoint;                               
	pp->DError = pp->Error - pp->LastError;
	
	pp->output +=  pp->Proportion * (pp->DError)+   \
								 PID_abs_limit(pp->Integral * pp->Error, pp->Integralmax ) +   \
								 pp->Derivative * ( pp->Error +  pp->PrevError - 2*pp->LastError);  

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  
		pp->output = -pp->outputmax;
	pp->PrevError = pp->LastError;  
	pp->LastError = pp->Error;
	
	if(ABS(pp->Error) < pp->deadzone)
	{
		pp->output = 0;
	}
}


// ����ʽPID,ֱ�Ӵ������
void PID_incremental_PID_calculation_by_error(PID *pp,  float error)  
{  
	pp->Error = error;                               
	pp->DError = pp->Error - pp->LastError;
	
	pp->output +=  pp->Proportion * (pp->DError)+   \
								 pp->Integral * pp->Error +  \
								 pp->Derivative * ( pp->Error +  pp->PrevError - 2*pp->LastError);  

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  
		pp->output = -pp->outputmax;
	pp->PrevError = pp->LastError;  
	pp->LastError = pp->Error;
	
	if(ABS(pp->Error) < pp->deadzone)
	{
		pp->output = 0;
	}
}


// λ��ʽPID
void PID_position_PID_calculation(PID *pp, float CurrentPoint, float NextPoint)  
{   
	if(pp->first_flag == 1)
	{
		pp->LastError = NextPoint - CurrentPoint;
		pp->PrevError = NextPoint - CurrentPoint;
		pp->first_flag = 0;
	}
	
	pp->Error =  NextPoint -  CurrentPoint;          
	pp->SumError += pp->Error;                      
	pp->DError = pp->Error - pp->LastError;
	
	pp->output =  pp->Proportion * pp->Error +   \
								PID_abs_limit(pp->Integral * pp->SumError, pp->Integralmax ) +   \
								pp->Derivative * pp->DError ;  

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax; 
	pp->LastError = pp->Error;
	
	if(ABS(pp->Error) < pp->deadzone)
	{
		pp->output = 0;
	}
}


// λ��ʽPID,ֱ�Ӵ������
void PID_position_PID_calculation_by_error(PID *pp, float error)  
{   
	if(pp->first_flag == 1)
	{
		pp->LastError = error;
		pp->PrevError = error;
		pp->first_flag = 0;
	}	
	
	pp->Error =  error;          
	pp->SumError += pp->Error;                      
	pp->DError = pp->Error - pp->LastError;
	
	pp->output =  pp->Proportion * pp->Error +   \
								PID_abs_limit(pp->Integral * pp->SumError, pp->Integralmax ) +   \
								pp->Derivative * pp->DError ;  

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax; 
	pp->LastError = pp->Error;
	
	if(ABS(pp->Error) < pp->deadzone)
	{
		pp->output = 0;
	}
}
