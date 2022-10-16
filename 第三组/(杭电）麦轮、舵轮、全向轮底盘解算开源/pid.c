/*
 * bsp_pid.c
 * Date			  Author			Notes
 * 2019.10.18	顾南小桑    带死区、抗积分饱和、梯形积分、变积分算法以及不完全微分算法的增量式、位置式PID控制器
 * 2021.2.18	YQY	        增加了梯形积分和变积分算法
 *                        See：https://blog.csdn.net/foxclever/article/details/80778748
 */
#include "pid.h"
#include "type.h"
#include "stm32f4xx.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

static float VariableIntegralCoefficient(float error, float absmax, float absmin);
/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout,  float I_Separation, float Dead_Zone, float gama, int angle_max, int angle_min)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    if(fabs(pid->Dead_Zone) < 1e-5)
        pid->Dead_Zone = 0;
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->I_Separation = I_Separation;
    pid->Dead_Zone = Dead_Zone;
    pid->gama = gama;
    pid->angle_max = angle_max;
    pid->angle_min = angle_min;
}

static fp32 pid_caculate(PidTypeDef *pid, const fp32 ref, const fp32 set)
{
    float factor;
    uint8_t index;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[0] = set - ref;

    if(pid->angle_max != pid->angle_min)
    {
        if( pid->error[0] > (pid->angle_max + pid->angle_min) / 2)
            pid->error[0] -= (pid->angle_max + pid->angle_min);
        else if( pid->error[0] < -(pid->angle_max + pid->angle_min) / 2)
            pid->error[0] += (pid->angle_max + pid->angle_min);
    }

    pid->set = set;
    pid->fdb = ref;//feedback

    if(fabs(pid->error[0]) < pid->Dead_Zone)	//死区
    {
        pid->error[0] = pid->error[1] = 0;
    }
    if(fabs(pid->error[0]) > pid->I_Separation) //误差过大，采用积分分离
    {
        index = 0;
    }
    else
    {
        index = 1;
    }

    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];

        pid->Iout += pid->Ki * pid->error[0];

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);

        //计算微分项增量带不完全微分
        pid->Dout = pid->Kd * (1 - pid-> gama) * (pid->Dbuf[0]) + pid-> gama * pid-> lastdout;
        LimitMax(pid->Iout, pid->max_iout);

        pid->out = pid->Pout + index * pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {

        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        //pid->Iout = pid->Ki * pid->error[0];
        //变积分系数获取
        factor = VariableIntegralCoefficient(pid->error[0], pid->I_Separation, 0);

        pid->Iout = pid->Ki * factor * (pid->error[0] + pid->error[1]) / 2.0f;
        LimitMax(pid->Iout, pid->max_iout);

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->out += pid->Pout + pid->Iout + pid->Dout;

        /*对输出限值，避免超调和积分饱和问题*/
        LimitMax(pid->out, pid->max_out);

    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid-> lastdout = pid->Dout;
    pid-> lastout = pid->out;
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/*中途更改参数设定(调试)------------------------------------------------------------*/
void pid_reset(PidTypeDef	*pid, fp32 PID[3])
{
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
}

/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
void pid_init(PidTypeDef *pid)
{
    pid->f_param_init = pid_param_init;
    pid->f_cal_pid = pid_caculate;
    pid->f_reset_pid = pid_reset;
    pid->f_clear_pid = PID_clear;
}

/*变积分系数获取-----------------------------------------------------------------*/
static float VariableIntegralCoefficient(float error, float absmax, float absmin)
{
    float factor = 0.0;
    if(fabs(error) <= absmin)
    {
        factor = 1.0;
    }
    else if(fabs(error) > absmax)
    {
        factor = 0.0;
    }
    else
    {
        factor = (absmax - fabs(error)) / (absmax - absmin);
    }
    return factor;
}
