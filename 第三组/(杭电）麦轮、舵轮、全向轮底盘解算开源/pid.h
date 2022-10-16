#ifndef __bsp_pid
#define __bsp_pid
#include "type.h"
#include "stm32f4xx_hal.h"
#include "main.h"


typedef struct PidTypeDef
{
    float Dead_Zone; //误差死区阈值
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set; //设定值
    float fdb; //反馈值

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
    int angle_max;
    int angle_min;	//角度相邻值 如在一个圆内，0°和360°相邻，则max=360，min=0
                        //			在一个电机内 0和8192相邻，则max=8192，min=0
    float I_Separation; //积分分离阈值
    float gama;			//微分先行滤波系数
    float lastdout;		//上一次微分输出

    void (*f_param_init)(struct PidTypeDef* pid,  //PID参数初始化
        uint8_t mode,
        fp32 PID[3],
        fp32 max_out,
        fp32 max_iout,
        float I_Separation,
        float Dead_Zone,
        float gama,
        int angle_max,
        int angle_min

        );

    void (*f_param_init)(PidTypeDef* pid, uint8_t mode, const float PID[3], float max_out, float max_iout, float I_Separation, float Dead_Zone, float gama, int angle_max, int angle_min);
    fp32 (*f_cal_pid)(struct PidTypeDef* pid, const fp32 ref, const fp32 set);   //pid计算
    void (*f_reset_pid)(struct PidTypeDef* pid, fp32 PID[3]);
    void (*f_clear_pid)(struct PidTypeDef* pid);

}PidTypeDef;

void pid_init(PidTypeDef *pid);

#endif
