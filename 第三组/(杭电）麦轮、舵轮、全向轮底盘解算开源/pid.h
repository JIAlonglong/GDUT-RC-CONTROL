#ifndef __bsp_pid
#define __bsp_pid
#include "type.h"
#include "stm32f4xx_hal.h"
#include "main.h"


typedef struct PidTypeDef
{
    float Dead_Zone; //���������ֵ
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set; //�趨ֵ
    float fdb; //����ֵ

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
    int angle_max;
    int angle_min;	//�Ƕ�����ֵ ����һ��Բ�ڣ�0���360�����ڣ���max=360��min=0
                        //			��һ������� 0��8192���ڣ���max=8192��min=0
    float I_Separation; //���ַ�����ֵ
    float gama;			//΢�������˲�ϵ��
    float lastdout;		//��һ��΢�����

    void (*f_param_init)(struct PidTypeDef* pid,  //PID������ʼ��
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
    fp32 (*f_cal_pid)(struct PidTypeDef* pid, const fp32 ref, const fp32 set);   //pid����
    void (*f_reset_pid)(struct PidTypeDef* pid, fp32 PID[3]);
    void (*f_clear_pid)(struct PidTypeDef* pid);

}PidTypeDef;

void pid_init(PidTypeDef *pid);

#endif
