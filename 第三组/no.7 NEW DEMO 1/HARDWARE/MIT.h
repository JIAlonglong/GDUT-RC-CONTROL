#ifndef __MIT_H_
#define __MIT_H_

#include "stm32f4xx.h" 

/*��ز���*/

//#define PI 3.1415928

#define AK80_ID1 0x01//Ҫʹ�õ�AK80ID
#define AK80_ID2 0x02
#define AK80_ID3 0x03
#define AK80_ID4 0x04

//��ʱ�����õ��ģʽ�ģ���ȻҲ������ö��
#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

//������ز��������޷�
#define P_MIN   -12.5f//position���λ��
#define P_MAX	 12.5f
#define V_MIN	-500.0f//����ٶ�
#define V_MAX	 500.0f
#define T_MIN 	-18.0f//���Ť��
#define T_MAX 	 18.0f
#define Kp_MIN 	 0//Kp��Χ
#define Kp_MAX   500.0f
#define Kd_MIN 	 0//Kd��Χ
#define Kd_MAX   5.0f
#define I_MAX  	 18.0f

/*����AK80_motion_control��**/
#define PITCH_MAX		90.0f
#define PITCH_MIN		-90.0f

//�˲�����������ʱδ֪
#define MIT_P_MIN						-12.5f
#define MIT_P_MAX						 12.5f
#define MIT_V_MIN						-500.0f
#define MIT_V_MAX						 500.0f

void AK80_Speed_Control(u8 ID,float vel);///�ٶ�ģʽ����AK80

//CAN���صĵ����ʵ����
typedef struct AK80_REAL_INFO
{
	float  ANGLE;   		        //�����Ƕ�	(rad)			
	float  V_angle;						  //�������ٶ�(rad/s	)	
	float  CURRENT;
	float  TARGET_CURRENT;
	float  TARGET_POS;
	float  TARGET_SPEED;
	float  REAL_ANGLE;         //���������ʵ�Ƕȣ�������float��
}AK80_REAL_INFO;

extern AK80_REAL_INFO MIT_DRIVER_REAL_INFO[3];

void AK80_Init(void);//��ʼ��AK80

float fmaxf(float a,float b);//a��bȡ���

float fminf(float a,float b);//a��bȡ��С

int float_to_uint(float x1,float x1_min,float x1_max,int bits);//����ת����

float uint_to_float(int x1_int,float x1_min,float x1_max,int bits);//����ת������

//void AK80_motion_control(u8 ID,float p_des,float v_des,float kp,float kd,float t_ff);//���Ƶ������

void ak80_control_cmd(uint8_t ID,uint8_t cmd);//ģʽ����
//int float_to_uint(float x,float x_min,float x_max,int bits);
void AK80_update_info(CanRxMsg *msg);//���ϸ�������







#endif
