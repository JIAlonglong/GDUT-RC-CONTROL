#ifndef CHASSIStaskH
#define CHASSIStaskH
#include "main.h"

#define LENGTH_A 200 //���̳���һ��
#define LENGTH_B 166 //���̿��һ��
typedef enum
{
    CHASSIS_FOLLOW_GIMBAL = 0,	//���̸�����������
    CHASSIS_GYROSCOPE = 1,			//С����ģʽ
    CHASSIS_NORMAL   = 2,//���̲�������̨����
    CHASSIS_CORGI    = 3,//Ťƨ��ģʽ
    CHASSIS_ROSHAN   = 4,//���ģʽ
    CHASSIS_SLOW     = 5,//��������ģʽ
    CHASSIS_SZUPUP   = 6,//����ģʽ
    CHASSIS_MISS     = 7,//�Զ�����ģʽ
    CHASSIS_PISA     = 8,//45��ģʽ
} eChassisAction;
extern eChassisAction actChassis;

typedef struct
{
    float vx;
    float vy;
    float vw;
} Chassis_Speed;

void Chassis_open_init(void);
void CHASSIS_InitArgument(void);
void Omni_calc(Chassis_Speed *speed, int16_t* out_speed);
void Omni_angle_calc(Chassis_Speed *speed, float* out_angle) ;
void Omni_Set_Motor_Speed(int16_t*out_speed,Motortype* Motor );
void Absolute_Cal(Chassis_Speed* absolute_speed, float angle )	;
float Find_Y_AnglePNY(void);
float Find_min_Angle(int16_t angle1,fp32 angle2);
void RemoteControlChassis(void);
void CHASSIS_Single_Loop_Out(void); //���̵�����

/**
  * @brief  �ǶȻػ� ����
  * @param  void
  * @retval �Ƕ�ֵ�����Ƕ�ֵ
  * @attention 
  */
//void AngleLoop_f (float* angle ,float max){
//	while((*angle<-(max/2)) ||(*angle>(max/2)))
//	{
//		if(*angle<-(max/2))
//		{
//			*angle+=max;
//		}
//		else if(*angle>(max/2))
//		{
//			*angle-=max;
//		}
//  }
//}

//void AngleLoop_int (int16_t* angle ,int16_t max){
//	while((*angle<-(max/2)) ||(*angle>(max/2)))
//	{
//		if(*angle<-(max/2))
//		{
//			*angle+=max;
//		}
//		else if(*angle>(max/2))
//		{
//			*angle-=max;
//		}
//  }
//}

#endif



