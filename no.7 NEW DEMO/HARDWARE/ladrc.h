#ifndef _LADRC_H
#define _LADRC_H
#include "stm32f4xx.h"
/**
   *@Brief  ����ΪLADRCϵͳ����
   *@WangShun  2022-07-03  ע��
   */
typedef struct LADRC
{
    float v1,v2;         //�������ֵ
    float r;             //�ٶ�����
    float h;             //���ֲ���
    float z1,z2,z3;      //�۲������
    float w0,wc,b0,u;    //�۲������� ���������� ϵͳ���� ���������
	  float  PrevError;          //  Error[-2]
    float  LastError;          //  Error[-1]  
	  float  Error;
	  float  DError;
    float  SumError;           //  Sums of Errors  
	  float  output;             //���õ����Ĵ�С
	  float  outputmax;          //�����޷�
	  float  errormax;           //����޷�
	  u8 first_flag;
	  float  deadzone;           //����
}LADRC_NUM;

/*
	wu = 2*3.1415/Pu;
    ku = 4*h/3.1415*a;

	wc = 2.3997wu - 0.4731;
	w0 = 0.7332wu + 3.5070;
	b0 = 3.6105wu + 4.8823;
*/
extern LADRC_NUM ADRC_M3508_CHASIS[3];
extern LADRC_NUM ADRC_M3508_UP;
extern LADRC_NUM ADRC_M3508_YAW;
extern LADRC_NUM ADRC_M3508_TRANSATE;

typedef struct Auto_Tuning 
{
	float Pu; //�̵�ʵ���������
	float a;  //�̵�ʵ�������ֵ
	float h;  //ָ�������ֵ
	float Wu; //ϵͳ�ٽ�Ƶ��
	float Kp; //ϵͳ�ٽ��ֵ
}AuTu;

/**
   *@Brief  ����ΪLADRC��غ���
   *@WangShun  2022-07-03  ע��
   */
void LADRC_Init(LADRC_NUM *LADRC_TYPE1,float h,float r,float wc,float w0,float b0,float outputmax, float deadzone);
void LADRC_REST(LADRC_NUM *LADRC_TYPE1);
void LADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect);
void LADRC_ESO(LADRC_NUM *LADRC_TYPE1,float FeedBack);
void LADRC_LF(LADRC_NUM *LADRC_TYPE1);
void LADRC_Loop(LADRC_NUM *LADRC_TYPE1,float RealTimeOut,float Expect);
#endif
