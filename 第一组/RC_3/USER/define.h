#ifndef __DEFINES_H
#define __DEFINES_H

#define PI 								 3.14159265358979f
#define COS45              0.70710678f
#define SIN45              0.70710678f
#define ABS(x)      ((x)>0? (x):(-(x)))

// Chassis Config
#define WHEEL_R            0.076f	                  //���Ӱ뾶(��λ��m) 
#define Robot_R            0.456f                  	//���ֵ����ľ���(��λ��m)
#define M3508_RM_To_MS     (PI*WHEEL_R)/570.0f      //ת�����ٶȵ�ת�� (��λ��m/s) 
#define M3508_MS_To_RM    1/(PI*WHEEL_R)      //�ٶ���ת�ٵ�ת�� (��λ��m/s)  
#define MS_TO_RPM          21*60/(PI*WHEEL_R*2)     //����ֱ��152mm��������ٱ�1:21������һȦpi*152mm\

#endif
