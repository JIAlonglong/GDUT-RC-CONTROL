#ifndef __MOTOR_H
#define __MOTOR_H
#include "can.h"
#include "PID.h"


// M3508������
#define M3508_CHASSIS_MOTOR_ID_1      0x201
#define M3508_CHASSIS_MOTOR_ID_2      0x202
#define M3508_CHASSIS_MOTOR_ID_3      0x203
#define M3508_CHASSIS_MOTOR_ID_4      0x204
#define M3508_CAST_MOTOR_ID	        	0x205

typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   		        //�����Ƕ�						
	int16_t  RPM;					//�ٶ�ֵ			
	int16_t  CURRENT;     //����ֵ
	int16_t  TARGET_CURRENT;//Ŀ�����ֵ
	
	// �ǶȻ���ʱ�õ��������
	float		 REAL_ANGLE;         //���������ʵ�Ƕȣ�������float��
	u8			 FIRST_ANGLE_INTEGRAL_FLAG;  //?
	uint16_t LAST_ANGLE;   //?
}M3508_REAL_INFO;


//�������߹滮�Ľṹ��
/* �������ٶ����߶��� */
typedef struct CurveObject {
  float startSpeed;    //��ʼ����ʱ�ĳ�ʼ�ٶ�
  float currentSpeed;   //��ǰ�ٶ�
  float targetSpeed;    //Ŀ���ٶ�
  float stepSpeed;     //���ٶ�
  float speedMax;     //����ٶ�
  float speedMin;     //��С�ٶ�
  uint32_t aTimes;     //����ʱ��
  uint32_t maxTimes;    //���ٿ��
	float  p_add;    //���ٵ�ռ��
	float  p_decrease; //���ٵ�ռ��
  
}CurveObjectType;



extern struct PID M3508_CAST_MOTOR_PID_RPM;				// ����������
extern struct PID M3508_CHASSIS_MOTOR_PID_RPM[4];	// 1-4���̵��
extern struct M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[4]; 

void M3508_Motor_Init(void);
void m3508_update_m3508_info(CanRxMsg *msg);
void chassis_m3508_send_motor_currents(void);
void shoot_m3508_send_motor_currents(void);
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR);

#endif

