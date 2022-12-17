#ifndef __M3508_H
#define __M3508_H
#include "stm32f4xx.h"
extern int start,end;
extern int transate_finished;
extern int up_finished;
extern 	float angle_1;
// M3508���صĵ����ʵ��Ϣ
typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   		        //�����Ƕ�						
	int16_t  RPM;
  int16_t  TARGET_RPM;	
	int16_t  CURRENT;
	int16_t  TARGET_CURRENT;
	
	// �ǶȻ���ʱ�õ��������
	float		 REAL_ANGLE;         //���������ʵ�Ƕȣ�������float��
	u8			 FIRST_ANGLE_INTEGRAL_FLAG;
	uint16_t LAST_ANGLE;
	
	
}M3508_REAL_INFO;

//�������߹滮�Ľṹ��
/* �������ٶ����߶��� */
typedef struct CurveObjectType {
	float distance;       //����
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

typedef struct M3508_CLASS
{
	M3508_REAL_INFO REAL_INFO; 
	PID MOTOR_PID;	
	int TARGET_RPM;								
	int SET_CURRENT;
	int TARGET_ANGLE;
}M3508_CLASS;

typedef struct ARM_VELOCITY_PLANNING //�ٶȹ滮
{
	float Pstart;        //��ʼλ��
	float Pend;          //����λ��
	float Vstart;        //��ʼ���ٶ�           // ��λ��RPM ����ֵ
	float Vmax;          //�����ٶ�
	float Vend;          //ĩβ���ٶ�
	float Rac;           //����·�̵ı���
	float Rde;           //����·�̵ı���
}ARM_VELOCITY_PLANNING;

typedef struct TRANSATE_VELOCITY_PLANNING //�ٶȹ滮
{
	float Distance;
	float Pstart;        //��ʼλ��
	float Pend;          //����λ��
	float Vstart;        //��ʼ���ٶ�           // ��λ��RPM ����ֵ
	float Vmax;          //�����ٶ�
	float Vend;          //ĩβ���ٶ�
	float Rac;           //����·�̵ı���
	float Rde;           //����·�̵ı���
}TRANSATE_VELOCITY_PLANNING;

extern struct ARM_VELOCITY_PLANNING  *UP_ARM_NOW_MOTION;		 // ָ��̧����ǰ����
extern struct ARM_VELOCITY_PLANNING   UP_INIT;//̧��������ʼ��
extern struct ARM_VELOCITY_PLANNING   UP_ON1;//̧��
extern struct ARM_VELOCITY_PLANNING   UP_ON2;//̧��
extern struct ARM_VELOCITY_PLANNING   UP_ON3;//̧��
extern struct ARM_VELOCITY_PLANNING   UP_ON4;
extern struct ARM_VELOCITY_PLANNING   UP_ON5;
extern struct ARM_VELOCITY_PLANNING   UP_ON6;
extern struct ARM_VELOCITY_PLANNING   UP_DOWN3;//�½�

extern struct ARM_VELOCITY_PLANNING  *YAW_ARM_NOW_MOTION;		 // ָ����̨��ǰ����
extern struct ARM_VELOCITY_PLANNING   YAW_INIT;//��̨������ʼ��
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_HANDLE;//��̨�����ֶ�ת  VRB
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_AUTO;//��̨�����Զ�ת(vision��)
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_1;//��̨�����̶���ת1
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_2;//��̨�����̶���ת2
extern struct ARM_VELOCITY_PLANNING   YAW_MOVE_3;//��̨�����̶���ת3

extern struct TRANSATE_VELOCITY_PLANNING *TRANSATE_NOW_MOTION; //���ݻ��ĵ����˼·һ�����涨���룩
																															//	˼·�� ��ʼ�ͽ�����λ���ۼ��ϴε�λ��ֵ
extern struct TRANSATE_VELOCITY_PLANNING TRANSATE_INIT;
extern struct TRANSATE_VELOCITY_PLANNING TRANSATE_1;
extern struct CurveObjectType TRANSATE;

extern int16_t UP_MOTOR_TARGET_RPM ;    // ̧�����Ŀ���ٶ�
extern int16_t YAW_MOTOR_TARGET_RPM ;    // ת����Ŀ���ٶ�
extern int16_t TRANSATE_MOTOR_TARGET_RPM;
extern int16_t PICK_TRANSATE_MOTOR_TARGET_RPM;

void M3508_Motor_Init(void);
void m3508_update_m3508_info_can1(CanRxMsg *msg);
void m3508_update_m3508_info_can2(CanRxMsg *msg);
void chassis_m3508_send_motor_currents_can1(void);
void chassis_m3508_m2006_send_motor_currents_can1(void);
void chassis_m3508_send_motor_currents_can2(void);
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR);
void ad_plan_arm_motor_RPM_YAW(ARM_VELOCITY_PLANNING motion, 							float pos			)	;// �滮��̨���Ӧ�е�RPM
void ad_plan_arm_motor_RPM_UP(ARM_VELOCITY_PLANNING motion, 							float pos			);// �滮̧������Ӧ�е�RPM
void ad_plan_arm_motor_RPM_TRANSATE1(TRANSATE_VELOCITY_PLANNING motion, 							float pos			)	;


extern struct M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[4];//����
extern struct M3508_REAL_INFO M3508_UP_MOTOR_REAL_INFO[1];//̧������
extern struct M3508_REAL_INFO M3508_YAW_MOTOR_REAL_INFO[1];//��̨����
extern struct M3508_REAL_INFO M3508_TRANSATE_MOTOR_REAL_INFO[1];//�������

extern struct PID M3508_CHASSIS_MOTOR_PID_RPM[4];    			 // ����M3508���
extern struct PID M3508_UP_MOTOR_PID_RPM[1];//̧������
extern struct PID M3508_RAIL_MOTOR_PID_RPM[1];//��̨����
extern struct PID M3508_TRANSATE_MOTOR_PID_RPM[1];//�������
extern struct PID M3508_UP_NORMAL;

extern struct M3508_CLASS M3508_UP;//̧�������ʼ�� CANidΪ5//��һ���������� 	M3508_REAL_INFO REAL_INFO; PID MOTOR_PID;	int TARGET_RPM;		int SET_CURRENT;
extern struct M3508_CLASS M3508_RAIL;//��̨�����ʼ�� CANidΪ6
extern struct M3508_CLASS M3508_TRANSATE;//���ݵ����ʼ�� CANidΪ7
extern struct M3508_CLASS M3508_PICK_TRANSATE;//
extern struct M3508_CLASS M3508_PICK;//
#endif
