#ifndef __MOTOR_H
#define __MOTOR_H
#include "can.h"
#include "PID.h"


// M3508µç»ú±àºÅ
#define M3508_CHASSIS_MOTOR_ID_1      0x201
#define M3508_CHASSIS_MOTOR_ID_2      0x202
#define M3508_CHASSIS_MOTOR_ID_3      0x203
#define M3508_CHASSIS_MOTOR_ID_4      0x204
#define M3508_CAST_MOTOR_ID	        	0x205

typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   		        //²ÉÑù½Ç¶È						
	int16_t  RPM;					//ËÙ¶ÈÖµ			
	int16_t  CURRENT;     //µçÁ÷Öµ
	int16_t  TARGET_CURRENT;//Ä¿±êµçÁ÷Öµ
	int16_t  TARGET_ANGLE;//Ä¿±êµçÁ÷Ö
	
	// ½Ç¶È»ı·ÖÊ±ÓÃµ½ÏÂÃæ±äÁ¿
	float		 REAL_ANGLE;         //´¦Àí¹ıµÄÕæÊµ½Ç¶È£¨±ØĞëÓÃfloat£©
	u8			 FIRST_ANGLE_INTEGRAL_FLAG;  //?
	uint16_t LAST_ANGLE;   //?
}M3508_REAL_INFO;





//ÓÃÓÚÇúÏß¹æ»®µÄ½á¹¹Ìå
/* ¶¨Òåµç»úËÙ¶ÈÇúÏß¶ÔÏó */
typedef struct CurveObject {
  float startSpeed;    //¿ªÊ¼µ÷ËÙÊ±µÄ³õÊ¼ËÙ¶È
  float currentSpeed;   //µ±Ç°ËÙ¶È
  float targetSpeed;    //Ä¿±êËÙ¶È
  float stepSpeed;     //¼ÓËÙ¶È
  float speedMax;     //×î´óËÙ¶È
  float speedMin;     //×îĞ¡ËÙ¶È
  uint32_t aTimes;     //µ÷ËÙÊ±¼ä
  uint32_t maxTimes;    //µ÷ËÙ¿ç¶È
	float  p_add;    //¼ÓËÙµÄÕ¼±È
	float  p_decrease; //¼õËÙµÄÕ¼±È
  
}CurveObjectType;



extern struct PID M3508_CAST_MOTOR_PID_RPM;				// Éä¼ı»ú¹¹µç»ú
extern struct PID M3508_CHASSIS_MOTOR_PID_RPM[4];	// 1-4µ×ÅÌµç»ú
extern struct M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[4]; 

void M3508_Motor_Init(void);
void m3508_update_m3508_info(CanRxMsg *msg);
void chassis_m3508_send_motor_currents(void);
void shoot_m3508_send_motor_currents(void);
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR);
int	CAST_ANGLE(float target_angle);

#endif

