#ifndef __GLOBAL_LOCATION_H
#define __GLOBAL_LOCATION_H

// �����˵���ʵλ��
typedef struct ROBOT_REAL_POS
{
  float POS_X;
  float POS_Y;     
  float POS_YAW;
}ROBOT_REAL_POS;

//// ����ȫ����λģ�鶨λ��λ��
//typedef struct ACTION_GL_POS
//{
//	float ANGLE_Z;
//	float ANGLE_X;
//	float ANGLE_Y;	
//	float POS_X;
//	float POS_Y;
//	float W_Z;
//	
//	float LAST_POS_X;
//	float LAST_POS_Y;
//	
//	float DELTA_POS_X;
//	float DELTA_POS_Y;	
//}ACTION_GL_POS;

extern float OFFSET_YAW;
extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;

#endif








