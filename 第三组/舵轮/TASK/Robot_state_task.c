#include "includes.h"
int speed_factor;
void Robot_state_task(void *pvParamerters)//������״̬
{
	while(1)
	{

//		if(AIR_R_SHORT<1500)//��������ģʽ
		if(AIR_R_SHORT>1500)//�����Զ�ģʽ
		{
		/***********����Ϊ�滮·��ģʽ*********************************/	
//			if(AIR_L_LONG<1400)//�Զ�����ȡ����
//			{
			MOVE_STATE=MOVE_1_CAST_POINT;

//			}
//			if(1400<AIR_L_LONG&AIR_L_LONG<1600)//�Զ�������1
//			{
////			MOVE_STATE=MOVE_1_SHOOT;

//			
//			}
//			if(AIR_L_LONG>1600)//��1��������
//			{
////			MOVE_STATE=MOVE_1_RESTART;

//			}
		
		}
		vTaskDelay(5);
		
	}
}
