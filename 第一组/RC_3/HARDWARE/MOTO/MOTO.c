#include "includeh.h"




M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[4] = {0}; // 1-4�ֱ��Ӧ˳ʱ�뷽��ĵ��̵��
M3508_REAL_INFO M3508_CAST_MOTOR_REAL_INFO 			 = {0};	// ����������

PID M3508_CHASSIS_MOTOR_PID_RPM[4];	// 1-4���̵��
PID M3508_CAST_MOTOR_PID_RPM;				



//M3580��ʼ��
void M3508_Motor_Init(void)
{
	// ���̵��PID��ʼ��
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[0], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[1], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[2], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[3], 10.0, 1.0, 0.0, 16384, 16384, -1);
	
	// ̧���������PID��ʼ��
	PID_parameter_init(&M3508_CAST_MOTOR_PID_RPM , 50, 0, 1, 7000, 7000, -1);
}


// ���õ��ͨ��CAN���������ݸ���m3508��״̬��Ϣ
// ����Ƶ�ʣ�1kHz
void m3508_update_m3508_info(CanRxMsg *msg)
{
	switch(msg -> StdId)  // ����׼ID
	{
    case M3508_CHASSIS_MOTOR_ID_1:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_2:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[1].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_3:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[2].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[2].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;	
		
		case M3508_CHASSIS_MOTOR_ID_4:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[3].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[3].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
		
		case M3508_CAST_MOTOR_ID:
		{ 
			M3508_CAST_MOTOR_REAL_INFO.ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CAST_MOTOR_REAL_INFO.RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CAST_MOTOR_REAL_INFO.CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;

		default: break;
	}
}



//���͵���
void chassis_m3508_send_motor_currents(void)
{
	CanTxMsg tx_message_1;

	// ���ÿ��ƶ�
	tx_message_1.IDE = CAN_Id_Standard;
	tx_message_1.RTR = CAN_RTR_Data;
	tx_message_1.DLC = 0x08;
	
	// �����ٲöκ����ݶ�	
	tx_message_1.StdId = 0x200;  // ����IDΪ 1 2 3 4 �ĵ��
	tx_message_1.Data[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	tx_message_1.Data[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT;
	tx_message_1.Data[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	tx_message_1.Data[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT;
	tx_message_1.Data[4] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	tx_message_1.Data[5] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT;
//	tx_message_1.Data[6] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
//	tx_message_1.Data[7] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT;

	CAN_Transmit(CAN1, &tx_message_1);
}

//���Ƶ���
void shoot_m3508_send_motor_currents(void)
{
	CanTxMsg tx_message_2;
	
	// ���ÿ��ƶ�
	tx_message_2.IDE = CAN_Id_Standard;//��׼ID
	tx_message_2.RTR = CAN_RTR_Data; //����֡
	tx_message_2.DLC = 0x08;
	
	// �����ٲöκ����ݶ�
	tx_message_2.StdId = 0x1FF;  // ����IDΪ 5 6 7 8 �ĵ��
	tx_message_2.Data[0] = (uint8_t)(M3508_CAST_MOTOR_REAL_INFO.TARGET_CURRENT >> 8);
	tx_message_2.Data[1] = (uint8_t) M3508_CAST_MOTOR_REAL_INFO.TARGET_CURRENT;	
	
	CAN_Transmit(CAN1, &tx_message_2);
}


//M3508����ǶȻ���
// M3508����ǶȻ���
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR)
{
	float delta_pos = 0;
	
	// ��¼��һ�ν���ʱ������
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// ����仯�ĽǶ�
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
			}
		}
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
		}
		
		// �˲�
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����	
	}
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
		}
		
		// �˲�
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����
	}

	// �洢�Ƕ�ֵ 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}

     
///********����ġ������޸ĵ�M3508·������****************************************************/
//void M3508AngleIntegral_special_up(M3508_REAL_INFO *M3508_MOTOR)  
//{
//	float delta_pos = 0;
//	
//	// ��¼��һ�ν���ʱ������
//	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG||Robot_State.UP==BOTTOM||Robot_State.UP==TOP)
//	{
//		delta_pos = 0;
//		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
//		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
//		return;
//	}
//	
//	// ����仯�ĽǶ�
//	if(M3508_MOTOR->RPM >= 0)
//	{
//		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
//		{
//			if(abs(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�
//			{
//				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
//				delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
//			}
//		}
//		else
//		{
//			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
//			delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
//		}
//		
//		// �˲�
//		if(delta_pos > 0) 
//			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����	
//	}
//	else
//	{
//		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
//		{
//			if(abs(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�			
//			{
//				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
//				delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
//			}
//		}	
//		else
//		{
//			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
//			delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
//		}
//		
//		// �˲�
//		if(delta_pos < 0) 
//			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����
//	}

//	// �洢�Ƕ�ֵ
//	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
//}


