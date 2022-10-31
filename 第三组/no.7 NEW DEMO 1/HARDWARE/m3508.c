#include "includes.h"

/*********************
 velocity �����˵��ٶ� 
 speed    �����rpm 
 RPM      ���ת��rpm
 *********************/
int start=0;
int end=0;
M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[3] = {0};

PID M3508_CHASSIS_MOTOR_PID_RPM[3];	// 3��M3508���
PID M3508_CAST_MOTOR_PID_RPM;	


//̧����������̨����
int16_t UP_MOTOR_TARGET_RPM ;     // ̧�����Ŀ���ٶ� 
int16_t YAW_MOTOR_TARGET_RPM ;    // ��̨���Ŀ���ٶ�
int16_t TRANSATE_MOTOR_TARGET_RPM;

M3508_CLASS M3508_UP;//̧�������ʼ�� CANidΪ5//��һ���������� 	M3508_REAL_INFO REAL_INFO; PID MOTOR_PID;	int TARGET_RPM;		int SET_CURRENT;
M3508_CLASS M3508_YAW;//��̨�����ʼ�� CANidΪ6
M3508_CLASS M3508_TRANSATE;//���ݵ����ʼ�� CANidΪ7


ARM_VELOCITY_PLANNING  *UP_ARM_NOW_MOTION;		 // ָ��̧����ǰ����
//								  ��ʼλ��  ����λ��    ��ʼ���ٶ�(RPM ����ֵ)  �����ٶ�	 ĩβ���ٶ�   ����·�̵ı��� ����·�̵ı���
ARM_VELOCITY_PLANNING   UP_INIT={0};//ʵ����
ARM_VELOCITY_PLANNING   UP_ON1	 ={0,        -4000,          2500,                3500,            0,          0.4,         0.3};
ARM_VELOCITY_PLANNING   UP_ON2	 ={-4000,        -8000,          2500,                3500,            0,          0.4,         0.3};
ARM_VELOCITY_PLANNING   UP_ON3	 ={-8000,       -10000,         2500,             3500,            0,          0.4,         0.2};
ARM_VELOCITY_PLANNING   UP_DOWN3={-10000,           0,         2500,             3500,            0,          0.2,         0.3};
	

ARM_VELOCITY_PLANNING  *YAW_ARM_NOW_MOTION;		 // ָ����̨��ǰ����
//								  ��ʼλ��  ����λ��    ��ʼ���ٶ�(RPM ����ֵ)  �����ٶ�	 ĩβ���ٶ�   ����·�̵ı��� ����·�̵ı���	
ARM_VELOCITY_PLANNING   YAW_INIT={0};
ARM_VELOCITY_PLANNING   YAW_MOVE_HANDLE={0};	
ARM_VELOCITY_PLANNING   YAW_MOVE_AUTO={0};
ARM_VELOCITY_PLANNING   YAW_MOVE_1={0};
ARM_VELOCITY_PLANNING   YAW_MOVE_2={0};
ARM_VELOCITY_PLANNING   YAW_MOVE_3={0};	

TRANSATE_VELOCITY_PLANNING *TRANSATE_NOW_MOTION; //ָ�򴫵ݵ���ĵ�ǰ����
//							��·��	 ��ʼλ��  ����λ��    ��ʼ���ٶ�(RPM ����ֵ)  �����ٶ�	 ĩβ���ٶ�   ����·�̵ı��� ����·�̵ı���	
TRANSATE_VELOCITY_PLANNING TRANSATE_INIT={0};
TRANSATE_VELOCITY_PLANNING TRANSATE_1={2000,0 , 0	,0		,0		,0		,0		,0};//0 , 2000		,1200		,5000		,0		,0.2		,0.4
/*float distance;       //����
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
*/
CurveObjectType TRANSATE={2000,0,0,500,30,600,0,1000,0,0.3,0.7};
// M3508��ʼ��
void M3508_Motor_Init(void)
{
	// ���ֵ��̵���ٶȻ�PID��ʼ��
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[0], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[1], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[2], 10.0, 1.0, 0.0, 16384, 16384, -1);	
	
	//Ҫ�Ƕȿ��Ƶĵ��
	PID_parameter_init(&M3508_UP.MOTOR_PID, 10.0, 1.0, 0.0, 16384, 16384,-0.1);
	PID_parameter_init(&M3508_YAW.MOTOR_PID, 10.0, 1.0, 0.0, 16384, 16384,-0.1);
	//��Ҫs�͹滮�ĵ��
	PID_parameter_init(&M3508_TRANSATE.MOTOR_PID, 10.0, 1.0, 0.0, 16384, 16384,-0.1);
	
	/****LADRC****/
	//���ֵ���
	LADRC_Init(&ADRC_M3508_CHASIS[0],0.005,20,100,400,0.5,16384, -1);
	LADRC_Init(&ADRC_M3508_CHASIS[1],0.005,20,100,400,0.5,16384,  -1);
	LADRC_Init(&ADRC_M3508_CHASIS[2],0.005,20,100,400,0.5,16384,  -1);
	//Ҫ�Ƕȿ��Ƶĵ��
	LADRC_Init(&ADRC_M3508_UP,0.005,20,100,400,0.5,16384, -1);
	//��Ҫs�͹滮�ĵ��
	LADRC_Init(&ADRC_M3508_TRANSATE,0.005,20,100,400,0.5,16384, -1);
	
}


//can�жϴ������
// ���õ��ͨ��CAN���������ݸ���m3508��״̬��Ϣ
// ����Ƶ�ʣ�1kHz
void m3508_update_m3508_info_can1(CanRxMsg *msg)
{
	switch(msg -> StdId)  // ����׼ID
	{
    case M3508_CHASSIS_MOTOR_ID_1://A����
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
			M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[0]);
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_2://B����
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[1].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
			M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[1]);
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_3://C����
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[2].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[2].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
			M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[2]);
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_4://D����
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[3].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[3].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
			M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[3]);
		}; break;

		case M3508_UP_MOTOR_ID_5://̧������
		{
			M3508_UP.REAL_INFO.ANGLE    =(msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_UP.REAL_INFO.RPM      =(msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_UP.REAL_INFO.CURRENT  =(msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
			M3508AngleIntegral(&M3508_UP.REAL_INFO);
			
		};break;
		case M3508_YAW_MOTOR_ID_6://��̨����
		{
			M3508_YAW.REAL_INFO.ANGLE    =(msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_YAW.REAL_INFO.RPM      =(msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_YAW.REAL_INFO.CURRENT  =(msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
			M3508AngleIntegral(&M3508_YAW.REAL_INFO);
		};break;
		case M3508_TRANSATE_MOTOR_ID_7://�������
		{
			M3508_TRANSATE.REAL_INFO.ANGLE    =(msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_TRANSATE.REAL_INFO.RPM      =(msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_TRANSATE.REAL_INFO.CURRENT  =(msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
			M3508AngleIntegral(&M3508_TRANSATE.REAL_INFO);
		};break;	
		default: break;
	}

}


// ����3508�ĵ�����С
void chassis_m3508_send_motor_currents_can1(void)
{
	CanTxMsg tx_message_1;

	// ���ÿ��ƶ�
	tx_message_1.IDE = CAN_Id_Standard;
	tx_message_1.RTR = CAN_RTR_Data;
	tx_message_1.DLC = 0x08;//0x02����һ�����
	
	// �����ٲöκ����ݶ�	
	tx_message_1.StdId = 0x200;  // ����IDΪ 1 2 3  �ĵ��
	tx_message_1.Data[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	tx_message_1.Data[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT;
	tx_message_1.Data[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	tx_message_1.Data[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT;
	tx_message_1.Data[4] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	tx_message_1.Data[5] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT;
	tx_message_1.Data[6] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	tx_message_1.Data[7] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT;

	CAN_Transmit(CAN1, &tx_message_1);
}

void chassis_m3508_m2006_send_motor_currents_can1(void)
{
	CanTxMsg tx_message_2;
	
	// ���ÿ��ƶ�
	tx_message_2.IDE = CAN_Id_Standard;
	tx_message_2.RTR = CAN_RTR_Data;
	tx_message_2.DLC = 0x08;
	
	// �����ٲöκ����ݶ�
	tx_message_2.StdId = 0x1FF;  // ����IDΪ 5 6 7 8 �ĵ��(������3508����2006)
	tx_message_2.Data[0] = (uint8_t)(M3508_UP.REAL_INFO.TARGET_CURRENT >> 8);
	tx_message_2.Data[1] = (uint8_t) M3508_UP.REAL_INFO.TARGET_CURRENT;
  tx_message_2.Data[2] = (uint8_t)(M3508_YAW.REAL_INFO.TARGET_CURRENT >> 8);
  tx_message_2.Data[3] = (uint8_t) M3508_YAW.REAL_INFO.TARGET_CURRENT;
  tx_message_2.Data[4] = (uint8_t)(M3508_TRANSATE.REAL_INFO.TARGET_CURRENT >> 8);
  tx_message_2.Data[5] = (uint8_t)(M3508_TRANSATE.REAL_INFO.TARGET_CURRENT);
	
	CAN_Transmit(CAN1, &tx_message_2);
}

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
int up_finished;
// �滮̧������Ӧ�е�RPM
//								*ARM_NOW_MOTION 	M3508_ARM_MOTOR_REAL_INFO.REAL_ANGLE[���������ʵ�Ƕ�]
void ad_plan_arm_motor_RPM_UP(ARM_VELOCITY_PLANNING motion, 							float pos			)	
{
	float Ssu;   //��·��
	float Sac;   //����·��
	float Sde;   //����·��
	float Sco;   //����·��
	float Aac;   //���ټ��ٶ�
	float Ade;   //���ټ��ٶ�
	float S;     //��ǰ·��
	up_finished=0;
	// �����������������ִ���ٶȹ滮		
	if((motion.Rac > 1) || (motion.Rac < 0) ||		//����·�̵ı���
		 (motion.Rde > 1) || (motion.Rde < 0) ||	//����·�̵ı���
		 (motion.Vmax < motion.Vstart) )			//�����ٶ�<��ʼ���ٶ� 
	{
		UP_MOTOR_TARGET_RPM = 0;  // ��̧���������˶�
		return;
	}
	// ����ģʽ
	if(motion.Pstart == motion.Pend)	//��ʼλ��=����λ��
	{
		UP_MOTOR_TARGET_RPM = motion.Vstart * motion.Vmax;	//��ʼ���ٶ�*�����ٶ�
		return;
	}
	
	// ����һЩ����
	Ssu = ABS(motion.Pend - motion.Pstart); 	//��·��   
	Sac = Ssu * motion.Rac;		//����·�� =	��·�� * ����·�̵ı���
	Sde = Ssu * motion.Rde;		//����·�� =	��·�� * ����·�̵ı���
	Sco = Ssu - Sac - Sde;		//����·�� = ��·�� - ����·�� - ����·��
	Aac = (motion.Vmax * motion.Vmax - motion.Vstart * motion.Vstart) / (2.0f * Sac);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
	Ade = (motion.Vend * motion.Vend -   motion.Vmax *   motion.Vmax) / (2.0f * Sde);	
	
	// �����쳣���
	if(((motion.Pend > motion.Pstart) && (pos < motion.Pstart)) ||		//[(����λ�� > ��ʼλ��) && (���������ʵ�Ƕ�pos <��ʼλ��)]	||
		 ((motion.Pend < motion.Pstart) && (pos > motion.Pstart)))		//	[(����λ�� < ��ʼλ��) && (���������ʵ�Ƕ�pos >��ʼλ��)]
	{
		UP_MOTOR_TARGET_RPM = motion.Vstart;	//TARGET_RPM = ��ʼ���ٶ�
	}
	else if(((motion.Pend > motion.Pstart) && (pos > motion.Pend)) ||
		      ((motion.Pend < motion.Pstart) && (pos < motion.Pend)))
	{
		UP_MOTOR_TARGET_RPM = motion.Vend;	//TARGET_RPM = ĩβ���ٶ�
	}
	else
	{
		S = ABS(pos - motion.Pstart);      //��ʼλ��
		
		// �滮RPM
		if     (S < Sac)       UP_MOTOR_TARGET_RPM = sqrt(2.0f * Aac * S + motion.Vstart * motion.Vstart);               // ���ٽ׶�
		else if(S < (Sac+Sco)) UP_MOTOR_TARGET_RPM = motion.Vmax;                                                        // ���ٽ׶�
		else                   UP_MOTOR_TARGET_RPM = sqrt(motion.Vend * motion.Vend - 2.0f * Ade * ABS(Ssu - S));  // ���ٽ׶�
	}
	
	// ������ʵ�������
	if(motion.Pend < motion.Pstart) UP_MOTOR_TARGET_RPM = -UP_MOTOR_TARGET_RPM;
	//pid
	PID_incremental_PID_calculation(&M3508_UP.MOTOR_PID, M3508_UP.REAL_INFO.RPM ,UP_MOTOR_TARGET_RPM);
	M3508_UP.REAL_INFO.TARGET_CURRENT = M3508_UP.MOTOR_PID.output;
	
	if(ABS(S-Ssu)<0.0001){up_finished=1;}
	//LADRC
//	LADRC_Loop(&ADRC_M3508_UP,M3508_UP.REAL_INFO.RPM ,UP_MOTOR_TARGET_RPM);
//	M3508_UP.REAL_INFO.TARGET_CURRENT=ADRC_M3508_UP.u;
}

// �滮��̨���Ӧ�е�RPM
//								*ARM_NOW_MOTION 	M3508_ARM_MOTOR_REAL_INFO.REAL_ANGLE[���������ʵ�Ƕ�]
void ad_plan_arm_motor_RPM_YAW(ARM_VELOCITY_PLANNING motion, 							float pos			)	
{
	float Ssu;   //��·��
	float Sac;   //����·��
	float Sde;   //����·��
	float Sco;   //����·��
	float Aac;   //���ټ��ٶ�
	float Ade;   //���ټ��ٶ�
	float S;     //��ǰ·��
	
	// �����������������ִ���ٶȹ滮		
	if((motion.Rac > 1) || (motion.Rac < 0) ||		//����·�̵ı���
		 (motion.Rde > 1) || (motion.Rde < 0) ||	//����·�̵ı���
		 (motion.Vmax < motion.Vstart) )			//�����ٶ�<��ʼ���ٶ� 
	{
		YAW_MOTOR_TARGET_RPM = 0;  // ���צ���˶�
		return;
	}
	// ����ģʽ
	if(motion.Pstart == motion.Pend)	//��ʼλ��=����λ��
	{
		YAW_MOTOR_TARGET_RPM = motion.Vstart * motion.Vmax;	//��ʼ���ٶ�*�����ٶ�
		return;
	}
	
	// ����һЩ����
	Ssu = ABS(motion.Pend - motion.Pstart); 	//��·��   
	Sac = Ssu * motion.Rac;		//����·�� =	��·�� * ����·�̵ı���
	Sde = Ssu * motion.Rde;		//����·�� =	��·�� * ����·�̵ı���
	Sco = Ssu - Sac - Sde;		//����·�� = ��·�� - ����·�� - ����·��
	Aac = (motion.Vmax * motion.Vmax - motion.Vstart * motion.Vstart) / (2.0f * Sac);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
	Ade = (motion.Vend * motion.Vend -   motion.Vmax *   motion.Vmax) / (2.0f * Sde);	
	
	// �����쳣���
	if(((motion.Pend > motion.Pstart) && (pos < motion.Pstart)) ||		//[(����λ�� > ��ʼλ��) && (���������ʵ�Ƕ�pos <��ʼλ��)]	||
		 ((motion.Pend < motion.Pstart) && (pos > motion.Pstart)))		//	[(����λ�� < ��ʼλ��) && (���������ʵ�Ƕ�pos >��ʼλ��)]
	{
		YAW_MOTOR_TARGET_RPM = motion.Vstart;	//TARGET_RPM = ��ʼ���ٶ�
	}
	else if(((motion.Pend > motion.Pstart) && (pos > motion.Pend)) ||
		      ((motion.Pend < motion.Pstart) && (pos < motion.Pend)))
	{
		YAW_MOTOR_TARGET_RPM = motion.Vend;	//TARGET_RPM = ĩβ���ٶ�
	}
	else
	{
		S = ABS(pos - motion.Pstart);      //��ʼλ��
		
		// �滮RPM
		if     (S < Sac)       YAW_MOTOR_TARGET_RPM = sqrt(2.0f * Aac * S + motion.Vstart * motion.Vstart);               // ���ٽ׶�
		else if(S < (Sac+Sco)) YAW_MOTOR_TARGET_RPM = motion.Vmax;                                                        // ���ٽ׶�
		else                   YAW_MOTOR_TARGET_RPM = sqrt(motion.Vend * motion.Vend - 2.0f * Ade * ABS(Ssu - S));  // ���ٽ׶�
	}
	
	// ������ʵ�������
	if(motion.Pend < motion.Pstart) YAW_MOTOR_TARGET_RPM = -YAW_MOTOR_TARGET_RPM;
	//pid
//	PID_incremental_PID_calculation(&M3508_YAW.MOTOR_PID, M3508_YAW.REAL_INFO.RPM ,YAW_MOTOR_TARGET_RPM);
//	M3508_YAW.REAL_INFO.TARGET_CURRENT = M3508_YAW.MOTOR_PID.output;
	//LADRC
//	LADRC_Loop(&ADRC_M3508_YAW,M3508_YAW.REAL_INFO.RPM ,YAW_MOTOR_TARGET_RPM);
//	M3508_YAW.REAL_INFO.TARGET_CURRENT=ADRC_M3508_YAW.u;
}

// �滮���ݵ��Ӧ�е�RPM
//								*ARM_NOW_MOTION 	M3508_ARM_MOTOR_REAL_INFO.REAL_ANGLE[���������ʵ�Ƕ�]
int transate_finished;
void ad_plan_arm_motor_RPM_TRANSATE1(TRANSATE_VELOCITY_PLANNING motion, 							float pos			)	
{
	float Ssu;   //��·��
	float Sac;   //����·��
	float Sde;   //����·��
	float Sco;   //����·��
	float Aac;   //���ټ��ٶ�
	float Ade;   //���ټ��ٶ�
	float S;     //��ǰ·��
	
	transate_finished=0;
	// �����������������ִ���ٶȹ滮		
	if((motion.Rac > 1) || (motion.Rac < 0) ||		//����·�̵ı���
		 (motion.Rde > 1) || (motion.Rde < 0) ||	//����·�̵ı���
		 (motion.Vmax < motion.Vstart) )			//�����ٶ�<��ʼ���ٶ� 
	{
		TRANSATE_MOTOR_TARGET_RPM = 0;  // ��̧���������˶�
		return ;
	}
	// ����ģʽ
	if(motion.Pstart == motion.Pend)	//��ʼλ��=����λ��
	{
		TRANSATE_MOTOR_TARGET_RPM = motion.Vstart * motion.Vmax;	//��ʼ���ٶ�*�����ٶ�
		return ;
	}
	
	// ����һЩ����
	Ssu = ABS(motion.Pend - motion.Pstart); 	//��·��   
	Sac = Ssu * motion.Rac;		//����·�� =	��·�� * ����·�̵ı���
	Sde = Ssu * motion.Rde;		//����·�� =	��·�� * ����·�̵ı���
	Sco = Ssu - Sac - Sde;		//����·�� = ��·�� - ����·�� - ����·��
	Aac = (motion.Vmax * motion.Vmax - motion.Vstart * motion.Vstart) / (2.0f * Sac);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
	Ade = (motion.Vend * motion.Vend -   motion.Vmax *   motion.Vmax) / (2.0f * Sde);	
	
	// �����쳣���
	if(((motion.Pend > motion.Pstart) && (pos < motion.Pstart)) ||		//[(����λ�� > ��ʼλ��) && (���������ʵ�Ƕ�pos <��ʼλ��)]	||
		 ((motion.Pend < motion.Pstart) && (pos > motion.Pstart)))		//	[(����λ�� < ��ʼλ��) && (���������ʵ�Ƕ�pos >��ʼλ��)]
	{
		TRANSATE_MOTOR_TARGET_RPM = motion.Vstart;	//TARGET_RPM = ��ʼ���ٶ�
	}
	else if(((motion.Pend > motion.Pstart) && (pos > motion.Pend)) ||
		      ((motion.Pend < motion.Pstart) && (pos < motion.Pend)))
	{
		TRANSATE_MOTOR_TARGET_RPM = motion.Vend;	//TARGET_RPM = ĩβ���ٶ�
	}
	else
	{
		S = ABS(pos - motion.Pstart);      //��ʼλ��
		
		// �滮RPM
		if     (S < Sac)       TRANSATE_MOTOR_TARGET_RPM = sqrt(2.0f * Aac * S + motion.Vstart * motion.Vstart);               // ���ٽ׶�
		else if(S < (Sac+Sco)) TRANSATE_MOTOR_TARGET_RPM = motion.Vmax;                                                        // ���ٽ׶�
		else                   TRANSATE_MOTOR_TARGET_RPM = sqrt(motion.Vend * motion.Vend - 2.0f * Ade * ABS(Ssu - S));  // ���ٽ׶�
	}
	
	// ������ʵ�������
	if(motion.Pend < motion.Pstart) TRANSATE_MOTOR_TARGET_RPM = -TRANSATE_MOTOR_TARGET_RPM;
	//pid
	PID_incremental_PID_calculation(&M3508_TRANSATE.MOTOR_PID, M3508_TRANSATE.REAL_INFO.RPM ,TRANSATE_MOTOR_TARGET_RPM);
	M3508_TRANSATE.REAL_INFO.TARGET_CURRENT = M3508_TRANSATE.MOTOR_PID.output;
	
	if(ABS(S-Ssu)<0.0001){transate_finished=1;}
	
	//���������ת(����)
	else if(ABS(S-Ssu)>5&&TRANSATE_MOTOR_TARGET_RPM<5){PUSH(Ssu,motion.Pend,motion.Vmax,motion.Vstart,motion.Vend,motion.Rac,motion.Rde);}//void PUSH(float start,float end,float speedmax,float speedstart,float speedend,float ac,float de)
//	//s�����߹滮�Ĳ�������
//	curve.speedMax=8000;
//	curve.aTimes=1000;
//	curve.p_add=0.1;
//	curve.speedMin=100;
//	curve.stepSpeed=40;
//	MotorVelocityCurve(&curve,&M3508_TRANSATE.MOTOR_PID);
//	M3508_TRANSATE.REAL_INFO.TARGET_CURRENT = M3508_TRANSATE.MOTOR_PID.output;
	//LADRC
//	LADRC_Loop(&ADRC_M3508_TRANSATE,M3508_TRANSATE.REAL_INFO.RPM ,TRANSATE_MOTOR_TARGET_RPM);
//	M3508_TRANSATE.REAL_INFO.TARGET_CURRENT=ADRC_M3508_TRANSATE.u;
}
void set_speed(float speed)
{
	TRANSATE_MOTOR_TARGET_RPM = speed;
	PID_incremental_PID_calculation(&M3508_TRANSATE.MOTOR_PID, M3508_TRANSATE.REAL_INFO.RPM ,TRANSATE_MOTOR_TARGET_RPM);
	M3508_TRANSATE.REAL_INFO.TARGET_CURRENT = M3508_TRANSATE.MOTOR_PID.output;
	chassis_m3508_m2006_send_motor_currents_can1();
}
	

