#include "includeh.h"
AK80_REAL_INFO MIT_DRIVER_REAL_INFO[3];

void AK80_Init(void)
{
  CAN2_Init();
	ak80_control_cmd(AK80_ID1, 0x01);
	ak80_control_cmd(AK80_ID2, 0x01);
	ak80_control_cmd(AK80_ID3, 0x01);
}

float fmaxf(float a,float b)//a��bȡ���
{
	return a>=b?a:b;
}

float fminf(float a,float b)//a��bȡ��С
{
	return a<=b?a:b;
}

/***������ת����***
��ڲ������������ݡ���������Сֵ�����������ֵ��λ��
*****************/
int float_to_uint(float x1,float x1_min,float x1_max,int bits)
{
	float span = x1_max-x1_min;
	float offset = x1_min;
	return (int)((x1-offset)*((float)((1<<bits)-1))/span);
}

//����ת������
//���ݸ����ķ�Χ��λ�������޷�������ת��Ϊ����
float uint_to_float(int x1_int,float x1_min,float x1_max,int bits)
{
	float span=x1_max-x1_min;
	float offset=x1_min;
	return ((float)x1_int)*span/((float)((1<<bits)-1)) + offset;
}

/*
�ٶ�ģʽ���ͺ���
*/
void AK80_Speed_Control(u8 ID,float vel)
{
	uint8_t *vbuf;
	vbuf=(uint8_t*)&vel;
	
	
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x200+ID;
	
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x04;
	
	tx_message.Data[0] = *vbuf; 
	tx_message.Data[1] = *(vbuf+1); 
	tx_message.Data[2] = *(vbuf+2); 
	tx_message.Data[3] = *(vbuf+3); 
	
	CAN_Transmit(CAN2,&tx_message);
}


/*
p_des Ŀ��λ��
v_des Ŀ���ٶ�
kp    λ�û�����
kd    �ٶȻ�����
t_ff  Ŀ��Ť��
*/

/*
��ʹ��˵���飬�䱾��������can���룬�ֱ����
1.����������ģʽ
2.�˳��������ģʽ
3.���õ����ǰλ��Ϊ���
����Ҫ���Ƶ��ģʽ��Ӧ�����ڴ˵�����غ���
*/
void ak80_control_cmd(uint8_t ID,uint8_t cmd)
{
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	
	// �����ٲöκ����ݶ�	
	tx_message.StdId = 0x200+ID;  // ����IDΪ 1 �ĵ��
	/// pack ints into the can buffer ///
	tx_message.Data[0] = (uint8_t)(0xFF);
	tx_message.Data[1] = (uint8_t)(0xFF);
	tx_message.Data[2] = (uint8_t)(0xFF);
	tx_message.Data[3] = (uint8_t)(0xFF);
	tx_message.Data[4] = (uint8_t)(0xFF);
	tx_message.Data[5] = (uint8_t)(0xFF);
	tx_message.Data[6] = (uint8_t)(0xFF);
	
	switch(cmd)
	{
		case CMD_MOTOR_MODE:
			tx_message.Data[7] = (uint8_t)(0xFC);break;
		case CMD_RESET_MODE:
			tx_message.Data[7] = (uint8_t)(0xFD);break;
		case CMD_ZERO_POSITION:
            tx_message.Data[7] = (uint8_t)(0xFE);break;
		default:
			return;
	}
	CAN_Transmit(CAN2, &tx_message);
}
//

/*
��CAN2�ж��в��ϸ��µ��������
*/
void AK80_update_info(CanRxMsg *msg)//���ϸ�������
{
	int p_int;
	int v_int;
	int i_int;
	switch(msg->Data[0])//���ID
	{
		case AK80_ID1:
		{
			p_int=(msg->Data[1]<<8)|msg->Data[2];//���λ������
			v_int = (msg->Data[3]<<4)|(msg->Data[4]>>4);//����ٶ�����
			i_int = ((msg->Data[4]&0xF)<<8)|msg->Data[5];//���Ť������
			
			MIT_DRIVER_REAL_INFO[0].ANGLE   = uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16);// ת�ӻ�е�Ƕ�
			MIT_DRIVER_REAL_INFO[0].V_angle = uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12);// ʵ��ת��ת��
			MIT_DRIVER_REAL_INFO[0].CURRENT = uint_to_float(i_int, -I_MAX, I_MAX, 12);		 // ʵ��ת�ص���
		}break;
		
		case AK80_ID2:
		{ 
			p_int = (msg->Data[1]<<8)|msg->Data[2]; 			//���λ������
			v_int = (msg->Data[3]<<4)|(msg->Data[4]>>4); 	//����ٶ�����
			i_int = ((msg->Data[4]&0xF)<<8)|msg->Data[5]; //���Ť������
				
			MIT_DRIVER_REAL_INFO[1].ANGLE   = uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16);// ת�ӻ�е�Ƕ�
			MIT_DRIVER_REAL_INFO[1].V_angle = uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12);// ʵ��ת��ת��
			MIT_DRIVER_REAL_INFO[1].CURRENT = uint_to_float(i_int, -I_MAX, I_MAX, 12);		 // ʵ��ת�ص���
		}; break;
		
		case AK80_ID3:
		{ 
			p_int = (msg->Data[1]<<8)|msg->Data[2]; 			//���λ������
			v_int = (msg->Data[3]<<4)|(msg->Data[4]>>4); 	//����ٶ�����
			i_int = ((msg->Data[4]&0xF)<<8)|msg->Data[5]; //���Ť������
				
			MIT_DRIVER_REAL_INFO[2].ANGLE   = uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16);// ת�ӻ�е�Ƕ�
			MIT_DRIVER_REAL_INFO[2].V_angle = uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12);// ʵ��ת��ת��
			MIT_DRIVER_REAL_INFO[2].CURRENT = uint_to_float(i_int, -I_MAX, I_MAX, 12);		 // ʵ��ת�ص���
		}; break;
		
		default:
		   break;
	}
}
