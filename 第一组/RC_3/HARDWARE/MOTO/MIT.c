#include "includeh.h"
AK80_REAL_INFO MIT_DRIVER_REAL_INFO[3];

void AK80_Init(void)
{
  CAN2_Init();
	ak80_control_cmd(AK80_ID1, 0x01);
	ak80_control_cmd(AK80_ID2, 0x01);
	ak80_control_cmd(AK80_ID3, 0x01);
}

float fmaxf(float a,float b)//a，b取最大
{
	return a>=b?a:b;
}

float fminf(float a,float b)//a，b取最小
{
	return a<=b?a:b;
}

/***浮点型转整形***
入口参数：浮点数据、该数据最小值、该数据最大值、位数
*****************/
int float_to_uint(float x1,float x1_min,float x1_max,int bits)
{
	float span = x1_max-x1_min;
	float offset = x1_min;
	return (int)((x1-offset)*((float)((1<<bits)-1))/span);
}

//整型转浮点型
//根据给定的范围和位数，将无符号整数转换为浮点
float uint_to_float(int x1_int,float x1_min,float x1_max,int bits)
{
	float span=x1_max-x1_min;
	float offset=x1_min;
	return ((float)x1_int)*span/((float)((1<<bits)-1)) + offset;
}

/*
速度模式发送函数
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
p_des 目标位置
v_des 目标速度
kp    位置环参数
kd    速度环参数
t_ff  目标扭矩
*/

/*
由使用说明书，其本身有特殊can代码，分别控制
1.进入电机控制模式
2.退出电机控制模式
3.设置电机当前位置为零点
即想要控制电机模式，应该先在此调用相关函数
*/
void ak80_control_cmd(uint8_t ID,uint8_t cmd)
{
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	
	// 配置仲裁段和数据段	
	tx_message.StdId = 0x200+ID;  // 用于ID为 1 的电机
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
在CAN2中断中不断更新电机的数据
*/
void AK80_update_info(CanRxMsg *msg)//不断更新数据
{
	int p_int;
	int v_int;
	int i_int;
	switch(msg->Data[0])//检测ID
	{
		case AK80_ID1:
		{
			p_int=(msg->Data[1]<<8)|msg->Data[2];//电机位置数据
			v_int = (msg->Data[3]<<4)|(msg->Data[4]>>4);//电机速度数据
			i_int = ((msg->Data[4]&0xF)<<8)|msg->Data[5];//电机扭矩数据
			
			MIT_DRIVER_REAL_INFO[0].ANGLE   = uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16);// 转子机械角度
			MIT_DRIVER_REAL_INFO[0].V_angle = uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12);// 实际转子转速
			MIT_DRIVER_REAL_INFO[0].CURRENT = uint_to_float(i_int, -I_MAX, I_MAX, 12);		 // 实际转矩电流
		}break;
		
		case AK80_ID2:
		{ 
			p_int = (msg->Data[1]<<8)|msg->Data[2]; 			//电机位置数据
			v_int = (msg->Data[3]<<4)|(msg->Data[4]>>4); 	//电机速度数据
			i_int = ((msg->Data[4]&0xF)<<8)|msg->Data[5]; //电机扭矩数据
				
			MIT_DRIVER_REAL_INFO[1].ANGLE   = uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16);// 转子机械角度
			MIT_DRIVER_REAL_INFO[1].V_angle = uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12);// 实际转子转速
			MIT_DRIVER_REAL_INFO[1].CURRENT = uint_to_float(i_int, -I_MAX, I_MAX, 12);		 // 实际转矩电流
		}; break;
		
		case AK80_ID3:
		{ 
			p_int = (msg->Data[1]<<8)|msg->Data[2]; 			//电机位置数据
			v_int = (msg->Data[3]<<4)|(msg->Data[4]>>4); 	//电机速度数据
			i_int = ((msg->Data[4]&0xF)<<8)|msg->Data[5]; //电机扭矩数据
				
			MIT_DRIVER_REAL_INFO[2].ANGLE   = uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16);// 转子机械角度
			MIT_DRIVER_REAL_INFO[2].V_angle = uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12);// 实际转子转速
			MIT_DRIVER_REAL_INFO[2].CURRENT = uint_to_float(i_int, -I_MAX, I_MAX, 12);		 // 实际转矩电流
		}; break;
		
		default:
		   break;
	}
}
