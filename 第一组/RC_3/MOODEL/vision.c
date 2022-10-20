#include "includeh.h"


/*-------------------------------������----------------------------------------------*/


extern PID laser_X_pid;
extern PID laser_Y_pid;


/*************************************************************************************/


/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 
++++0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/

/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/


/**************************************************************************
ͨ�ŵķ��ͺ����ͽ��պ��������һЩ���������������������
**************************************************************************/

int location_x=0;
int location_y=0;
int ladar_distance=0;
int ladar_yaw=0;
unsigned char crtlFlag=0x00;
//���ݽ����ݴ���
unsigned char  receiveBuff[16] = {0};         
//ͨ��Э�鳣��
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//�������ݣ������١������١��Ƕȣ������壨-32767 - +32768��
union sendData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;

//�������ٿ����ٶȹ�����
union receiveData
{
	short d;
	unsigned char data[2];
}distance,yaw_lidar;

union receiveData1
{
	int d;
	unsigned char data[4];
}x_position,y_position;

/**************************************************************************
�������ܣ�ͨ�������жϷ���������ȡ��λ�����͵������ֿ����ٶȡ�Ԥ�����Ʊ�־λ���ֱ���������
��ڲ������������ٿ��Ƶ�ַ���������ٿ��Ƶ�ַ��Ԥ�����Ʊ�־λ
����  ֵ������������
**************************************************************************/
int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver              = 0;          //��������
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART1);   //@@@@@#####�����ʹ�ò���USART1���ĳ���Ӧ�ģ�����USART3
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://�����������ٶ����ݵĳ���
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[7]					
				j++;
				if(j >= dataLength)                         
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://����У��ֵ��Ϣ
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // �����ϢУ��ֵ
				if (checkSum != receiveBuff[3 + dataLength]) //buf[11]
				{
					printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					//����0d     buf[9]  �����ж�
					k++;
				}
				else if (k==1)
				{
					//����0a     buf[10] �����ж�

					//�����ٶȸ�ֵ����					
					 for(k = 0; k < 2; k++)
					{
					 distance.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]  buf[5]  buf[6]
						yaw_lidar.data[k] = receiveBuff[k + 5]; //buf[7]  buf[8]  buf[9]  buf[10]
					}				
					
					//�ٶȸ�ֵ����
					*p_leftSpeedSet  = (int)distance.d;
					*p_rightSpeedSet = (int)yaw_lidar.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[7];                //buf[7]
					
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**************************************************************************
�������ܣ����������ٺͽǶ����ݡ������źŽ��д����ͨ�����ڷ��͸�Linux
��ڲ�����ʵʱ�������١�ʵʱ�������١�ʵʱ�Ƕȡ������źţ����û�нǶ�Ҳ���Բ�����
����  ֵ����
**************************************************************************/
void usartSendData(short leftVel, short rightVel,short angle,unsigned char ctrlFlag)
{
	// Э�����ݻ�������
	unsigned char buf[13] = {0};
	int i, length = 0;

	// ���������������ٶ�
	leftVelNow.d  = leftVel;
	rightVelNow.d = rightVel;
	angleNow.d    = angle;
	
	// ������Ϣͷ
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1] 
	
	// ���û������������ٶȡ��Ƕ�
	length = 7;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		buf[i + 3] = leftVelNow.data[i];         // buf[3] buf[4]
		buf[i + 5] = rightVelNow.data[i];        // buf[5] buf[6]
		buf[i + 7] = angleNow.data[i];           // buf[7] buf[8]
	}
	// Ԥ������ָ��
	buf[3 + length - 1] = ctrlFlag;              // buf[9]
	
	// ����У��ֵ����Ϣβ
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[10]
	buf[3 + length + 1] = ender[0];              // buf[11]
	buf[3 + length + 2] = ender[1];              // buf[12]
	
	//�����ַ�������
	USART_Send_String(buf,sizeof(buf));
}
/**************************************************************************
�������ܣ�����ָ����С���ַ����飬��usartSendData��������
��ڲ����������ַ�������С
����  ֵ����
**************************************************************************/
void USART_Send_String(u8 *p,u16 sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		//@@@@@#####�����ʹ�ò���USART1���ĳ���Ӧ�ģ�����USART3�������������޸�
		while( !(USART1->SR&(0x01<<7)) );//���ͻ�����Ϊ��
		USART1->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}
/**************************************************************************
�������ܣ������λѭ������У�飬��usartSendData��usartReceiveOneData��������
��ڲ����������ַ�������С
����  ֵ����
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/

//�����޸ģ��޸����ڼ�����
int VisionReceiveData(int *theta_angle,int *theta_pitch)
{
	unsigned char USART_Receiver              = 0;          //��������
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART1);   //�����ʹ�ò���USART1���ĳ���Ӧ�ģ�����USART3
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://�������ݵĳ���
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] buf[4]/buf[5] buf[6]					
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
			case 2://����У��ֵ��Ϣ(�趨Ϊ0x07)
				receiveBuff[2 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
//				checkSum = 0x07;
				  // �����ϢУ��ֵ
//				if (checkSum != receiveBuff[3 + dataLength]) //buf[11]
//				{
////					printf("Received data check sum error!");
//					return 0;
//				}
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					//����0d     buf[11]  �����ж�
					k++;
				}
				else if (k==1)
				{
					//����0a     buf[12] �����ж�

					//�����ٶȸ�ֵ����					
					 for(k = 0; k < 4; k++)
					{
						x_position.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						y_position.data[k] = receiveBuff[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
					}				
//					if(x_position.d==12)
//					{
//						printf("OK!");
//					}
//					else
//					{
//						printf("error!");
//					}
					//��ֵ����
					*theta_angle = x_position.d;//ƫ����
					*theta_pitch = y_position.d;//������
//					*theta =(int)angle.d;
//					
//					//ctrlFlag
//					*flag = receiveBuff[9];                //buf[9]
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}



//�����жϷ����������ڽ����״�����
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		 //��ADC���������ݣ��������Ǵ�ŵ�����������������
		 usartReceiveOneData(&ladar_distance,&ladar_yaw,&crtlFlag);
//		 VisionReceiveData(&location_x,&location_y);
	
	 }
}

//�����жϷ����������ڽ��ռ�������
//void USART2_IRQHandler()
//{
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
// 	 {
//		 USART_ClearITPendingBit(USART2,USART_IT_RXNE);
//		 //��ROS�������ݣ��������Ǵ�ŵ�����������������
////	usartReceiveOneData(&ladar_distance,&ladar_yaw,&crtlFlag);
//	 }
//}






