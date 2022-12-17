#include "includes.h"

/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/


/**************************************************************************
ͨ�ŵķ��ͺ����ͽ��պ��������һЩ���������������������
**************************************************************************/

//���ݽ����ݴ���
unsigned char  receiveBuff[32] = {0};         
//ͨ��Э�鳣��
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//�������ݣ�vx vy vz�������壨-32767 - +32768��
union sendData
{
	short d;
	unsigned char data[5];//0 1 2 3 4 5
}VXNow,VYNow,VZNow,RobotX,RobotY,RobotZ;

//�����ٶȹ�����
union receiveData
{
	short d;
	unsigned char data[5];// 0 1 2
}VXSet,VYSet,VZSet,update_X,update_Y,update_Z;


/**************************************************************************
�������ܣ�ͨ�������жϷ���������ȡ��λ�����͵Ŀ����ٶȡ�Ԥ�����Ʊ�־λ���ֱ���������
��ڲ�����VX���Ƶ�ַ��VY���Ƶ�ַ��VZ ���Ƶ�ַ,Ԥ�����Ʊ�־λ
����  ֵ������������
**************************************************************************/
int usartReceiveOneData(int *p_VXSet,int *p_VYSet,int *p_VZSet,int *p_update_X,int *p_update_Y,int *p_update_Z,unsigned char *p_crtlFlag)
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
			case 0://����vx,vy,vz�ٶ����ݵĳ���
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
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
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
						VXSet.data[k]   = receiveBuff[k + 3]; //buf[3]  buf[4]
						VYSet.data[k]   = receiveBuff[k + 5]; //buf[5]  buf[6]
						VZSet.data[k]   = receiveBuff[k + 7];//buf[7]  buf[8]
						//���긳ֵ����
						update_X.data[k]= receiveBuff[k + 9];
						update_Y.data[k]= receiveBuff[k + 11];
						update_Z.data[k]= receiveBuff[k + 13];
					}				
					
					//�ٶȸ�ֵ����
					*p_VXSet  = (int)VXSet.d;
					*p_VYSet  = (int)VYSet.d;
					*p_VZSet  = (int)VZSet.d;
					//���긳ֵ����
					*p_update_X=(int)update_X.d;
					*p_update_Y=(int)update_Y.d;
					*p_update_Z=(int)update_Z.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[15];                //buf[9]
					
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
��ڲ�����ʵʱVX���١�ʵʱVY���١�ʵʱVZ���١� ���� �����źţ����û�нǶ�Ҳ���Բ�����
����  ֵ����
**************************************************************************/
void usartSendData(short VX, short VY,short VZ,short Robot_X,short Robot_Y,short Robot_Z,unsigned char ctrlFlag)
{
	// Э�����ݻ�������
	unsigned char buf[20] = {0};
	int i, length = 0;

	// ���������ٶ�
	VXNow.d  = VX;
	VYNow.d = VY;
	VZNow.d    = VZ;
	RobotX.d=Robot_X;
	RobotY.d=Robot_Y;
	RobotZ.d=Robot_Z;
	
	// ������Ϣͷ
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1] 
	
	// ���û������������ٶȡ��Ƕ�
	length = 13;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		buf[i + 3]  = VXNow.data[i];         // buf[3] buf[4]
		buf[i + 5]  = VYNow.data[i];        // buf[5] buf[6]
		buf[i + 7]  = VZNow.data[i];           // buf[7] buf[8]
		buf[i + 9]  = RobotX.data[i];           // buf[9] buf[10]
		buf[i + 11] = RobotY.data[i];            // buf[11] buf[12]
		buf[i + 13] = RobotZ.data[i];            // buf[13] buf[14]
		
	}
	// Ԥ������ָ��
	buf[3 + length - 1] = ctrlFlag;              // buf[15]
	
	// ����У��ֵ����Ϣβ
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[16]
	buf[3 + length + 1] = ender[0];              // buf[17]
	buf[3 + length + 2] = ender[1];              // buf[18]
	
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

unsigned char rosctrl_flag=0X00; 
int ros_vx=0,ros_vy=0,ros_vz=0,new_x=0,new_y=0,new_z=0;


//====================================�����жϷ������=================================================
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//��������жϱ�־λ
		 //��ROS���յ������ݣ���ŵ������ĸ�������
		 usartReceiveOneData(&ros_vx,&ros_vy,&ros_vz,&new_x,&new_y,&new_z,&rosctrl_flag);
	 }
}
//===========================================END=======================================================

