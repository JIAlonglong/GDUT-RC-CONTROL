#include "main.h"

u16 R0,R1,R2,R3,R4,R5,R6,R7;

void oled_show()
{
  R0 = (int) (adc_update_ranging_distance(0));//���Ͻ�  ���
  R4 =(int)(adc_update_ranging_distance(4));//���½�  ����
//     R3 = (int)(adc_update_ranging_distance(3));//���Ͻ�
   R3 = (int)(adc_update_ranging_distance(3));//���Ͻ�               ����
R5=(int)(adc_update_ranging_distance(2));
	VisionSendData(R4,R0);
// {
//		USART_SendData(USART1, (int)(2*(R0/256)+1));
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET){
//		};//�ȴ����ͽ���
//	  delay_ms(5);
//	  USART_SendData(USART1, R0);         //�򴮿�3��������
//	  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET){
//		};//�ȴ����ͽ���
//	}
//	
//		
//	
//	
//	
//	{
//		USART_SendData(USART1, (int)(2*(R5/256+1)));
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET){
//		};//�ȴ����ͽ���
//		delay_ms(5);    
//	 USART_SendData(USART1, R5);         //�򴮿�3��������
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
//	}
//	R6 = adc_update_ranging_distance(6)*10;
//	R7 = adc_update_ranging_distance(7)*10;
//	R1 = adc_update_ranging_distance(1)*10;
//	R2 = adc_update_ranging_distance(2)*10;
				//  OLED_ShowNum(32,0,R2/10,4,16);		//��ʾ��������	    
				//  OLED_ShowNum(76,0,R2%10,3,16);		//��ʾС������
	
}



short Temp_temp;

//ʦ�ֶ���ķ���Э��
/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/

//ͨ��Э�鶨��
const unsigned char header[2]={0x55,0xaa};
const unsigned char ender[2]={0x0d,0x0a};
const unsigned char judge ={0x07};

//���ݽ����ݴ���
unsigned char receiveBuff[16]={0};

/******************************************************************
//��������
//�궨��
//union sendData
//{
//	int d;
//	unsigned char data[2];
//}dr_x,dr_y,dr_yaw;

//union receiveData
//{
//	int d;
//	unsigned char data[2];
//}x_position,y_position,angle;

//void VisionSendData(int X,int Y,int yaw)
//{
//	unsigned char buf[16]={0};
//	int i= 0;
//	
//	//�����������ٶȣ�δ֪
//	dr_x.d = X;
//	dr_y.d = Y;
//	dr_yaw.d = yaw;
//	
//	for(i=0;i<2;i++)
//	{
//		buf[i]=header[i];//Э������ͷ
//	}
//	for(i=0;i<4;i++)
//	{
//		buf[i+2]=dr_x.data[i];
//		buf[i+6]=dr_y.data[i];
//		buf[i+10]=dr_yaw.data[i];
//	}

//	buf[14]=ender[0];
//	buf[15]=ender[1];
//	
//	USART_Send_String(buf,sizeof(buf));//�����ַ������ͺ�����������
//}
******************************************************************************/

//��������
union sendData
{
	int d;
	unsigned char data[4];
}dr_x,dr_y,dr_yaw;

union receiveData
{
	int d;
	unsigned char data[4];
}x_position,y_position;

/*
//��������
*/
void VisionSendData(int X,int Y)
{
	unsigned char buf[14]={0};
	int i,length = 0;
	
	//�����������ٶȣ�δ֪
	dr_x.d = X;
	dr_y.d = Y;

	
	for(i=0;i<2;i++)
	{
		buf[i]=header[i];//Э������ͷ
	}
	length = 9;
	buf[2] =length;//sizeof
	for(i=0;i<4;i++)
	{
		buf[i+3]=dr_x.data[i];
		buf[i+7]=dr_y.data[i];
		
	}
	buf[3+length-1]=getCrc8(buf,3+length);
	buf[3+length]=ender[0];
	buf[3+length+1]=ender[1];
	
	USART_Send_String(buf,sizeof(buf));//�����ַ������ͺ�����������
}
/*
�ַ������ͺ���
����ָ����С���ַ�����
��ڲ����������ַ�������С
*/
void USART_Send_String(u8 *p,u16 sendSize)
{
	static int length=0;//��̬������ֹ���ݶ�ʧ
	while(length<sendSize)
	{
		while(!(USART1->SR&(0x01<<7)));//���ͻ�����Ϊ��(�������ݻ���λӦ��Ϊ�ڰ�λ)
		USART1->DR=*p;
		p++;
		length++;
	}
	length=0;
}

/*
//��������
*/


/*
�����λѭ������У��
��ڲ����������ַ�������С
����ֵ��CRCУ����
*/
unsigned char getCrc8(unsigned char *ptr,unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc=0;
	while(len--)
	{
		crc^=*ptr++;
		for(i=0;i<8;i++)
		{
			if(crc&0x01)
				crc=(crc>>1)^0x8C;
			else
				crc>>=1;
		}
	}
	return crc;
}
//�����޸ģ��޸�Ϊֻ����ƫ�����븩����
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
			case 0://�������ݵĳ���
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				Temp_temp      = dataLength;
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
				checkSum = getCrc8(receiveBuff, 2 + dataLength);
//				checkSum = 0x07;
				  // �����ϢУ��ֵ
				if (checkSum != receiveBuff[2 + dataLength]) //buf[11]
				{
//					printf("Received data check sum error!");
					return 0;
				}
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




 