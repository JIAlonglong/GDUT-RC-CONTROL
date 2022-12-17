#include "includes.h"
int location_x=0;
int location_y=0;
int ladar_distance=0;
int ladar_yaw=0;

union receiveData1
{
	int d;
	unsigned char data[4];
}x_position,y_position;

int UPDATE_x;
int UPDATE_y;
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

	USART_Receiver = USART_ReceiveData(USART3);   //�����ʹ�ò���USART1���ĳ���Ӧ�ģ�����USART3
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
					UPDATE_x=x_position.d;
					UPDATE_y=y_position.d;
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
