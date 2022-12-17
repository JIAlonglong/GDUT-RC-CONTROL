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


//做了修改，修改用于激光测距
int VisionReceiveData(int *theta_angle,int *theta_pitch)
{
	unsigned char USART_Receiver              = 0;          //接收数据
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART3);   //如果你使用不是USART1更改成相应的，比如USART3
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
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
			case 0://接收数据的长度
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] buf[4]/buf[5] buf[6]					
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
			case 2://接收校验值信息(设定为0x07)
				receiveBuff[2 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
//				checkSum = 0x07;
				  // 检查信息校验值
//				if (checkSum != receiveBuff[3 + dataLength]) //buf[11]
//				{
////					printf("Received data check sum error!");
//					return 0;
//				}
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					//数据0d     buf[11]  无需判断
					k++;
				}
				else if (k==1)
				{
					//数据0a     buf[12] 无需判断

					//进行速度赋值操作					
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
					//赋值操作
					*theta_angle = x_position.d;//偏航角
					*theta_pitch = y_position.d;//俯仰角
					UPDATE_x=x_position.d;
					UPDATE_y=y_position.d;
//					*theta =(int)angle.d;
//					
//					//ctrlFlag
//					*flag = receiveBuff[9];                //buf[9]
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
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
