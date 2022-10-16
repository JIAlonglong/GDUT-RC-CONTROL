#include "main.h"

u16 R0,R1,R2,R3,R4,R5,R6,R7;

void oled_show()
{
  R0 = (int) (adc_update_ranging_distance(0));//左上角  左边
  R4 =(int)(adc_update_ranging_distance(4));//左下角  后面
//     R3 = (int)(adc_update_ranging_distance(3));//右上角
   R3 = (int)(adc_update_ranging_distance(3));//右上角               激光
R5=(int)(adc_update_ranging_distance(2));
	VisionSendData(R4,R0);
// {
//		USART_SendData(USART1, (int)(2*(R0/256)+1));
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET){
//		};//等待发送结束
//	  delay_ms(5);
//	  USART_SendData(USART1, R0);         //向串口3发送数据
//	  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET){
//		};//等待发送结束
//	}
//	
//		
//	
//	
//	
//	{
//		USART_SendData(USART1, (int)(2*(R5/256+1)));
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET){
//		};//等待发送结束
//		delay_ms(5);    
//	 USART_SendData(USART1, R5);         //向串口3发送数据
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//	}
//	R6 = adc_update_ranging_distance(6)*10;
//	R7 = adc_update_ranging_distance(7)*10;
//	R1 = adc_update_ranging_distance(1)*10;
//	R2 = adc_update_ranging_distance(2)*10;
				//  OLED_ShowNum(32,0,R2/10,4,16);		//显示整数部分	    
				//  OLED_ShowNum(76,0,R2%10,3,16);		//显示小数部分
	
}



short Temp_temp;

//师兄定义的发送协议
/*--------------------------------发送协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

//通信协议定义
const unsigned char header[2]={0x55,0xaa};
const unsigned char ender[2]={0x0d,0x0a};
const unsigned char judge ={0x07};

//数据接收暂存区
unsigned char receiveBuff[16]={0};

/******************************************************************
//发送数据
//标定用
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
//	//左右轮期望速度，未知
//	dr_x.d = X;
//	dr_y.d = Y;
//	dr_yaw.d = yaw;
//	
//	for(i=0;i<2;i++)
//	{
//		buf[i]=header[i];//协议数据头
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
//	USART_Send_String(buf,sizeof(buf));//利用字符串发送函数发送数据
//}
******************************************************************************/

//发送数据
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
//发送数据
*/
void VisionSendData(int X,int Y)
{
	unsigned char buf[14]={0};
	int i,length = 0;
	
	//左右轮期望速度，未知
	dr_x.d = X;
	dr_y.d = Y;

	
	for(i=0;i<2;i++)
	{
		buf[i]=header[i];//协议数据头
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
	
	USART_Send_String(buf,sizeof(buf));//利用字符串发送函数发送数据
}
/*
字符串发送函数
发送指定大小的字符数组
入口参数：数组地址、数组大小
*/
void USART_Send_String(u8 *p,u16 sendSize)
{
	static int length=0;//静态变量防止数据丢失
	while(length<sendSize)
	{
		while(!(USART1->SR&(0x01<<7)));//发送缓冲区为空(发送数据缓冲位应该为第八位)
		USART1->DR=*p;
		p++;
		length++;
	}
	length=0;
}

/*
//发送数据
*/


/*
计算八位循环冗余校验
入口参数：数组地址、数组大小
返回值：CRC校验码
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
//做了修改，修改为只接受偏航角与俯仰角
int VisionReceiveData(int *theta_angle,int *theta_pitch)
{
	unsigned char USART_Receiver              = 0;          //接收数据
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART1);   //如果你使用不是USART1更改成相应的，比如USART3
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //数据头两位 //buf[0]
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
				Temp_temp      = dataLength;
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
				checkSum = getCrc8(receiveBuff, 2 + dataLength);
//				checkSum = 0x07;
				  // 检查信息校验值
				if (checkSum != receiveBuff[2 + dataLength]) //buf[11]
				{
//					printf("Received data check sum error!");
					return 0;
				}
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




 