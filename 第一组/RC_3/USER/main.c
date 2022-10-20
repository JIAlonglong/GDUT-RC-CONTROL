#include "sys.h"
#include "delay.h"

#include "led.h"
#include "lcd.h"
#include "key.h"
#include "can.h"
#include "air.h"
#include "includeh.h"
TaskHandle_t MotorVelocityCurve_Handler;
TaskHandle_t ChassisDrive_Handler;
TaskHandle_t Move_Handler;
TaskHandle_t shooting_Handler;

//ALIENTEK 探索者STM32F407开发板 实验27
//CAN通信实验-库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK
//测试发送变量
short testSend1   =5000;
short testSend2   =2000;
short testSend3   =1000;
unsigned char testSend4 = 0x05;



int main(void)
 { 
	
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	// 设置系统中断优先级分组4
		delay_init(168);	
   CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	 M3508_Motor_Init();
		delay_ms(500);
	 PPM_Init();//使用PE2
	TIM2_GET_TIM_Init();//输入捕获
bsp_UART4_Init(115200);
 uart_init(115200);
	MoveInit();
	
	
	delay_ms(500);
//while(1)
//	{
////	//将需要发送到ROS的数据，从该函数发出，前三个数据范围（-32768 - +32767），第四个数据的范围(0 - 255)
////		usartSendData(testSend1,testSend2,testSend3,testSend4);
//		//必须的延时
//		delay_ms(13);
//	}
//}
//	
	
	taskENTER_CRITICAL();           //进入临界区
	
	 //ChassisDrive任务
	xTaskCreate((TaskFunction_t )ChassisDrive_task,     
							(const char*    )"ChassisDrive_task",   
							(uint16_t       )CHASSIS_STK_SIZE,
							(void*          )NULL,
							(UBaseType_t    )CHASSIS_TASK_PRIO,
							(TaskHandle_t*  )&ChassisDrive_Handler);
							
							
//	//Cast任务
	xTaskCreate((TaskFunction_t )Move_task,     
							(const char*    )"Move_task",   
							(uint16_t       )MOVE_STK_SIZE,
							(void*          )NULL,
							(UBaseType_t    )MOVE_TASK_PRIO,
							(TaskHandle_t*  )&Move_Handler);	

		xTaskCreate((TaskFunction_t )shooting_FSM_task,     
							(const char*    )"shooting_FSM_task",   
							(uint16_t       )SHOOTING_STK_SIZE,
							(void*          )NULL,
							(UBaseType_t    )SHOOTING_TASK_PRIO,
							(TaskHandle_t*  )&shooting_Handler);									
							taskEXIT_CRITICAL();            //退出临界区						              
	vTaskStartScheduler();          //开启任务调度
}
