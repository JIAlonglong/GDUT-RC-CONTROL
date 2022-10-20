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

//ALIENTEK ̽����STM32F407������ ʵ��27
//CANͨ��ʵ��-�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com  
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK
//���Է��ͱ���
short testSend1   =5000;
short testSend2   =2000;
short testSend3   =1000;
unsigned char testSend4 = 0x05;



int main(void)
 { 
	
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	// ����ϵͳ�ж����ȼ�����4
		delay_init(168);	
   CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	 M3508_Motor_Init();
		delay_ms(500);
	 PPM_Init();//ʹ��PE2
	TIM2_GET_TIM_Init();//���벶��
bsp_UART4_Init(115200);
 uart_init(115200);
	MoveInit();
	
	
	delay_ms(500);
//while(1)
//	{
////	//����Ҫ���͵�ROS�����ݣ��Ӹú���������ǰ�������ݷ�Χ��-32768 - +32767�������ĸ����ݵķ�Χ(0 - 255)
////		usartSendData(testSend1,testSend2,testSend3,testSend4);
//		//�������ʱ
//		delay_ms(13);
//	}
//}
//	
	
	taskENTER_CRITICAL();           //�����ٽ���
	
	 //ChassisDrive����
	xTaskCreate((TaskFunction_t )ChassisDrive_task,     
							(const char*    )"ChassisDrive_task",   
							(uint16_t       )CHASSIS_STK_SIZE,
							(void*          )NULL,
							(UBaseType_t    )CHASSIS_TASK_PRIO,
							(TaskHandle_t*  )&ChassisDrive_Handler);
							
							
//	//Cast����
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
							taskEXIT_CRITICAL();            //�˳��ٽ���						              
	vTaskStartScheduler();          //�����������
}
