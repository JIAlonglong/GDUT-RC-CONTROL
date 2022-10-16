#include "includes.h"


//任务列表
TaskHandle_t StartTask_Handler;
TaskHandle_t Robot_state_task_Handler;
TaskHandle_t Auto_Task_Handler;
TaskHandle_t move_task_Handler;
int action_start_flag=0;
int main(void)
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4);
  delay_init(168);
	MoveInit();	
	CAN1_Init();																		// CAN1
	CAN2_Init();																		// CAN2	
	M3508_Motor_Init();															// M3508电机
	TMOTOR_init();                                  //初始化电调
	delay_ms(500);
	bsp_Air_Init();                                 //初始化航模遥控
	uart_init(115200);                              //初始化与上位机通讯
	TIM2_GET_TIM_Init();
	Location_Init();
	delay_ms(500);
	/*************************************************************************
		初始化机器人的状态
	**************************************************************************/
	//开启创建任务函数
	xTaskCreate(  (TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )128,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )1,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄        
	vTaskStartScheduler();          //开启任务调度

}

//任务开始的函数（所有的Task任务都在这里）
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
		//1.底盘控制          10
		xTaskCreate((TaskFunction_t )move_task,       
							(const char*    )"move_task",   
							(uint16_t       )128,
							(void*          )NULL,
							(UBaseType_t    )2,
							(TaskHandle_t*  )&move_task_Handler);	
	
	//2.机器人状态处理           5
	xTaskCreate((TaskFunction_t )Robot_state_task, 							
							(const char*    )"Robot_state_task",   
							(uint16_t       )128,
							(void*          )NULL,
							(UBaseType_t    )3,
							(TaskHandle_t*  )&Robot_state_task_Handler);	
	//3.机器人自动路径
	xTaskCreate((TaskFunction_t )Auto_Task, 							
							(const char*    )"Auto_Task",   
							(uint16_t       )128,
							(void*          )NULL,
							(UBaseType_t    )4,
							(TaskHandle_t*  )&Auto_Task_Handler);	
	vTaskDelete(StartTask_Handler); //删除开始任务
  taskEXIT_CRITICAL();            //退出临界区
}


