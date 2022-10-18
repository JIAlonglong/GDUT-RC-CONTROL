#include "includes.h"

//ѡ��ģʽ
int mode_flag=0;//0:��ͨģʽ 1:���Զ�ģʽ 2:ROSģʽ
//�����б�
TaskHandle_t StartTask_Handler;
TaskHandle_t Robot_state_task_Handler;
TaskHandle_t Auto_Task_Handler;
TaskHandle_t move_task_Handler;
TaskHandle_t Motor_Control_Handler;
int action_start_flag=0;
int main(void)
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4);
  delay_init(168);
	MoveInit();	
	FSM_Init();                                     //������״̬����ʼ��
	CAN1_Init();																		// CAN1
	AK80_Init();
	M3508_Motor_Init();															// M3508���
	TMOTOR_init();                                  //��ʼ�����
	delay_ms(500);
	bsp_Air_Init();                                 //��ʼ����ģң��
	uart_init(115200);                              //��ʼ������λ��ͨѶ
	usart2_init(115200);                                  //�뼤��ģ��ͨѶ
	TIM2_GET_TIM_Init();
	Location_Init();
	delay_ms(500);
	/*************************************************************************
		��ʼ�������˵�״̬
	**************************************************************************/
	//��������������
	xTaskCreate(  (TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )128,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )1,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������        
	vTaskStartScheduler();          //�����������

}

//����ʼ�ĺ��������е�Task���������
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();           //�����ٽ���
		//1.���̿���          10
		xTaskCreate((TaskFunction_t )move_task,       
							(const char*    )"move_task",   
							(uint16_t       )128,
							(void*          )NULL,
							(UBaseType_t    )2,
							(TaskHandle_t*  )&move_task_Handler);	
	
	//2.������״̬����           5
	xTaskCreate((TaskFunction_t )Robot_state_task, 							
							(const char*    )"Robot_state_task",   
							(uint16_t       )128,
							(void*          )NULL,
							(UBaseType_t    )3,
							(TaskHandle_t*  )&Robot_state_task_Handler);	
	//3.�������Զ�·��
	xTaskCreate((TaskFunction_t )Auto_Task, 							
							(const char*    )"Auto_Task",   
							(uint16_t       )128,
							(void*          )NULL,
							(UBaseType_t    )4,
							(TaskHandle_t*  )&Auto_Task_Handler);	
	//4.motor_control
	xTaskCreate((TaskFunction_t )Motor_Control, 							
							(const char*    )"Motor_Control",   
							(uint16_t       )128,
							(void*          )NULL,
							(UBaseType_t    )5,
							(TaskHandle_t*  )&Motor_Control_Handler);							
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
  taskEXIT_CRITICAL();            //�˳��ٽ���
}


