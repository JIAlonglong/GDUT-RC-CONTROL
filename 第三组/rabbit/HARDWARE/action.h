#ifndef __ACTION_H
#define __ACTION_H


// 定位系统安装误差		(单位：mm)
#define INSTALL_ERROR_Y		0
#define INSTALL_ERROR_X		0



// 机器人的真实位置
typedef struct ROBOT_REAL_POS
{
  float POS_X;
  float POS_Y;     
  float POS_YAW;
}ROBOT_REAL_POS;

// 东大全场定位模块定位的位置
typedef struct ACTION_GL_POS
{
	float ANGLE_Z;
	float ANGLE_X;
	float ANGLE_Y;	
	float POS_X;
	float POS_Y;
	float W_Z;
	
	float LAST_POS_X;
	float LAST_POS_Y;
	
	float DELTA_POS_X;
	float DELTA_POS_Y;	
	
	float REAL_X;
	float REAL_Y;
}ACTION_GL_POS;


extern float OFFSET_YAW;
extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;
extern int action_start_flag;

void Location_Init(void);
void Update_Action_gl_position(float value[6]);
void Action_Uart_init(u32 baud_rate);
void USART_SendString(USART_TypeDef* USARTx, char *DataString);
void USART_SendFloat(USART_TypeDef* USARTx, char *DataString,float DateFloat);
extern int Jiguang_Action_Update(float i,float j);
void Update_J(float New_J);
void Update_X(float New_X);
void Update_Y(float New_Y);
#endif

