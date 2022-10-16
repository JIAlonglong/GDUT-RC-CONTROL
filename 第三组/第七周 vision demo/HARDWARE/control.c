#include "includes.h"

float start_pos_x=0;
float end_pos_x=90;
float vx_in_t=0;
float vx=0;
float real_vx=0;
float real_angle_x=0;

float start_pos_y=0;
float end_pos_y=90;
float vy_in_t=0;
float vy=0;
float real_vy=0;
float real_angle_y=0;

float start_pos_w=0;
float end_pos_w=90;
float vw_in_t=0;
int vw=0;
float real_vw=0;
float real_angle_w=0;

int relative_vx=0;
int relative_vy=0;
int manual_move_mode=0;//(mode=0:�������ꣻmode=1:�ֲ�����)

float VX,VY;
int VW;

ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
CHASSIS_MOTOR_RPM CHASSIS_MOTOR_TARGET_RPM;
CurveObjectType curve;


// 3����������ϵ���˶�ѧ
// thetaΪ����������ϵx������������ϵx��н� ��λ����
// W����ֵ-��ʱ�� ��ֵ-˳ʱ�� 
//�������������˶�ѧ��⡪����Ϊǰ����
void World_3wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta)
{
	theta = theta* PI / 180.0f;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM = ( -cos(theta) * Vx_RPM -sin(theta) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM = (+sin(theta+PI/6.0f) * Vx_RPM - cos(theta+PI/6.0f) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM = (+cos(theta+PI/3.0f) * Vx_RPM + sin(theta+PI/3.0f) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;

}

float CURRENT_PID_M3508(float now_current,float target_current)
{ 
	 float CURRENT_KP=0.1,CURRENT_KI=0,CURRENT_KD=0;
	 static float Bias,Pwm,Integral_bias,Last_Bias;	
	 Bias=now_current-target_current;                                  //����ƫ��
	 Integral_bias+=Bias;	                                             //���ƫ��Ļ���
	 Pwm=CURRENT_KP*Bias+CURRENT_KI*Integral_bias+CURRENT_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
//	if(Pwm<=0) Pwm=0;
	 return Pwm;                                           //�������
}

//���߹滮
void MotorVelocityCurve(CurveObjectType *curve,PID *M3508)
{
	
 int flag=0;
 int time_now =0;
 int  time_acc = curve->p_add * curve->aTimes;		//����ʱ��
int time_con = curve->aTimes-time_acc;//����ʱ�䡣
    //���Ʋ��ܳ��١�
	if(curve->currentSpeed>curve->speedMax)
  {
    curve->currentSpeed=curve->speedMax;
  }
	//���Ʋ��ܳ���
//	 if(curve->targetSpeed<curve->speedMin)
//  {
//    curve->targetSpeed=curve->speedMin;
//  }
	if((M3508->output>curve->speedMin)&&flag!=1)
	//���ٽ׶�
	{
		while(time_now < time_acc)
	{
		time_now++;
		curve->currentSpeed+=curve->stepSpeed;
		vTaskDelay(1);
	}
	}
	while(time_now > time_acc)
	{
		flag=1;
		
		if(M3508->output>curve->speedMax)
		{
		time_now++;
		curve->currentSpeed=curve->speedMax;}
		if(M3508->output<curve->speedMax)
			curve->currentSpeed=M3508->output;
		vTaskDelay(1);
	}
if(M3508->output<curve->speedMin)
	flag=0;
if(time_now>curve->aTimes)
{
time_now=0;
}
}


void move()
{
	/***********�����ٶ�*******************/
		// �ٶȷֽ�
		World_3wheels
									(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM,
									ROBOT_TARGET_VELOCITY_DATA.Vy_RPM, 
									ROBOT_TARGET_VELOCITY_DATA.W_RPM,
									ROBOT_REAL_POS_DATA.POS_YAW);
		
		//����PID����
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[0], M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM);
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[1], M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM);
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[2], M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM);
	
//	  MotorVelocityCurve_task();
	
		/***********���õ���*******************/
		// ����M3508����
		M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[0].output;
		M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[1].output;  
		M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[2].output;
		
		/***********��������*******************/
		chassis_m3508_send_motor_currents_can1();


}	
	

