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
int manual_move_mode=0;//(mode=0:世界坐标；mode=1:局部坐标)

float VX,VY;
int VW;

ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
CHASSIS_MOTOR_RPM CHASSIS_MOTOR_TARGET_RPM;
CurveObjectType curve;


// 3轮世界坐标系逆运动学
// theta为机器人坐标系x轴与世界坐标系x轴夹角 单位：度
// W：正值-逆时针 负值-顺时针 
//三轮世界坐标运动学逆解——边为前方向
void World_3wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta)
{
	theta = theta* PI / 180.0f;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM = -( -cos(theta) * Vx_RPM -sin(theta) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM = -(+sin(theta+PI/6.0f) * Vx_RPM - cos(theta+PI/6.0f) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;
	CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM = -(+cos(theta+PI/3.0f) * Vx_RPM + sin(theta+PI/3.0f) * Vy_RPM + Robot_R*W_RPM) * M3508_MS_To_RM;

}

float CURRENT_PID_M3508(float now_current,float target_current)
{ 
	 float CURRENT_KP=0.1,CURRENT_KI=0,CURRENT_KD=0;
	 static float Bias,Pwm,Integral_bias,Last_Bias;	
	 Bias=now_current-target_current;                                  //计算偏差
	 Integral_bias+=Bias;	                                             //求出偏差的积分
	 Pwm=CURRENT_KP*Bias+CURRENT_KI*Integral_bias+CURRENT_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
//	if(Pwm<=0) Pwm=0;
	 return Pwm;                                           //增量输出
}

//曲线规划
void MotorVelocityCurve(CurveObjectType *curve,PID *M3508,M3508_REAL_INFO *M3508_REAL)
{
	
 int flag=0;
 int time_now =0;
//	int deta_distance=M3508_REAL->RPM*M3508_RM_To_MS;
//	int distance =distance+deta_distance;
 int  time_acc = curve->p_add * curve->aTimes;		//加速时间
int time_con = curve->aTimes-time_acc;//匀速时间
	int time_slow=curve->p_decrease*curve->aTimes;//减速时间
	float TARGET_RPM=0;
    //控制不能超速。
	if(curve->currentSpeed>curve->speedMax)
  {
    curve->currentSpeed=curve->speedMax;
  }
	if((M3508->output>curve->speedMin)&&flag!=1)
	//加速阶段
	{
			while(time_now < time_acc)
		{
				time_now++;
				curve->currentSpeed+=curve->stepSpeed;
				vTaskDelay(1);//1ms
		}
	}
	while(time_now > time_acc&&time_now<time_acc+time_con)
	{
		flag=1;
		
				if(M3508->output>curve->speedMax)
		{
				time_now++;
				curve->currentSpeed=curve->speedMax;
		}
		if(M3508->output<curve->speedMax)
				curve->currentSpeed=M3508->output;
		vTaskDelay(1);
	}
	if((M3508->output>curve->speedMin)&&flag!=1)
	//减速阶段
	{
		while(time_now > time_acc+time_con)
	{
		time_now++;
		curve->currentSpeed-=curve->stepSpeed;
		vTaskDelay(1);
	}
	}
if(M3508->output<curve->speedMin)
	flag=0;
if(time_now>curve->aTimes)
{
time_now=0;
}
    M3508_REAL->TARGET_RPM=curve->currentSpeed;// 分配合适的正负号
		PID_incremental_PID_calculation(M3508, M3508_REAL->RPM ,M3508_REAL->TARGET_RPM);
		M3508_REAL->TARGET_CURRENT=M3508->output;
}


void move()
{
	/***********赋予速度*******************/
		// 速度分解
		World_3wheels
									(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM,
									ROBOT_TARGET_VELOCITY_DATA.Vy_RPM, 
									ROBOT_TARGET_VELOCITY_DATA.W_RPM,
									ROBOT_REAL_POS_DATA.POS_YAW);
		
		//进行PID计算
	
	  PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[0], M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM);
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[1], M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM);
		PID_incremental_PID_calculation(&M3508_CHASSIS_MOTOR_PID_RPM[2], M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM);
	  
	  //MotorVelocityCurve_task();
	
		/****LADRC计算****/
	  //实验中
//		LADRC_Loop(&ADRC_M3508_CHASIS[0],M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR1_RPM);
//		LADRC_Loop(&ADRC_M3508_CHASIS[1],M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR2_RPM);
//	  LADRC_Loop(&ADRC_M3508_CHASIS[2],M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM ,CHASSIS_MOTOR_TARGET_RPM.MOTOR3_RPM);
//		
//		// 设置M3508电流(LADRC)
//		M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT = ADRC_M3508_CHASIS[0].u;
//		M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT = ADRC_M3508_CHASIS[1].u;
//		M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT = ADRC_M3508_CHASIS[2].u;
		/***********设置电流*******************/
		// 设置M3508电流(pid)
		M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[0].output;
		M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[1].output;  
		M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT = M3508_CHASSIS_MOTOR_PID_RPM[2].output;
		
		/***********发送数据*******************/
		chassis_m3508_send_motor_currents_can1();


}	
	
void Free_Control(void)
{
					if(ROCK_L_Y>1460&&ROCK_L_Y<1540)   ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
				 else if(ROCK_L_Y>=1540) ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1540)*2;
				 else if(ROCK_L_Y<=1460) ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1460)*2;
				 
		//     vx=(ROCK_L_X_Processed/10-150.0)*speed_factor;
				 if(ROCK_L_X>1460&&ROCK_L_X<1540)   ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
				 else if(ROCK_L_X>=1540) ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1540)*2;
				 else if(ROCK_L_X<=1460) ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1460)*2;
				 
		//     vw=-(ROCK_R_X_Processed/10-150.0)*speed_factor*1.5;
				 if(ROCK_R_X>1400&&ROCK_R_X<1600)   ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
				 else if(ROCK_R_X>=1600) ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_X-1600)*2;
				 else if(ROCK_R_X<=1400) ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_X-1400)*2;

				//free_up();	


}

void Free_Control_Limit(void)
{
					if(ROCK_L_Y>1160&&ROCK_L_Y<1840)   ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=0;
				 else if(ROCK_L_Y>=1840) ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1840)*0.1;
				 else if(ROCK_L_Y<=1160) ROBOT_TARGET_VELOCITY_DATA.Vy_RPM=(ROCK_L_Y-1160)*0.1;
				 
		//     vx=(ROCK_L_X_Processed/10-150.0)*speed_factor;
				 if(ROCK_L_X>1160&&ROCK_L_X<1840)   ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=0;
				 else if(ROCK_L_X>=1840) ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1840)*0.1;
				 else if(ROCK_L_X<=1160) ROBOT_TARGET_VELOCITY_DATA.Vx_RPM=(ROCK_L_X-1160)*0.1;
				 
		//     vw=-(ROCK_R_X_Processed/10-150.0)*speed_factor*1.5;
				 if(ROCK_R_X>1200&&ROCK_R_X<1800)   ROBOT_TARGET_VELOCITY_DATA.W_RPM=0;
				 else if(ROCK_R_X>=1800) ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_X-1800)*0.1;
				 else if(ROCK_R_X<=1200) ROBOT_TARGET_VELOCITY_DATA.W_RPM=-(ROCK_R_X-1200)*0.1;
					
//				free_up();	




}

void free_up(void)
{
//	if(UP_ARM_NOW_MOTION==&UP_INIT){M3508_UP.TARGET_ANGLE=0;}
//	if(UP_ARM_NOW_MOTION==&UP_ON1){M3508_UP.TARGET_ANGLE=-2700;}
//	if(UP_ARM_NOW_MOTION==&UP_ON2){M3508_UP.TARGET_ANGLE=-10000;}	
//	if(UP_ARM_NOW_MOTION==&UP_ON3){M3508_UP.TARGET_ANGLE=-11000;}
	PID_position_PID_calculation(&M3508_UP_NORMAL,M3508_UP.REAL_INFO.REAL_ANGLE,M3508_UP.TARGET_ANGLE);
	M3508_UP.REAL_INFO.TARGET_RPM=M3508_UP_NORMAL.output;
	PID_incremental_PID_calculation(&M3508_UP.MOTOR_PID,M3508_UP.REAL_INFO.RPM,M3508_UP.REAL_INFO.TARGET_RPM);
	M3508_UP.REAL_INFO.TARGET_CURRENT = M3508_UP.MOTOR_PID.output;
	 if(ROCK_R_Y>1400&&ROCK_R_Y<1600)   M3508_UP.TARGET_ANGLE+=0;
	else if(ROCK_R_Y>=1600) M3508_UP.TARGET_ANGLE-=10;
	else if(ROCK_R_Y<=1400) M3508_UP.TARGET_ANGLE+=10;
	chassis_m3508_m2006_send_motor_currents_can1();

}

void RPM_MOTOR_PLANNING(void)
{
	ad_plan_arm_motor_RPM_UP(*UP_ARM_NOW_MOTION,M3508_UP.REAL_INFO.REAL_ANGLE);
	ad_plan_arm_motor_RPM_YAW(*YAW_ARM_NOW_MOTION,M3508_YAW.REAL_INFO.REAL_ANGLE);
	ad_plan_arm_motor_RPM_TRANSATE1(*TRANSATE_NOW_MOTION,M3508_TRANSATE.REAL_INFO.REAL_ANGLE);
	//	PUSH(0,2000,);
	/***********发送数据*******************/
  chassis_m3508_m2006_send_motor_currents_can1();


}

void PUSH(float start,float end,float speedmax,float speedstart,float speedend,float ac,float de)
{
	TRANSATE_1.Pstart=start;
	TRANSATE_1.Pend=end;
	TRANSATE_1.Vmax=speedmax;
	TRANSATE_1.Vstart=speedstart;
	TRANSATE_1.Vend=speedend;
	TRANSATE_1.Rac=ac;
	TRANSATE_1.Rde=de;
	TRANSATE_NOW_MOTION=&TRANSATE_1;
	if(transate_finished)
	{
		M3508_TRANSATE.REAL_INFO.REAL_ANGLE=0;
		TRANSATE_NOW_MOTION=&TRANSATE_INIT;
	
	}
	chassis_m3508_m2006_send_motor_currents_can1();
}
