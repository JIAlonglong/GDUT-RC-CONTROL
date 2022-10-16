#include "chassis_task.h"
#include "arm_math.h"
#include "param.h"
#include "type.h"
#include "judge.h"
#include "cap.h"
#include "protect_task.h"

void ChassisFun(void const * argument) 
{
    portTickType currentTime;

    CHASSIS_InitArgument();//��ʼ��������ز���
    
    while(1)
    {
        currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
        if(SystemValue == Starting)
        {
            Chassis_open_init();
            SystemValue = Running;      //ϵͳ��ʼ������
        }
        else
        {
            if(rc.sw2==1)//ң��������
                actChassis=CHASSIS_FOLLOW_GIMBAL;	//���̸�����̨
            else if(rc.sw2==3)
                actChassis=CHASSIS_NORMAL;	//���̲�������̨
            else if(rc.sw2==2)
                actChassis=CHASSIS_GYROSCOPE;		//С����ģʽ

            RemoteControlChassis();
            CHASSIS_Single_Loop_Out();
        }
        vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
    }
}

eChassisAction actChassis=CHASSIS_NORMAL;   //Ĭ�ϵ��̸�����̨����
eChassisAction actChassis_last = CHASSIS_NORMAL;
Chassis_Speed absolute_chassis_speed;

PidTypeDef Chassis_Follow_PID;
fp32 Chassis_Follow_pid[3]= {0.004,0,0.002};  //0.00018,0.00001,0.002 ////0.00015,0,0.005

int16_t chassis_3508_setspeed[4];		//�ĸ�3508����Ŀ��ת��
fp32 chassis_6020_setangle[4];		//�ĸ�6020����Ŀ��Ƕ�
fp32 chassis_6020_speed_pid[3]= {5,0,5};
fp32 chassis_6020_Position_pid[3]= {10,0.5,0};	//6020λ�û�pid
fp32 chassis_3508_speed_pid[3] = {20,0,0};
fp32 chassis_3508_Position_nothing[3]= {0,0,0};

/**
  * @brief  ���̲�����ʼ��
  * @param  void
  * @retval void
  * @attention ֻ��ϵͳ����ʱ����һ��
  */
void CHASSIS_InitArgument()
{
    //���̸���ר��
    pid_init(&Chassis_Follow_PID);
    Chassis_Follow_PID.f_param_init(&Chassis_Follow_PID,PID_POSITION,Chassis_Follow_pid,3,     0.5,     1e30,  100,   0.2,    8192,         0);
	  
	  //3508
    Motor_Init2(&Chassis_Motor[0],1,chassis_3508_Position_nothing,0,0,chassis_3508_speed_pid,20000,10000);
    Motor_Init2(&Chassis_Motor[1],2,chassis_3508_Position_nothing,0,0,chassis_3508_speed_pid,20000,10000);
    Motor_Init2(&Chassis_Motor[2],3,chassis_3508_Position_nothing,0,0,chassis_3508_speed_pid,20000,10000);
    Motor_Init2(&Chassis_Motor[3],4,chassis_3508_Position_nothing,0,0,chassis_3508_speed_pid,20000,10000);
	  //6020
	  Motor_Init3(&Chassis_Motor[4],5,chassis_6020_Position_pid,4000,2000,chassis_6020_speed_pid,10000,5000);
		Motor_Init3(&Chassis_Motor[5],6,chassis_6020_Position_pid,4000,2000,chassis_6020_speed_pid,10000,5000);
		Motor_Init3(&Chassis_Motor[6],7,chassis_6020_Position_pid,4000,2000,chassis_6020_speed_pid,10000,5000);
		Motor_Init3(&Chassis_Motor[7],8,chassis_6020_Position_pid,4000,2000,chassis_6020_speed_pid,10000,5000);
}


/**
  * @brief  ���̵�����
  * @param  void
  * @retval void
  * @attention
  */
void CHASSIS_Single_Loop_Out()
{
    if(actChassis==CHASSIS_FOLLOW_GIMBAL)
    {
      Absolute_Cal(&absolute_chassis_speed,0);//������������Ŀ���ٶ�
    }
    else
    {
      Absolute_Cal(&absolute_chassis_speed,(fp32)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//������������Ŀ���ٶ�
    }
    AGV_Set_Motor_Speed(chassis_3508_setspeed,Chassis_Motor);//���ø��������Ŀ���ٶ�
	AGV_Set_Motor_angle(chassis_6020_setangle,Chassis_Motor);//���ø��������Ŀ��Ƕ�
 		 //Chassis_Power_Limit();//��������
    /************************************����3508����ٶȻ�����*********************************************/
    Chassis_Motor[0].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[0].Motor_PID_Speed,Chassis_Motor[0].motor_value->speed_rpm,
            -Chassis_Motor[0].motor_value->target_speed_rpm);
    Chassis_Motor[1].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[1].Motor_PID_Speed,Chassis_Motor[1].motor_value->speed_rpm,
            +Chassis_Motor[1].motor_value->target_speed_rpm);
    Chassis_Motor[2].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[2].Motor_PID_Speed,Chassis_Motor[2].motor_value->speed_rpm,
            +Chassis_Motor[2].motor_value->target_speed_rpm);
    Chassis_Motor[3].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[3].Motor_PID_Speed,Chassis_Motor[3].motor_value->speed_rpm,
            -Chassis_Motor[3].motor_value->target_speed_rpm);
	/************************************����6020���λ�û��ٶȻ�����*********************************************/
	Chassis_Motor[4].Motor_PID_Position.f_cal_pid(&Chassis_Motor[4].Motor_PID_Position, Chassis_Motor[4].motor_value->angle, Chassis_Motor[4].motor_value->target_angle);
    Chassis_Motor[4].motor_value->target_speed_rpm = Chassis_Motor[4].Motor_PID_Position.out; //λ�û�
    Chassis_Motor[4].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[4].Motor_PID_Speed,Chassis_Motor[4].motor_value->speed_rpm, //�ٶȻ�
            +Chassis_Motor[4].motor_value->target_speed_rpm);
	
	Chassis_Motor[5].Motor_PID_Position.f_cal_pid(&Chassis_Motor[5].Motor_PID_Position, Chassis_Motor[5].motor_value->angle, Chassis_Motor[5].motor_value->target_angle);
    Chassis_Motor[5].motor_value->target_speed_rpm = Chassis_Motor[5].Motor_PID_Position.out; //λ�û�
    Chassis_Motor[5].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[5].Motor_PID_Speed, Chassis_Motor[5].motor_value->speed_rpm, //�ٶȻ�
            +Chassis_Motor[5].motor_value->target_speed_rpm);
		
	Chassis_Motor[6].Motor_PID_Position.f_cal_pid(&Chassis_Motor[6].Motor_PID_Position, Chassis_Motor[6].motor_value->angle, Chassis_Motor[6].motor_value->target_angle);
    Chassis_Motor[6].motor_value->target_speed_rpm = Chassis_Motor[6].Motor_PID_Position.out; //λ�û�
    Chassis_Motor[6].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[6].Motor_PID_Speed, Chassis_Motor[6].motor_value->speed_rpm, //�ٶȻ�
            +Chassis_Motor[6].motor_value->target_speed_rpm);
	
	Chassis_Motor[7].Motor_PID_Position.f_cal_pid(&Chassis_Motor[7].Motor_PID_Position, Chassis_Motor[7].motor_value->angle, Chassis_Motor[7].motor_value->target_angle);
    Chassis_Motor[7].motor_value->target_speed_rpm = Chassis_Motor[7].Motor_PID_Position.out; //λ�û�
    Chassis_Motor[7].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[7].Motor_PID_Speed, Chassis_Motor[7].motor_value->speed_rpm, //�ٶȻ�
            +Chassis_Motor[7].motor_value->target_speed_rpm);
    /************************************�������������͸����*********************************************/

    set_moto1234_current(&hcan2,Chassis_Motor[0].Motor_PID_Speed.out,Chassis_Motor[1].Motor_PID_Speed.out,Chassis_Motor[2].Motor_PID_Speed.out,Chassis_Motor[3].Motor_PID_Speed.out);
    set_moto5678_current(&hcan1,Chassis_Motor[4].Motor_PID_Speed.out,Chassis_Motor[5].Motor_PID_Speed.out,Chassis_Motor[6].Motor_PID_Speed.out,Chassis_Motor[7].Motor_PID_Speed.out);
}


/**
  * @brief  ����̨����ת��Ϊ��������
  * @param  absolute_speed ����������Ҫ���ٶ� 
  * @param  angle ��̨����ڵ��̵ĽǶ�
  * @retval ƫ��ǣ��Ƕ���
  * @attention
  */
void Absolute_Cal(Chassis_Speed* absolute_speed, fp32 angle)
{
    fp32 angle_hd=angle* PI / 180;
    Chassis_Speed temp_speed;
    temp_speed.vw = absolute_speed->vw; 
    temp_speed.vx = absolute_speed->vx * cos(angle_hd) - absolute_speed->vy * sin(angle_hd);
    temp_speed.vy = absolute_speed->vx * sin(angle_hd) + absolute_speed->vy * cos(angle_hd);
	AGV_angle_calc(&temp_speed,chassis_6020_setangle);
    AGV_calc(&temp_speed,chassis_3508_setspeed);
}

/**
  * @brief  ����������������Ŀ���ٶ�
  * @param  speed ����������ٶ� 
  * @param  out_speed 3508Ŀ���ٶ�
  * @retval 
  * @attention
  */
int8_t drct=1;//���������������ת
void AGV_calc(Chassis_Speed *speed, int16_t* out_speed) 
{
	  //3508Ŀ���ٶȼ���
    int16_t wheel_rpm[4];
    fp32 wheel_rpm_ratio;
    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;
    
    wheel_rpm[0] = sqrt(	pow(speed->vy + speed->vw * Radius * 0.707107f,2)
                       +	pow(speed->vx - speed->vw * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio;
    wheel_rpm[1] = sqrt(	pow(speed->vy - speed->vw * Radius * 0.707107f,2)
                       +	pow(speed->vx - speed->vw * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio;
    wheel_rpm[2] = sqrt(	pow(speed->vy - speed->vw * Radius * 0.707107f,2)
                       +	pow(speed->vx + speed->vw * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio;
    wheel_rpm[3] = sqrt(	pow(speed->vy + speed->vw * Radius * 0.707107f,2)
                       +	pow(speed->vx + speed->vw * Radius * 0.707107f,2) 
                       ) * wheel_rpm_ratio;
	  
    for(int i=0;i<4;i++)
       out_speed[i] = drct * wheel_rpm[i];
}

/**
  * @brief  ������̺�������Ŀ��Ƕ�
  * @param  speed ����������ٶ� 
  * @param  out_angle 6020Ŀ��Ƕ�
  * @retval 
  * @attention
  */
bool flag=1;
void AGV_angle_calc(Chassis_Speed *speed, fp32* out_angle) 
{
    int16_t angle_temp;
    fp32 wheel_angle[4]; //6020������Ŀ��Ƕ�
    fp32 wheel_angle_last[4];
    fp64 atan_angle[4];   
	  //6020Ŀ��Ƕȼ���
    if(!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0))//��ֹ����Ϊ��
    {
      atan_angle[0]=atan2((speed->vx - speed->vw*Radius*0.707107f),(speed->vy + speed->vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[1]=atan2((speed->vx - speed->vw*Radius*0.707107f),(speed->vy - speed->vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[2]=atan2((speed->vx + speed->vw*Radius*0.707107f),(speed->vy - speed->vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[3]=atan2((speed->vx + speed->vw*Radius*0.707107f),(speed->vy + speed->vw*Radius*0.707107f))*180.0f/PI;	
    }  

		wheel_angle[0] = CHASSIS_6020_1_Y_ANGLE + (fp32)(atan_angle[0]*22.75);
		wheel_angle[1] = CHASSIS_6020_2_Y_ANGLE + (fp32)(atan_angle[1]*22.75);
		wheel_angle[2] = CHASSIS_6020_3_Y_ANGLE + (fp32)(atan_angle[2]*22.75);
		wheel_angle[3] = CHASSIS_6020_4_Y_ANGLE + (fp32)(atan_angle[3]*22.75);

		AngleLoop_f(&wheel_angle[0],8192);//�����ͽǶȻػ�
		AngleLoop_f(&wheel_angle[1],8192);
		AngleLoop_f(&wheel_angle[2],8192);
		AngleLoop_f(&wheel_angle[3],8192);
    
		switch(actChassis)
		{
			case CHASSIS_NORMAL:
			{	
				angle_temp=Chassis_Motor[4].motor_value->angle;
				AngleLoop_int(&angle_temp,8192);//���ͽǶȻػ�
				if(fabs(Find_min_Angle(angle_temp,wheel_angle[0]))>2048)
				{
					for(int i=0;i<4;i++)
						wheel_angle[i] += 4096;				
					if(flag) //���������ת ʹ����ִ��ʱֻ��ȡ��һ��
					{
						drct = -drct;
						flag=0;
					}
				}
				else
					  flag=1;
				
				if((speed->vx == 0 || speed->vy == 0)&&speed->vw==0)
				{	 
					 flag=1;
					 drct=1;
			  }
        
				AngleLoop_f(&wheel_angle[0],8192);//�ٴνǶȻػ�,��Ϊ�ӹ�4096
				AngleLoop_f(&wheel_angle[1],8192);
				AngleLoop_f(&wheel_angle[2],8192);
				AngleLoop_f(&wheel_angle[3],8192);
				break;
			}
		  case CHASSIS_FOLLOW_GIMBAL:
		  {
				angle_temp=Chassis_Motor[4].motor_value->angle;
				AngleLoop_int(&angle_temp,8192);
				if(fabs(Find_min_Angle(angle_temp,wheel_angle[0]))>2048)
				{
					for(int i=0;i<4;i++)
						wheel_angle[i] += 4096;		
                    drct=-1;										
				}
				else
					drct=1;
			
				AngleLoop_f(&wheel_angle[0],8192);
				AngleLoop_f(&wheel_angle[1],8192);
				AngleLoop_f(&wheel_angle[2],8192);
				AngleLoop_f(&wheel_angle[3],8192);
				break;
			}
			case CHASSIS_GYROSCOPE:
			{
				//��Ҫ�ټ�			
				break;
			}
			default:break;
	}
    
  if(speed->vx == 0 && speed->vy == 0 && speed->vw == 0)//ҡ�˻���ʱ
  {
    for(int i=0;i<4;i++)//memcpy��������
     out_angle[i] = wheel_angle_last[i];
  }
  else
  {
    for(int i=0;i<4;i++)
    {
     out_angle[i] = wheel_angle[i];
     wheel_angle_last[i] = wheel_angle[i];
    }
  }
}

/**
  * @brief  ����3508Ŀ���ٶ�
  * @param  out_speed Ŀ���ٶ�
  * @param  Motor ����ṹ��
  * @retval 
  * @attention 
  */
void AGV_Set_Motor_Speed(int16_t*out_speed,Motortype* Motor ) {
    Motor[0].motor_value->target_speed_rpm=out_speed[0];
    Motor[1].motor_value->target_speed_rpm=out_speed[1];
    Motor[2].motor_value->target_speed_rpm=out_speed[2];
    Motor[3].motor_value->target_speed_rpm=out_speed[3];
}

/**
  * @brief  ����6020Ŀ��Ƕ�
  * @param  out_angle Ŀ��Ƕ�
  * @param  Motor ����ṹ��
  * @retval 
  * @attention 
  */
void AGV_Set_Motor_angle(fp32 *out_angle,Motortype* Motor ) {
    Motor[4].motor_value->target_angle=out_angle[0];
    Motor[5].motor_value->target_angle=out_angle[1];
    Motor[6].motor_value->target_angle=out_angle[2];
    Motor[7].motor_value->target_angle=out_angle[3];
}

/**
  * @brief  �ҳ���+-y����Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
  */
fp32 Find_Y_AnglePNY(void)
{
    fp32 temp1 = Gimbal_MotorYaw.motor_value->angle - GIMBAL_YAW_ENCODER_MIDDLE1;
    fp32 temp2 = Gimbal_MotorYaw.motor_value->angle - GIMBAL_YAW_ENCODER_MIDDLE2;
    fp32 mintemp1;
    if(temp1 > 4096)
        temp1 -= 8192;
    else if(temp1 < -4096)
        temp1 += 8192;
    if(temp2 > 4096)
        temp2 -= 8192;
    else if(temp2 < -4096)
        temp2 += 8192;

    mintemp1 = (abs((int32_t)temp1) < abs((int32_t)temp2) ? temp1 : temp2);
		if(fabs(mintemp1)<50)
			 mintemp1=0;
    return mintemp1;
}

/**
  * @brief  �ҳ����ǵĽ�С��ֵ
  * @param  ��1����2
  * @retval 
  * @attention 
  */
fp32 Find_min_Angle(int16_t angle1,fp32 angle2)
{
	  fp32 err;
    err = (fp32)angle1 - angle2;
    if(fabs(err) > 4096)
    {
        err = 8192 - fabs(err);
    }
    return err;
}

/**
  * @brief  ң�������Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention
  */
void RemoteControlChassis(void) {
    /***********************************ȷ�������ĸ������Ŀ���ٶ�*****************************************/
    switch(actChassis) {
    case CHASSIS_FOLLOW_GIMBAL://������̨
        absolute_chassis_speed.vx =(fp32)rc.ch3/300; //ǰ�����
        absolute_chassis_speed.vy =(fp32)rc.ch4/300; //���Ҽ���
        absolute_chassis_speed.vw=-Chassis_Follow_PID.f_cal_pid(&Chassis_Follow_PID,Find_Y_AnglePNY(),0);//PIDʹ���̸�����̨�ٶ�
        break;
    case CHASSIS_NORMAL://��������̨
        absolute_chassis_speed.vx=(fp32)rc.ch3/300;
        absolute_chassis_speed.vy=(fp32)rc.ch4/300;
        absolute_chassis_speed.vw=0;
        break;
    case CHASSIS_GYROSCOPE:		//С����ģʽ
        absolute_chassis_speed.vx=(fp32)rc.ch3/300;
        absolute_chassis_speed.vy=(fp32)rc.ch4/300;
        absolute_chassis_speed.vw=-2;
        break;
		case CHASSIS_SLOW:		//С����ģʽ
        absolute_chassis_speed.vx=0;
        absolute_chassis_speed.vy=0;
        absolute_chassis_speed.vw=0;
        break;
    default:
        break;
    }
}

/**
  * @brief  ��������ʼ����ͳһת����λ
  * @param  void
  * @retval void
  * @attention
  */
void Chassis_open_init(void)
{
    bool return_flag = true;
    uint16_t cnt=0;
    while(return_flag)
    {
     if(cnt > 1500)//��ʱ
     {
       return_flag = false;
     }

        /************************************���̵���ٶȻ�����*********************************************/
      cnt++;
     
      if(cnt%2==0)
        Chassis_Motor[4].Motor_PID_Position.f_cal_pid(&Chassis_Motor[4].Motor_PID_Position, 
                                                       Chassis_Motor[4].motor_value->angle, 
                                                       CHASSIS_6020_1_Y_ANGLE);
      
      Chassis_Motor[4].motor_value->target_speed_rpm = Chassis_Motor[4].Motor_PID_Position.out; //λ�û�
      Chassis_Motor[4].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[4].Motor_PID_Speed,Chassis_Motor[4].motor_value->speed_rpm, //�ٶȻ�
              +Chassis_Motor[4].motor_value->target_speed_rpm);
      
      if(cnt%2==0)
        Chassis_Motor[5].Motor_PID_Position.f_cal_pid(&Chassis_Motor[5].Motor_PID_Position, 
                                                       Chassis_Motor[5].motor_value->angle, 
                                                       CHASSIS_6020_2_Y_ANGLE);
      
      Chassis_Motor[5].motor_value->target_speed_rpm = Chassis_Motor[5].Motor_PID_Position.out; //λ�û�
      Chassis_Motor[5].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[5].Motor_PID_Speed, Chassis_Motor[5].motor_value->speed_rpm, //�ٶȻ�
              +Chassis_Motor[5].motor_value->target_speed_rpm);
      
      if(cnt%2==0)
        Chassis_Motor[6].Motor_PID_Position.f_cal_pid(&Chassis_Motor[6].Motor_PID_Position, 
                                                       Chassis_Motor[6].motor_value->angle, 
                                                       CHASSIS_6020_3_Y_ANGLE);
      
      Chassis_Motor[6].motor_value->target_speed_rpm = Chassis_Motor[6].Motor_PID_Position.out; //λ�û�
      Chassis_Motor[6].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[6].Motor_PID_Speed, Chassis_Motor[6].motor_value->speed_rpm, //�ٶȻ�
              +Chassis_Motor[6].motor_value->target_speed_rpm);
      
      if(cnt%2==0)
        Chassis_Motor[7].Motor_PID_Position.f_cal_pid(&Chassis_Motor[7].Motor_PID_Position, 
                                                       Chassis_Motor[7].motor_value->angle, 
                                                       CHASSIS_6020_4_Y_ANGLE);
      
      Chassis_Motor[7].motor_value->target_speed_rpm = Chassis_Motor[7].Motor_PID_Position.out; //λ�û�
      Chassis_Motor[7].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[7].Motor_PID_Speed, Chassis_Motor[7].motor_value->speed_rpm, //�ٶȻ�
              +Chassis_Motor[7].motor_value->target_speed_rpm);
      
      /************************************�������������͸����*********************************************/
      set_gimbal_current (&hcan1,Chassis_Motor[4].Motor_PID_Speed.out,
                                 Chassis_Motor[5].Motor_PID_Speed.out,
                                 Chassis_Motor[6].Motor_PID_Speed.out,
                                 Chassis_Motor[7].Motor_PID_Speed.out);
      osDelay(1); //pid����Ƶ��1000hz
    }
}

