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
            //Chassis_open_init();
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
fp32 Chassis_Follow_pid[3]= {0.004,0,0.002}; 

int16_t chassis_3508_setspeed[4];		//�ĸ�3508����Ŀ��ת��
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
}

/**
  * @brief  ң�������Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention
  */
void RemoteControlChassis(void)
{
    switch(actChassis) 
    {
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
     Omni_Set_Motor_Speed(chassis_3508_setspeed,Chassis_Motor);//���ø��������Ŀ���ٶ�

 		 //Chassis_Power_Limit();//��������
    /************************************����3508����ٶȻ�����*********************************************/
    Chassis_Motor[0].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[0].Motor_PID_Speed,Chassis_Motor[0].motor_value->speed_rpm,
            Chassis_Motor[0].motor_value->target_speed_rpm);
    Chassis_Motor[1].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[1].Motor_PID_Speed,Chassis_Motor[1].motor_value->speed_rpm,
            Chassis_Motor[1].motor_value->target_speed_rpm);
    Chassis_Motor[2].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[2].Motor_PID_Speed,Chassis_Motor[2].motor_value->speed_rpm,
            Chassis_Motor[2].motor_value->target_speed_rpm);
    Chassis_Motor[3].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[3].Motor_PID_Speed,Chassis_Motor[3].motor_value->speed_rpm,
            Chassis_Motor[3].motor_value->target_speed_rpm);
    /************************************�������������͸����*********************************************/
    set_moto1234_current(&hcan2,Chassis_Motor[0].Motor_PID_Speed.out,Chassis_Motor[1].Motor_PID_Speed.out,Chassis_Motor[2].Motor_PID_Speed.out,Chassis_Motor[3].Motor_PID_Speed.out);
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
    temp_speed.vx = absolute_speed->vx*cos(angle_hd)-absolute_speed->vy*sin(angle_hd);
    temp_speed.vy = absolute_speed->vx*sin(angle_hd)+absolute_speed->vy*cos(angle_hd);

    //��֤�������������ͷ���ƶ���������ͷת��90��ʱx�����ٶȴ�1��0��
    //y�����ٶȴ�0��1����֤�Ӿ������������
  
    Omni_calc(&temp_speed,chassis_3508_setspeed);
}

void Omni_calc(Chassis_Speed *speed, int16_t* out_speed) 
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER*3.14f)*CHASSIS_DECELE_RATIO*1000;

    wheel_rpm[0] = (   speed->vx + speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//left//x��y�����ٶ�,w����ת���ٶ�
    wheel_rpm[1] = (   speed->vx - speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//forward
    wheel_rpm[2] = (  -speed->vx - speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//right
    wheel_rpm[3] = (  -speed->vx + speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//back

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}
/**
  * @brief  ����3508Ŀ���ٶ�
  * @param  out_speed Ŀ���ٶ�
  * @param  Motor ����ṹ��
  * @retval 
  * @attention 
  */
void Omni_Set_Motor_Speed(int16_t*out_speed,Motortype* Motor ) 
{
    Motor[0].motor_value->target_speed_rpm=out_speed[0];
    Motor[1].motor_value->target_speed_rpm=out_speed[1];
    Motor[2].motor_value->target_speed_rpm=out_speed[2];
    Motor[3].motor_value->target_speed_rpm=out_speed[3];
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


