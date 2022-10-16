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

    CHASSIS_InitArgument();//初始化底盘相关参数
    
    while(1)
    {
        currentTime = xTaskGetTickCount();//当前系统时间
        if(SystemValue == Starting)
        {
            //Chassis_open_init();
            SystemValue = Running;      //系统初始化结束
        }
        else
        {
            if(rc.sw2==1)//遥控器控制
                actChassis=CHASSIS_FOLLOW_GIMBAL;	//底盘跟随云台
            else if(rc.sw2==3)
                actChassis=CHASSIS_NORMAL;	//底盘不跟随云台
            else if(rc.sw2==2)
                actChassis=CHASSIS_GYROSCOPE;		//小陀螺模式

            RemoteControlChassis();
            CHASSIS_Single_Loop_Out();
        }
        vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
    }
}

eChassisAction actChassis=CHASSIS_NORMAL;   //默认底盘跟随云台行走
eChassisAction actChassis_last = CHASSIS_NORMAL;
Chassis_Speed absolute_chassis_speed;

PidTypeDef Chassis_Follow_PID;
fp32 Chassis_Follow_pid[3]= {0.004,0,0.002}; 

int16_t chassis_3508_setspeed[4];		//四个3508轮子目标转速
fp32 chassis_3508_speed_pid[3] = {20,0,0};
fp32 chassis_3508_Position_nothing[3]= {0,0,0};

/**
  * @brief  底盘参数初始化
  * @param  void
  * @retval void
  * @attention 只在系统启动时调用一次
  */
void CHASSIS_InitArgument()
{
    //底盘跟随专用
    pid_init(&Chassis_Follow_PID);
    Chassis_Follow_PID.f_param_init(&Chassis_Follow_PID,PID_POSITION,Chassis_Follow_pid,3,     0.5,     1e30,  100,   0.2,    8192,         0);
	  
	  //3508
    Motor_Init2(&Chassis_Motor[0],1,chassis_3508_Position_nothing,0,0,chassis_3508_speed_pid,20000,10000);
    Motor_Init2(&Chassis_Motor[1],2,chassis_3508_Position_nothing,0,0,chassis_3508_speed_pid,20000,10000);
    Motor_Init2(&Chassis_Motor[2],3,chassis_3508_Position_nothing,0,0,chassis_3508_speed_pid,20000,10000);
    Motor_Init2(&Chassis_Motor[3],4,chassis_3508_Position_nothing,0,0,chassis_3508_speed_pid,20000,10000);
}

/**
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention
  */
void RemoteControlChassis(void)
{
    switch(actChassis) 
    {
    case CHASSIS_FOLLOW_GIMBAL://跟随云台
        absolute_chassis_speed.vx =(fp32)rc.ch3/300; //前后计算
        absolute_chassis_speed.vy =(fp32)rc.ch4/300; //左右计算
        absolute_chassis_speed.vw=-Chassis_Follow_PID.f_cal_pid(&Chassis_Follow_PID,Find_Y_AnglePNY(),0);//PID使底盘跟随云台速度
        break;
    case CHASSIS_NORMAL://不跟随云台
        absolute_chassis_speed.vx=(fp32)rc.ch3/300;
        absolute_chassis_speed.vy=(fp32)rc.ch4/300;
        absolute_chassis_speed.vw=0;
        break;
    case CHASSIS_GYROSCOPE:		//小陀螺模式
        absolute_chassis_speed.vx=(fp32)rc.ch3/300;
        absolute_chassis_speed.vy=(fp32)rc.ch4/300;
        absolute_chassis_speed.vw=-2;
        break;
		case CHASSIS_SLOW:		//小陀螺模式
        absolute_chassis_speed.vx=0;
        absolute_chassis_speed.vy=0;
        absolute_chassis_speed.vw=0;
        break;
    default:
        break;
    }
}

/**
  * @brief  底盘电机输出
  * @param  void
  * @retval void
  * @attention
  */
void CHASSIS_Single_Loop_Out()
{
    if(actChassis==CHASSIS_FOLLOW_GIMBAL)
    {
      Absolute_Cal(&absolute_chassis_speed,0);//计算各个电机的目标速度
    }
    else
    {
      Absolute_Cal(&absolute_chassis_speed,(fp32)(Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1)*0.043945f);//计算各个电机的目标速度
    }
     Omni_Set_Motor_Speed(chassis_3508_setspeed,Chassis_Motor);//设置各个电机的目标速度

 		 //Chassis_Power_Limit();//功率限制
    /************************************底盘3508电机速度环计算*********************************************/
    Chassis_Motor[0].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[0].Motor_PID_Speed,Chassis_Motor[0].motor_value->speed_rpm,
            Chassis_Motor[0].motor_value->target_speed_rpm);
    Chassis_Motor[1].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[1].Motor_PID_Speed,Chassis_Motor[1].motor_value->speed_rpm,
            Chassis_Motor[1].motor_value->target_speed_rpm);
    Chassis_Motor[2].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[2].Motor_PID_Speed,Chassis_Motor[2].motor_value->speed_rpm,
            Chassis_Motor[2].motor_value->target_speed_rpm);
    Chassis_Motor[3].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[3].Motor_PID_Speed,Chassis_Motor[3].motor_value->speed_rpm,
            Chassis_Motor[3].motor_value->target_speed_rpm);
    /************************************将电流参数发送给电机*********************************************/
    set_moto1234_current(&hcan2,Chassis_Motor[0].Motor_PID_Speed.out,Chassis_Motor[1].Motor_PID_Speed.out,Chassis_Motor[2].Motor_PID_Speed.out,Chassis_Motor[3].Motor_PID_Speed.out);
}


/**
  * @brief  将云台坐标转换为底盘坐标
  * @param  absolute_speed 绝对坐标需要的速度 
  * @param  angle 云台相对于底盘的角度
  * @retval 偏差角，角度制
  * @attention
  */
void Absolute_Cal(Chassis_Speed* absolute_speed, fp32 angle)
{
    fp32 angle_hd=angle* PI / 180;
    Chassis_Speed temp_speed;
    temp_speed.vw = absolute_speed->vw;
    temp_speed.vx = absolute_speed->vx*cos(angle_hd)-absolute_speed->vy*sin(angle_hd);
    temp_speed.vy = absolute_speed->vx*sin(angle_hd)+absolute_speed->vy*cos(angle_hd);

    //保证底盘是相对摄像头做移动，当摄像头转过90度时x方向速度从1变0，
    //y方向速度从0变1，保证视觉上是相对右移
  
    Omni_calc(&temp_speed,chassis_3508_setspeed);
}

void Omni_calc(Chassis_Speed *speed, int16_t* out_speed) 
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER*3.14f)*CHASSIS_DECELE_RATIO*1000;

    wheel_rpm[0] = (   speed->vx + speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = (   speed->vx - speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//forward
    wheel_rpm[2] = (  -speed->vx - speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//right
    wheel_rpm[3] = (  -speed->vx + speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//back

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}
/**
  * @brief  设置3508目标速度
  * @param  out_speed 目标速度
  * @param  Motor 电机结构体
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
  * @brief  找出与+-y轴最小偏差角
  * @param  void
  * @retval 偏差角，角度制
  * @attention 通过遥控器/键盘
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


