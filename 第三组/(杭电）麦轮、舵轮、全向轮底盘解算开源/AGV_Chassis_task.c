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
            Chassis_open_init();
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
fp32 Chassis_Follow_pid[3]= {0.004,0,0.002};  //0.00018,0.00001,0.002 ////0.00015,0,0.005

int16_t chassis_3508_setspeed[4];		//四个3508轮子目标转速
fp32 chassis_6020_setangle[4];		//四个6020轮子目标角度
fp32 chassis_6020_speed_pid[3]= {5,0,5};
fp32 chassis_6020_Position_pid[3]= {10,0.5,0};	//6020位置环pid
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
	  //6020
	  Motor_Init3(&Chassis_Motor[4],5,chassis_6020_Position_pid,4000,2000,chassis_6020_speed_pid,10000,5000);
		Motor_Init3(&Chassis_Motor[5],6,chassis_6020_Position_pid,4000,2000,chassis_6020_speed_pid,10000,5000);
		Motor_Init3(&Chassis_Motor[6],7,chassis_6020_Position_pid,4000,2000,chassis_6020_speed_pid,10000,5000);
		Motor_Init3(&Chassis_Motor[7],8,chassis_6020_Position_pid,4000,2000,chassis_6020_speed_pid,10000,5000);
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
    AGV_Set_Motor_Speed(chassis_3508_setspeed,Chassis_Motor);//设置各个电机的目标速度
	AGV_Set_Motor_angle(chassis_6020_setangle,Chassis_Motor);//设置各个电机的目标角度
 		 //Chassis_Power_Limit();//功率限制
    /************************************底盘3508电机速度环计算*********************************************/
    Chassis_Motor[0].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[0].Motor_PID_Speed,Chassis_Motor[0].motor_value->speed_rpm,
            -Chassis_Motor[0].motor_value->target_speed_rpm);
    Chassis_Motor[1].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[1].Motor_PID_Speed,Chassis_Motor[1].motor_value->speed_rpm,
            +Chassis_Motor[1].motor_value->target_speed_rpm);
    Chassis_Motor[2].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[2].Motor_PID_Speed,Chassis_Motor[2].motor_value->speed_rpm,
            +Chassis_Motor[2].motor_value->target_speed_rpm);
    Chassis_Motor[3].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[3].Motor_PID_Speed,Chassis_Motor[3].motor_value->speed_rpm,
            -Chassis_Motor[3].motor_value->target_speed_rpm);
	/************************************底盘6020电机位置环速度环计算*********************************************/
	Chassis_Motor[4].Motor_PID_Position.f_cal_pid(&Chassis_Motor[4].Motor_PID_Position, Chassis_Motor[4].motor_value->angle, Chassis_Motor[4].motor_value->target_angle);
    Chassis_Motor[4].motor_value->target_speed_rpm = Chassis_Motor[4].Motor_PID_Position.out; //位置环
    Chassis_Motor[4].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[4].Motor_PID_Speed,Chassis_Motor[4].motor_value->speed_rpm, //速度环
            +Chassis_Motor[4].motor_value->target_speed_rpm);
	
	Chassis_Motor[5].Motor_PID_Position.f_cal_pid(&Chassis_Motor[5].Motor_PID_Position, Chassis_Motor[5].motor_value->angle, Chassis_Motor[5].motor_value->target_angle);
    Chassis_Motor[5].motor_value->target_speed_rpm = Chassis_Motor[5].Motor_PID_Position.out; //位置环
    Chassis_Motor[5].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[5].Motor_PID_Speed, Chassis_Motor[5].motor_value->speed_rpm, //速度环
            +Chassis_Motor[5].motor_value->target_speed_rpm);
		
	Chassis_Motor[6].Motor_PID_Position.f_cal_pid(&Chassis_Motor[6].Motor_PID_Position, Chassis_Motor[6].motor_value->angle, Chassis_Motor[6].motor_value->target_angle);
    Chassis_Motor[6].motor_value->target_speed_rpm = Chassis_Motor[6].Motor_PID_Position.out; //位置环
    Chassis_Motor[6].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[6].Motor_PID_Speed, Chassis_Motor[6].motor_value->speed_rpm, //速度环
            +Chassis_Motor[6].motor_value->target_speed_rpm);
	
	Chassis_Motor[7].Motor_PID_Position.f_cal_pid(&Chassis_Motor[7].Motor_PID_Position, Chassis_Motor[7].motor_value->angle, Chassis_Motor[7].motor_value->target_angle);
    Chassis_Motor[7].motor_value->target_speed_rpm = Chassis_Motor[7].Motor_PID_Position.out; //位置环
    Chassis_Motor[7].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[7].Motor_PID_Speed, Chassis_Motor[7].motor_value->speed_rpm, //速度环
            +Chassis_Motor[7].motor_value->target_speed_rpm);
    /************************************将电流参数发送给电机*********************************************/

    set_moto1234_current(&hcan2,Chassis_Motor[0].Motor_PID_Speed.out,Chassis_Motor[1].Motor_PID_Speed.out,Chassis_Motor[2].Motor_PID_Speed.out,Chassis_Motor[3].Motor_PID_Speed.out);
    set_moto5678_current(&hcan1,Chassis_Motor[4].Motor_PID_Speed.out,Chassis_Motor[5].Motor_PID_Speed.out,Chassis_Motor[6].Motor_PID_Speed.out,Chassis_Motor[7].Motor_PID_Speed.out);
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
    temp_speed.vx = absolute_speed->vx * cos(angle_hd) - absolute_speed->vy * sin(angle_hd);
    temp_speed.vy = absolute_speed->vx * sin(angle_hd) + absolute_speed->vy * cos(angle_hd);
	AGV_angle_calc(&temp_speed,chassis_6020_setangle);
    AGV_calc(&temp_speed,chassis_3508_setspeed);
}

/**
  * @brief  计算底盘驱动电机的目标速度
  * @param  speed 底盘坐标的速度 
  * @param  out_speed 3508目标速度
  * @retval 
  * @attention
  */
int8_t drct=1;//决定驱动电机正反转
void AGV_calc(Chassis_Speed *speed, int16_t* out_speed) 
{
	  //3508目标速度计算
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
  * @brief  计算底盘航向电机的目标角度
  * @param  speed 底盘坐标的速度 
  * @param  out_angle 6020目标角度
  * @retval 
  * @attention
  */
bool flag=1;
void AGV_angle_calc(Chassis_Speed *speed, fp32* out_angle) 
{
    int16_t angle_temp;
    fp32 wheel_angle[4]; //6020编码器目标角度
    fp32 wheel_angle_last[4];
    fp64 atan_angle[4];   
	  //6020目标角度计算
    if(!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0))//防止除数为零
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

		AngleLoop_f(&wheel_angle[0],8192);//浮点型角度回环
		AngleLoop_f(&wheel_angle[1],8192);
		AngleLoop_f(&wheel_angle[2],8192);
		AngleLoop_f(&wheel_angle[3],8192);
    
		switch(actChassis)
		{
			case CHASSIS_NORMAL:
			{	
				angle_temp=Chassis_Motor[4].motor_value->angle;
				AngleLoop_int(&angle_temp,8192);//整型角度回环
				if(fabs(Find_min_Angle(angle_temp,wheel_angle[0]))>2048)
				{
					for(int i=0;i<4;i++)
						wheel_angle[i] += 4096;				
					if(flag) //驱动电机反转 使程序执行时只能取反一次
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
        
				AngleLoop_f(&wheel_angle[0],8192);//再次角度回环,因为加过4096
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
				//需要再加			
				break;
			}
			default:break;
	}
    
  if(speed->vx == 0 && speed->vy == 0 && speed->vw == 0)//摇杆回中时
  {
    for(int i=0;i<4;i++)//memcpy狗都不用
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
  * @brief  设置3508目标速度
  * @param  out_speed 目标速度
  * @param  Motor 电机结构体
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
  * @brief  设置6020目标角度
  * @param  out_angle 目标角度
  * @param  Motor 电机结构体
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

/**
  * @brief  找出两角的较小差值
  * @param  角1，角2
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
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention
  */
void RemoteControlChassis(void) {
    /***********************************确定底盘四个电机的目标速度*****************************************/
    switch(actChassis) {
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
  * @brief  航向电机初始化，统一转到零位
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
     if(cnt > 1500)//超时
     {
       return_flag = false;
     }

        /************************************底盘电机速度环计算*********************************************/
      cnt++;
     
      if(cnt%2==0)
        Chassis_Motor[4].Motor_PID_Position.f_cal_pid(&Chassis_Motor[4].Motor_PID_Position, 
                                                       Chassis_Motor[4].motor_value->angle, 
                                                       CHASSIS_6020_1_Y_ANGLE);
      
      Chassis_Motor[4].motor_value->target_speed_rpm = Chassis_Motor[4].Motor_PID_Position.out; //位置环
      Chassis_Motor[4].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[4].Motor_PID_Speed,Chassis_Motor[4].motor_value->speed_rpm, //速度环
              +Chassis_Motor[4].motor_value->target_speed_rpm);
      
      if(cnt%2==0)
        Chassis_Motor[5].Motor_PID_Position.f_cal_pid(&Chassis_Motor[5].Motor_PID_Position, 
                                                       Chassis_Motor[5].motor_value->angle, 
                                                       CHASSIS_6020_2_Y_ANGLE);
      
      Chassis_Motor[5].motor_value->target_speed_rpm = Chassis_Motor[5].Motor_PID_Position.out; //位置环
      Chassis_Motor[5].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[5].Motor_PID_Speed, Chassis_Motor[5].motor_value->speed_rpm, //速度环
              +Chassis_Motor[5].motor_value->target_speed_rpm);
      
      if(cnt%2==0)
        Chassis_Motor[6].Motor_PID_Position.f_cal_pid(&Chassis_Motor[6].Motor_PID_Position, 
                                                       Chassis_Motor[6].motor_value->angle, 
                                                       CHASSIS_6020_3_Y_ANGLE);
      
      Chassis_Motor[6].motor_value->target_speed_rpm = Chassis_Motor[6].Motor_PID_Position.out; //位置环
      Chassis_Motor[6].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[6].Motor_PID_Speed, Chassis_Motor[6].motor_value->speed_rpm, //速度环
              +Chassis_Motor[6].motor_value->target_speed_rpm);
      
      if(cnt%2==0)
        Chassis_Motor[7].Motor_PID_Position.f_cal_pid(&Chassis_Motor[7].Motor_PID_Position, 
                                                       Chassis_Motor[7].motor_value->angle, 
                                                       CHASSIS_6020_4_Y_ANGLE);
      
      Chassis_Motor[7].motor_value->target_speed_rpm = Chassis_Motor[7].Motor_PID_Position.out; //位置环
      Chassis_Motor[7].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[7].Motor_PID_Speed, Chassis_Motor[7].motor_value->speed_rpm, //速度环
              +Chassis_Motor[7].motor_value->target_speed_rpm);
      
      /************************************将电流参数发送给电机*********************************************/
      set_gimbal_current (&hcan1,Chassis_Motor[4].Motor_PID_Speed.out,
                                 Chassis_Motor[5].Motor_PID_Speed.out,
                                 Chassis_Motor[6].Motor_PID_Speed.out,
                                 Chassis_Motor[7].Motor_PID_Speed.out);
      osDelay(1); //pid控制频率1000hz
    }
}

