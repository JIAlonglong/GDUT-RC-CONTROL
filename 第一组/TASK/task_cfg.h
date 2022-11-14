#ifndef __TASKS_H
#define __TASKS_H
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t ChassisDrive_Handler;

//任务的优先级设置
#define CHASSIS_TASK_PRIO		8
#define MotorVelocityCurve_TASK_PRIO  9
#define MOVE_TASK_PRIO      7
#define SHOOTING_TASK_PRIO  10

//栈的大小设置
#define CHASSIS_STK_SIZE 		256 
#define MotorVelocityCurve_STK_SIZE  128
#define MOVE_STK_SIZE      256
#define SHOOTING_STK_SIZE  256

void ChassisDrive_task(void *pvParameters);
void MotorVelocityCurve_task(void);
void Move_task(void *pvParameters);
void shooting_FSM_task(void *pvParameters);
#endif
