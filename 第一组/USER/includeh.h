#ifndef __INCLUDEH_H
#define __INCLUDEH_H

#include "FreeRTOSConfig.h"
//ϵͳ�ļ�
// ϵͳ�ļ�
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include "math.h"
// FreeRTOSϵͳ
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"

#include "sys.h"
#include "usart.h"
#include "delay.h"

//HARDWARE�ļ�
#include "pid.h"
#include "can.h"
#include "MOTO.h"
#include "can2.h"
#include "define.h"
#include "calculate_driver.h"
#include "global_location.h"
#include "task_cfg.h"
#include "air.h"
#include "ACTION.h"
#include "MIT.h"

#include "Cylinder.h"


//MOODEL�ļ�
#include "vision.h"

//action�ļ�
#include "move.h"
#include "path.h"
#include "location.h"
#include "FSM.h"
#endif
