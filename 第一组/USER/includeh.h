#ifndef __INCLUDEH_H
#define __INCLUDEH_H

#include "FreeRTOSConfig.h"
//系统文件
// 系统文件
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include "math.h"
// FreeRTOS系统
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"

#include "sys.h"
#include "usart.h"
#include "delay.h"

//HARDWARE文件
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


//MOODEL文件
#include "vision.h"

//action文件
#include "move.h"
#include "path.h"
#include "location.h"
#include "FSM.h"
#endif
