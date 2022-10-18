#ifndef __DEFINES_H
#define __DEFINES_H

#define PI 								 3.14159265358979f
#define COS45              0.70710678f
#define SIN45              0.70710678f
#define ABS(x)      ((x)>0? (x):(-(x)))

// Chassis Config
#define WHEEL_R            0.076f	                  //轮子半径(单位：m) 
#define Robot_R            0.456f                  	//车轮到中心距离(单位：m)
#define M3508_RM_To_MS     (PI*WHEEL_R)/570.0f      //转速与速度的转换 (单位：m/s) 
#define M3508_MS_To_RM    1/(PI*WHEEL_R)      //速度与转速的转换 (单位：m/s)  
#define MS_TO_RPM          21*60/(PI*WHEEL_R*2)     //轮子直径152mm，电机减速比1:21，轮子一圈pi*152mm\

#endif
