#pragma once
#include "vex.h"
// typedef struct _CascadePID
// {
// 	PID inner;    // 内环速度环PID
// 	PID outer;    // 外环角度环PID
// 	float output;
// }CascadePID;

// typedef struct _Motor
// {
// 	float lastAngle;       //上次计数结束时转过的角度
// 	float totalAngle;      //总共转过的角度
// 	float loopNum;         //电机计数过零计数
// 	float speed;             //电机输出轴速度
// 	float targetSpeed;       //添加设定的目标速度
//     CascadePID anglePID;     //串级PID 
// }Motor;

// //串级PID计算，参数为串级PID指针，角度目标值，角度返回值，速度返回值
// void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
float get_arm_angle();