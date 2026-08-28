#ifndef __CONTROL_SYSTEM_H
#define __CONTROL_SYSTEM_H
#include "sys.h"
#include "motor.h"
#include "encoder.h"

int Incremental_PI_A(int Encoders_A, int Target_A);   //左轮增量式PI控制器
int Incremental_PI_B(int Encoders_B, int Target_B);   //右轮增量式PI控制器
int Rs_To_CR(float r);                                //目标转速(转/s)换算成每100ms脉冲数
void System_Control(void);                            //闭环控制函数(每100ms调用一次)

#endif
