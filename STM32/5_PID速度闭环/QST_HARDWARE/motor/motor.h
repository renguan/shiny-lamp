#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

//电机方向控制引脚(位带操作)
#define AIN  PBout(14)    //左电机方向控制引脚
#define BIN  PBout(13)    //右电机方向控制引脚

//电机PWM输出(TIM4_CH1->PB6, TIM4_CH2->PB7)
#define PWMA TIM4->CCR1   //左电机PWM
#define PWMB TIM4->CCR2   //右电机PWM

void Motor_Init(void);             //电机方向引脚初始化
void PWM_Init(u16 arr,u16 psc);    //定时器PWM初始化
void Set_Pwm(int moto1,int moto2); //设置左右轮PWM
u32 myabs(long int a);             //绝对值函数

#endif
