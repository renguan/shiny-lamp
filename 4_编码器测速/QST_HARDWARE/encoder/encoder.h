#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"

#define ENCODER_TIM_PERIOD 65535   //计数器重装载值(编码器脉冲计数最大值)

void Encoder_Init_TIM2(void);   //左电机编码器初始化(PA0/PA1 -> TIM2_CH1/CH2)
void Encoder_Init_TIM3(void);   //右电机编码器初始化(PA6/PA7 -> TIM3_CH1/CH2)
int Read_Encoder(u8 TIMX);      //读取编码器脉冲数并清零, 返回两次读取之间的脉冲差值

#endif
