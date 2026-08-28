#include "motor.h"

/*******************************
函数功能: 初始化电机方向引脚
入口参数: 无
返回值:   无
*******************************/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能PB端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_13;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//IO口速度50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//根据设定参数初始化GPIOB

	AIN=0;
	BIN=0;
}

/*******************************
函数功能: 初始化定时器PWM
入口参数: arr:自动重装载值, psc:预分频系数
返回值:   无
*******************************/
void PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	Motor_Init();											//电机方向引脚初始化

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能GPIOB时钟

	//设置PB6、PB7引脚复用输出, 作为TIM4_CH1/CH2的PWM波形输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//TIM4_CH1/TIM4_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;					//自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler = psc;				//预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//时钟分割 TDTS=Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);			//初始化TIM4时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;		//PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;						//比较值初始为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//输出极性:高
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);				//初始化TIM4通道1
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);				//初始化TIM4通道2

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);		//CH1预装载使能
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);		//CH2预装载使能
	TIM_ARRPreloadConfig(TIM4, ENABLE);						//ARR预装载使能

	TIM_CtrlPWMOutputs(TIM4, ENABLE);						//MOE主输出使能
	TIM_Cmd(TIM4, ENABLE);									//使能TIM4
}

/*******************************
函数功能: 设置左右轮PWM(速度/方向)
入口参数: moto1:右轮速度, moto2:左轮速度
          正值正转, 负值反转, 0停止
返回值:   无
*******************************/
void Set_Pwm(int moto1,int moto2)
{
	//左轮(moto1): 方向AIN(PB14/L-IB) + PWM=PWMB(TIM4_CH2/PB7/L-IA)
	if(moto1>=0)
	{
		AIN=0;						//左轮正转
		PWMB=myabs(moto1);
	}
	else
	{
		AIN=1;						//左轮反转
		PWMB=7199-myabs(moto1);
	}

	//右轮(moto2): 方向BIN(PB13/R-IA) + PWM=PWMA(TIM4_CH1/PB6/R-IB)
	if(moto2>=0)
	{
		BIN=0;						//右轮正转
		PWMA=myabs(moto2);
	}
	else
	{
		BIN=1;						//右轮反转
		PWMA=7199-myabs(moto2);
	}
}

/*******************************
函数功能: 绝对值函数
入口参数: long int a
返回值:   u32
*******************************/
u32 myabs(long int a)
{
	u32 temp;
	if(a<0)  temp=-a;
	else     temp=a;
	return temp;
}
