#include "encoder.h"

/********************************************
函数功能: 把TIM2初始化为编码器接口模式(左电机编码器 PA0/PA1)
入口参数: 无
返回值:   无
********************************************/
void Encoder_Init_TIM2(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   //使能TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;   //PA0/PA1端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //初始化GPIOA

    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;      //自动重装载值65535
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                  //预分频器, 不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分割:不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);             //初始化TIM2

    //编码器接口模式: TI1+TI2双通道计数(四倍频), 上升沿触发
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 10;     //输入滤波
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);      //清除更新标志位
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //使能更新中断
    TIM_SetCounter(TIM2, 0);                   //计数器清零
    TIM_Cmd(TIM2, ENABLE);                     //使能TIM2
}

/********************************************
函数功能: 把TIM3初始化为编码器接口模式(右电机编码器 PA6/PA7)
入口参数: 无
返回值:   无
********************************************/
void Encoder_Init_TIM3(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   //使能TIM3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;   //PA6/PA7端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //初始化GPIOA

    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;      //自动重装载值65535
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                  //预分频器, 不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分割:不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);             //初始化TIM3

    //编码器接口模式: TI1+TI2双通道计数(四倍频), 上升沿触发
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 10;     //输入滤波
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM3, TIM_FLAG_Update);      //清除更新标志位
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //使能更新中断
    TIM_SetCounter(TIM3, 0);                   //计数器清零
    TIM_Cmd(TIM3, ENABLE);                     //使能TIM3
}

/********************************************
函数功能: 单位时间读取编码器计数(并清零)
入口参数: TIMX: 定时器号(2=左轮, 3=右轮)
返回值:   速度值(两次读取之间的脉冲数)
********************************************/
static long Encoder_TIM_A_now = 0;   //左轮当前计数值
static long Encoder_TIM_B_now = 0;   //右轮当前计数值

int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;
    switch (TIMX)
    {
        case 2:
            TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
            Encoder_TIM_A_now = (short)TIM2->CNT;   //取两次之间的差值(短整型防溢出)
            Encoder_TIM = Encoder_TIM_A_now;
            TIM_SetCounter(TIM2, 0);                //清零, 以便下次读取
            break;
        case 3:
            TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
            Encoder_TIM_B_now = (short)TIM3->CNT;
            Encoder_TIM = Encoder_TIM_B_now;
            TIM_SetCounter(TIM3, 0);
            break;
        default:
            Encoder_TIM = 0;
    }
    return Encoder_TIM;
}
