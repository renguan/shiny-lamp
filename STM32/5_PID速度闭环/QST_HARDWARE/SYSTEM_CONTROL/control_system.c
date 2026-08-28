#include "control_system.h"

int L_coder = 0;                 //左轮编码器实际脉冲数(100ms内)
int R_coder = 0;                 //右轮编码器实际脉冲数(100ms内)
int Motor_A = 0;                 //左轮PWM输出
int Motor_B = 0;                 //右轮PWM输出
int OverflowTime = 100;          //控制周期: 100ms
volatile uint32_t millis = 0;    //毫秒计数

/********************************************
函数功能: 增量式PI控制器(左轮)
入口参数: Encoders_A: 编码器实测脉冲数, Target_A: 目标脉冲数
返回值:   电机PWM值(可正可负, 负值为反转)
公式:     Pwm += Kp*[e(k)-e(k-1)] + Ki*e(k)
********************************************/
int Incremental_PI_A(int Encoders_A, int Target_A)
{
    float Velocity_KP = 7.0;       //比例系数
    float Velocity_KI = 0.010;     //积分系数
    static int Pwm_A = 0;
    static float Error_prev_A = 0;
    float Error_A;

    Error_A = (float)(Target_A - Encoders_A);                  //本次偏差 e(k)

    //增量式PI: 增量 = Kp*[e(k)-e(k-1)] + Ki*e(k)
    Pwm_A += (int)(Velocity_KP * (Error_A - Error_prev_A) + Velocity_KI * Error_A);
    Error_prev_A = Error_A;                                    //保存上次偏差

    if (Pwm_A > 7199)  Pwm_A = 7199;                           //PWM输出限幅
    if (Pwm_A < -7199) Pwm_A = -7199;

    return Pwm_A;
}

/********************************************
函数功能: 增量式PI控制器(右轮)
入口参数: Encoders_B: 编码器实测脉冲数, Target_B: 目标脉冲数
返回值:   电机PWM值(可正可负, 负值为反转)
********************************************/
int Incremental_PI_B(int Encoders_B, int Target_B)
{
    float Velocity_KP = 7.0;
    float Velocity_KI = 0.010;
    static int Pwm_B = 0;
    static float Error_prev_B = 0;
    float Error_B;

    Error_B = (float)(Target_B - Encoders_B);
    Pwm_B += (int)(Velocity_KP * (Error_B - Error_prev_B) + Velocity_KI * Error_B);
    Error_prev_B = Error_B;

    if (Pwm_B > 7199)  Pwm_B = 7199;
    if (Pwm_B < -7199) Pwm_B = -7199;

    return Pwm_B;
}

/********************************************
函数功能: 目标转速(转/s)换算成每100ms的编码器脉冲数
入口参数: r: 目标转速, 范围约 -1.5 ~ 1.5 (转/s)
返回值:   目标脉冲数
说明: 电机每转一圈产生(700*4)个脉冲(700线x4倍频, 需按实车实测修正)
      100ms内的目标脉冲数 = 转速 x 每转脉冲数 / (1000/OverflowTime)
********************************************/
int Rs_To_CR(float r)
{
    int CR = 0;
    CR = (int)(r * (700 * 4) / (1000 / OverflowTime));
    return CR;
}

/********************************************
函数功能: 系统控制函数: 读编码器 -> 算目标值 -> PI计算 -> 输出PWM
入口参数: 无
返回值:   无
说明: 每100ms执行一次(SysTick_Handler中调用)
********************************************/
void System_Control(void)
{
    int TageA = 0;   //左轮目标脉冲数
    int TageB = 0;   //右轮目标脉冲数

    L_coder = -Read_Encoder(2);  //读取左轮100ms内脉冲数(取负: 前进为正)
    R_coder = -Read_Encoder(3);  //读取右轮100ms内脉冲数(取负: 前进为正)

    TageA = Rs_To_CR(1.0);       //左轮目标: 1转/s 前进
    TageB = Rs_To_CR(1.0);       //右轮目标: 1转/s 前进 (与左轮同方向)

    Motor_A = Incremental_PI_A(L_coder, TageA);   //左轮速度闭环 -> PWM
    Motor_B = Incremental_PI_B(R_coder, TageB);   //右轮速度闭环 -> PWM

    printf("L_coder:%d R_coder:%d\r\n", L_coder, R_coder);
    printf("TageA:%d TageB:%d\r\n", TageA, TageB);
    printf("Motor_A:%d Motor_B:%d\r\n", Motor_A, Motor_B);

    Set_Pwm(Motor_A, Motor_B);   //输出PWM驱动电机
}

/********************************************
函数功能: 系统滴答定时器中断服务函数(SysTick每1ms进一次)
说明: 每100ms执行一次闭环控制
********************************************/
void SysTick_Handler(void)
{
    millis++;                       //毫秒数加1
    if (millis % OverflowTime == 0) //每100ms
    {
        millis = 0;
        System_Control();           //执行闭环控制
    }
}
