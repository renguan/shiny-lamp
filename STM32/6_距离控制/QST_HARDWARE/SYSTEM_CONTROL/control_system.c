#include "control_system.h"

int L_coder = 0;                 //左轮编码器实际脉冲数(100ms内, 前进为正)
int R_coder = 0;                 //右轮编码器实际脉冲数(100ms内, 前进为正)
int Motor_A = 0;                 //左轮PWM输出
int Motor_B = 0;                 //右轮PWM输出
int OverflowTime = 100;          //控制周期: 100ms
volatile uint32_t millis = 0;    //毫秒计数

/* ===== 距离控制参数(已实测标定) ===== */
#define DISTANCE_COUNTS_PER_METER  20200   //每米脉冲数: 轮径4.4cm, 周长0.1382m, (700*4)/0.1382≈20256, 实测折中取20200
#define DISTANCE_METERS            1       //前进/后退距离(米)
#define FORWARD_RPS   1.5f                //前进目标转速(转/s), 越大越快(参考1.0~2.0)
#define BACKWARD_RPS  -1.7f               //后退目标转速(转/s), 略大于前进, 补偿后退偏慢(按实际微调)

long distance_sum = 0;           //累计脉冲数(前进为正)
int Pwm_A = 0;                   //左轮PI累加器(全局, 供状态切换时清零)
int Pwm_B = 0;                   //右轮PI累加器
float Error_prev_A = 0;          //左轮上次偏差
float Error_prev_B = 0;          //右轮上次偏差
u8 run_state = 0;                //0=前进, 1=停止, 2=后退, 3=到达终点
u32 state_timer = 0;             //状态计时(ms)

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
说明: 电机每转一圈产生(700*4)个脉冲(700线x4倍频, 按实车修正)
      100ms内的目标脉冲数 = 转速 x 每转脉冲数 / (1000/OverflowTime)
********************************************/
int Rs_To_CR(float r)
{
    int CR = 0;
    CR = (int)(r * (700 * 4) / (1000 / OverflowTime));
    return CR;
}

/********************************************
函数功能: 系统控制函数: 距离状态机 + 速度闭环, 每100ms执行一次
流程: 前进1米 -> 停0.5秒 -> 后退1米 -> 停(回到起点)
********************************************/
void System_Control(void)
{
    int TageA = 0;
    int TageB = 0;

    L_coder = Read_Encoder(2);    //读取左轮100ms内脉冲数(实车标定: 前进为正)
    R_coder = Read_Encoder(3);    //读取右轮100ms内脉冲数(实车标定: 前进为正)

    /* 距离状态机 */
    switch (run_state)
    {
        case 0:   //前进: 累计脉冲, 达到1米后停止
            distance_sum += L_coder;
            if (distance_sum >= DISTANCE_COUNTS_PER_METER * DISTANCE_METERS)
            {
                run_state = 1;
                state_timer = millis;
                printf("到达1米, 停止\r\n");
            }
            break;

        case 1:   //停止0.5秒后后退
            if (millis - state_timer >= 500)
            {
                run_state = 2;
                distance_sum = 0;
                Pwm_A = 0;            //清零PI累加器, 后退立即反转
                Pwm_B = 0;
                Error_prev_A = 0;
                Error_prev_B = 0;
                printf("开始后退\r\n");
            }
            break;

        case 2:   //后退: 累计脉冲(反向为正), 达到1米后停止
            distance_sum += (-L_coder);
            if (distance_sum >= DISTANCE_COUNTS_PER_METER * DISTANCE_METERS)
            {
                run_state = 3;
                state_timer = millis;
                printf("回到起点!\r\n");
            }
            break;

        case 3:   //结束: 保持停止
            break;
    }

    /* 目标速度: 前进+1转/s, 后退-1转/s, 停止0 */
    if (run_state == 0)      { TageA = Rs_To_CR(FORWARD_RPS);  TageB = Rs_To_CR(FORWARD_RPS);  }
    else if (run_state == 2) { TageA = Rs_To_CR(BACKWARD_RPS); TageB = Rs_To_CR(BACKWARD_RPS); }
    else                     { TageA = 0;              TageB = 0;              }

    Motor_A = Incremental_PI_A(L_coder, TageA);   //左轮速度闭环 -> PWM
    Motor_B = Incremental_PI_B(R_coder, TageB);   //右轮速度闭环 -> PWM

    if (run_state == 1 || run_state == 3)  Set_Pwm(0, 0);         //停止状态强制停电机
    else                                   Set_Pwm(Motor_A, Motor_B);

    printf("L:%d R:%d sum:%ld state:%d\r\n", L_coder, R_coder, distance_sum, run_state);
}

/********************************************
函数功能: 系统滴答定时器中断服务函数(SysTick每1ms进一次), 每100ms执行一次闭环
********************************************/
void SysTick_Handler(void)
{
    millis++;                       //毫秒数加1(持续累计, 不清零, 供状态计时使用)
    if (millis % OverflowTime == 0) //每100ms
    {
        System_Control();           //执行闭环控制
    }
}
