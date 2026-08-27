#include "stm32f10x.h"
#include "sys.h"
#include "encoder.h"

int L_speed = 0;                 //左轮编码器脉冲数(每100ms)
int R_speed = 0;                 //右轮编码器脉冲数(每100ms)
int OverflowTime = 100;          //读取周期: 100ms
volatile uint32_t millis = 0;    //毫秒计数

/* ===== 速度换算参数(学生需手动实测后修改!) ===== */
#define ENCODER_PULSES_PER_REV  (360*4)    //电机转一圈的编码器总脉冲数 = 编码器线数x4倍频, 教材默认360线, 以实测为准
#define WHEEL_CIRCUMFERENCE     (0.204f)   //车轮周长(m) = pi x 车轮直径, 默认按直径65mm, 以实测为准

/********************************************
函数功能: 系统控制函数, 每100ms读取一次编码器, 换算成速度(m/s)打印
入口参数: 无
返回值:   无
********************************************/
void System_Control(void)
{
    float L_v, R_v;

    L_speed = Read_Encoder(2);   //读取左轮100ms内脉冲数
    R_speed = Read_Encoder(3);   //读取右轮100ms内脉冲数

    //速度(m/s) = 脉冲数/每圈脉冲数 x 车轮周长 / 时间间隔(s)
    L_v = (float)L_speed / ENCODER_PULSES_PER_REV * WHEEL_CIRCUMFERENCE / (OverflowTime / 1000.0f);
    R_v = (float)R_speed / ENCODER_PULSES_PER_REV * WHEEL_CIRCUMFERENCE / (OverflowTime / 1000.0f);

    printf("left  speed : %d, %.3f m/s\r\n", L_speed, L_v);
    printf("right speed : %d, %.3f m/s\r\n", R_speed, R_v);
}

/********************************************
函数功能: 系统滴答定时器中断服务函数(SysTick每1ms进一次)
入口参数: 无
返回值:   无
********************************************/
void SysTick_Handler(void)
{
    millis++;                            //毫秒数加1
    if (millis % OverflowTime == 0)       //每100ms
    {
        millis = 0;
        System_Control();                //读取编码器速度并打印
    }
}

int main(void)
{
    Stm32_Clock_Init(9);                 //外部时钟8MHz 9倍频 -> 72MHz
    MY_NVIC_PriorityGroupConfig(2);      //中断优先级分组
    uart_init(115200);                   //串口初始化为115200
    JTAG_Set(JTAG_SWD_DISABLE);          //关闭JTAG接口
    JTAG_Set(SWD_ENABLE);                //打开SWD接口, 可以利用主板的SWD接口调试

    Encoder_Init_TIM2();                 //初始化左电机编码器
    Encoder_Init_TIM3();                 //初始化右电机编码器
    colorful_led_Init();                 //炫彩灯初始化

    SysTick_Config(72000000/1000);       //滴答定时器, 每1ms触发一次中断

    printf("QST青软\r\n");
    /*主程序*/
    while (1)
    {
        delay_ms(100);
    }
}
