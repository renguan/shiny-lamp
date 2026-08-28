#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "encoder.h"
#include "control_system.h"

int main(void)
{
    Stm32_Clock_Init(9);                 //外部时钟8MHz 9倍频 -> 72MHz
    MY_NVIC_PriorityGroupConfig(2);      //中断优先级分组
    uart_init(115200);                   //串口初始化为115200
    JTAG_Set(JTAG_SWD_DISABLE);          //关闭JTAG接口
    JTAG_Set(SWD_ENABLE);                //打开SWD接口, 可以利用主板的SWD接口调试

    Encoder_Init_TIM2();                 //初始化左电机编码器
    Encoder_Init_TIM3();                 //初始化右电机编码器
    PWM_Init(7199,9);                    //定时器PWM初始化, 频率1000Hz
    colorful_led_Init();                 //炫彩灯初始化

    SysTick_Config(72000000/1000);       //滴答定时器, 每1ms触发一次中断

    printf("QST青软\r\n");
    /*主程序*/
    while (1)
    {
        delay_ms(100);
    }
}
