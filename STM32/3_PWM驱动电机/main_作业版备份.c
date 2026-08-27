#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"

int main(void)
{
	RCC->CR |= 1<<24;					//使能外部高速晶振HSE
	Stm32_Clock_Init(9);				//外部时钟8MHz 9倍频 -> 72MHz
	MY_NVIC_PriorityGroupConfig(2);		//中断优先级分组
	uart_init(115200);					//串口初始化为115200
	JTAG_Set(JTAG_SWD_DISABLE);			//关闭JTAG接口
	JTAG_Set(SWD_ENABLE);				//打开SWD接口, 可以利用主板的SWD接口调试

	PWM_Init(7199,9);					//定时器PWM初始化, 输出频率1000Hz
	colorful_led_Init();				//炫彩灯初始化

	printf("QS智联小车\r\n");
	/*主程序*/
	while(1)
	{
		Set_Pwm(2500,2500);				//设置左右轮速度(占空比2500/7199)
		delay_ms(100);
	}
}
