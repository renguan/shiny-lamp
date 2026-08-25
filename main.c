#include "stm32f10x.h"
#include "sys.h"
#include "colorful_led.h"

int main(void)
  { 
		Stm32_Clock_Init(9);						//�ⲿʱ��8Mhz 9��Ƶ  8*9= 72mhz��Ƶ72mhz
		MY_NVIC_PriorityGroupConfig(2);	//=====�ж����ȼ�����		
		uart_init(115200);	            //=====���ڳ�ʼ��Ϊ
		JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
		JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���

		colorful_led_Init();            //=====�ŲʵƳ�ʼ��
		//SysTick_Config(72000000/1000);		//�δ�ʱ����ÿ1ms����һ���ж�
    
		L_runingled();                  //ǰ����Ч
		/**��Ҫ����**/
	while(1)
	{
		
	}
}
	

