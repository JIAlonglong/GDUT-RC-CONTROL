#include "main.h"

 

extern vu16 ADC_DMA_IN[8]; //声明外部变量

 int main(void)
 {  
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
 	LED_Init();			     //LED端口初始化	 	

	ADC_Configuration(); //ADC初始化设置（模拟摇杆的ADC初始化）
  OLED_Init();			//初始化OLED
  OLED_Clear();
	 
  OLED_ShowString(0,0,"R2:      .   cm",16);

	 
	while(1)
	{
//   printf("a");
//	 LED0 = 1;
//			 LED1 = 1;
//			 LED2 = 1;
//			 LED3 = 1;
//	 delay_ms(100);
//	 LED0 = 0;
//			 LED1 = 0;
//			 LED2 = 0;
//			 LED3 = 0;
		oled_show();
     delay_ms(5);
	



	}
}


