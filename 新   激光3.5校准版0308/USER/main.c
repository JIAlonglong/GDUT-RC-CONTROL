#include "main.h"

 

extern vu16 ADC_DMA_IN[8]; //�����ⲿ����

 int main(void)
 {  
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
 	LED_Init();			     //LED�˿ڳ�ʼ��	 	

	ADC_Configuration(); //ADC��ʼ�����ã�ģ��ҡ�˵�ADC��ʼ����
  OLED_Init();			//��ʼ��OLED
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


