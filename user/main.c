#include "main.h"
#include "function_list.h"

static u32 ticks_msimg = (u32)-1;

void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,BLACK,GREEN,GREEN);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	Quad_Encoder_Configuration();
	Encoder_Start1();
	Encoder_Start2();
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
}

int main(void)
{	
//	u8 dum[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '\n'};
	init();

	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			//buzzer_check();			
			
			//if (ticks_msimg%500==0)
				//DataMonitor_Send(dum, sizeof(dum));
			
			//TODO: nothing:-)
			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				tft_prints(0,2,"Infantry V1.2");
				
				
				tft_update();
				LED_blink(LED1);
			}			
			
		}
	}	
}	

	



