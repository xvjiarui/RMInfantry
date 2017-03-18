#include "main.h"
#include "function_list.h"
#include "PID_s.h"
#include "power_control.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "customized_function.h"
#include "external_control.h"
#include "flash.h"

volatile u32 ticks_msimg = (u32)-1;

void init(){
	SysTick_Init();  
	Dbus_init();//usart1
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
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
	ENCODER_Init();
	GUN_Init();
}

s32 debug = 0;
s32 target_angle = 0;
s32 current_angle = 0;
s32 last_angle = 0;
int16_t result[4] = {0, 0, 0, 0};
PID wheels_pos_pid[4];
PID wheels_speed_pid[4];
PID angle_pid;
PID buffer_pid;
PID gimbal_speed_pid[2];
PID gimbal_pos_pid[2];
PID gimbal_reset_pid;
u8 str[256];
float buffer_remain;
float init_yaw_pos;
float init_pitch_pos;
int16_t buff_yaw_pos[3] = {-15, 0, 15};
int16_t buff_pitch_pos[3] = {18, 10, 3};
union u32ANDint16_t manual_buff_yaw_pos[3][3];
union u32ANDint16_t manual_buff_pitch_pos[3][3];
int main(void)
{	
	init();
	for (int i = 0; i < 4; i++) {
			//PID_init(&wheels_pos_pid[i],130, 0.004, 0, 5000);//30 0.001 5 65 0.001 5
			PID_init(&wheels_pos_pid[i], 0.15, 0, 0, 20000);
			PID_init(&wheels_speed_pid[i], 80, 5, 100, 20000); //0.00001
	}
	PID_init(&gimbal_speed_pid[0], 80, 5, 100, 20000);
	PID_init(&gimbal_speed_pid[1], 80, 5, 100, 20000);
	PID_init(&gimbal_pos_pid[0], 0.15, 0, 0, 20000);
	PID_init(&gimbal_pos_pid[1], 0.35, 0, 0, 20000);
	PID_init(&angle_pid, 4, 0, 0, 660); //5 20
	PID_init(&buffer_pid, 0.02, 0, 0, 60);
	PID_init(&gimbal_reset_pid, 0.02, 0, 0, 50);
	buffer_remain = 60;
	init_yaw_pos = GMYawEncoder.ecd_angle;
	init_pitch_pos = GMPitchEncoder.ecd_angle + 14 * 19;
	for (u8 i = 0; i < 3; ++i)
	{
		for (u8 j = 0; j < 3; ++j)
		{
			manual_buff_yaw_pos[i][j].flash = readFlash(3 * i + j);
			manual_buff_pitch_pos[i][j].flash = readFlash(3 * i + j + 9);
		}
	}
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			if (ticks_msimg % 20 == 0)
				GUN_PokeControl();

			//TODO: nothing:-)
			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				tft_prints(0,2,"Infantry V1.3");
				tft_prints(0,3,"%d",ticks_msimg);
				tft_prints(0,4,"Q:%d W:%d E:%d", manual_buff_yaw_pos[0][0].mem, manual_buff_yaw_pos[0][1].mem, manual_buff_yaw_pos[0][2].mem);
				tft_prints(0,5,"Q:%d W:%d E:%d", manual_buff_pitch_pos[0][0].mem, manual_buff_pitch_pos[0][1].mem, manual_buff_pitch_pos[0][2].mem);
				tft_prints(0,6,"A:%d S:%d D:%d", manual_buff_yaw_pos[1][0].mem, manual_buff_yaw_pos[1][1].mem, manual_buff_yaw_pos[1][2].mem);
				tft_prints(0,7,"A:%d S:%d D:%d", manual_buff_pitch_pos[1][0].mem, manual_buff_pitch_pos[1][1].mem, manual_buff_pitch_pos[1][2].mem);
				tft_prints(0,8,"Z:%d X:%d C:%d", manual_buff_yaw_pos[2][0].mem, manual_buff_yaw_pos[2][1].mem, manual_buff_yaw_pos[2][2].mem);
				tft_prints(0,9,"Z:%d X:%d C:%d", manual_buff_pitch_pos[2][0].mem, manual_buff_pitch_pos[2][1].mem, manual_buff_pitch_pos[2][2].mem);
				
				tft_update();
				LED_blink(LED1);
			}			
			external_control();
		}
	}	
}	

	



