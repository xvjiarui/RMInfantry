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

float debug = 0;
s32 target_angle = 0;
s32 current_angle = 0;
int16_t M_wheel_result[4] = {0, 0, 0, 0};
PID wheels_pos_pid[4];
PID wheels_speed_pid[4];
PID wheels_speed_semi_closed_pid[4];
PID angle_pid;
PID buffer_pid;
PID gimbal_speed_pid[2];
PID gimbal_pos_pid[2];
PID gimbal_reset_pid;
PID gimbal_relative_angle_pid;
u8 str[256];
float buffer_remain;
float init_yaw_pos;
float init_pitch_pos;
int16_t buff_yaw_pos[3] = {-15, 0, 15};
int16_t buff_pitch_pos[3] = {18, 10, 3};
union u32ANDint16_t manual_buff_pos[18];
uint8_t Chassis_Connected = 1;
uint8_t Gimbal_Connected = 1;
uint8_t DBUS_Connected = 1;

int main(void)
{	
	init();
	PID_init_all();
	buffer_remain = 60;
	init_yaw_pos = GMYawEncoder.ecd_angle;
	init_pitch_pos = GMPitchEncoder.ecd_angle + 14 * 19;
	for (int i = 0; i < 18; ++i)
	{
		manual_buff_pos[i].flash = readFlash(i);
	}
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			if (ticks_msimg % 20 == 0)
			{
				GUN_PokeControl();
				Chassis_Connected = CanCheckConnection_for_Chassis();
				Gimbal_Connected = CanCheckConnection_for_Gimbal();
				DBUS_Connected = DBUS_CheckConnection();
			}

			//TODO: nothing:-)
			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				tft_prints(0,2,"Infantry V1.3");
				tft_prints(0,4,"Debug:%f", GMYawEncoder.ecd_angle - init_yaw_pos);
				//tft_prints(0,5,"Debug:%f", (YAW_RIGHT_BOUND - (GMYawEncoder.ecd_angle - init_yaw_pos))/YAW_RIGHT_BOUND);
				tft_prints(0,4,"Debug:%f", (YAW_RIGHT_BOUND - (GMYawEncoder.ecd_angle - init_yaw_pos))/YAW_RIGHT_BOUND);
				tft_prints(0,5,"Debug:%f", YAW_RIGHT_BOUND/YAW_RIGHT_BOUND);
				
				// tft_prints(0,6,"X_s:%d", DBUS_ReceiveData.mouse.x);
				// tft_prints(0,7,"Y_s:%d", DBUS_ReceiveData.mouse.y);
				// tft_prints(0,8,"X_p:%d", DBUS_ReceiveData.mouse.x_position);
				// tft_prints(0,9,"Y_p:%d", DBUS_ReceiveData.mouse.y_position);
				tft_prints(0, 6,"Chassis:%d", Chassis_Connected);
				tft_prints(0, 7,"Gimbal:%d", Gimbal_Connected);
				tft_prints(0, 8,"DBUS:%d", DBUS_Connected);
				tft_prints(0,10,"Cur:%d",current_angle);
				tft_prints(0,11,"Tar:%d",target_angle);
				tft_prints(0,12,"Out:%d",output_angle);
				// tft_prints(0,4,"Q:%d W:%d E:%d", manual_buff_pos[0].mem, manual_buff_pos[1].mem, manual_buff_pos[2].mem);
				// tft_prints(0,5,"Q:%d W:%d E:%d", manual_buff_pos[9].mem, manual_buff_pos[10].mem, manual_buff_pos[11].mem);
				// tft_prints(0,6,"A:%d S:%d D:%d", manual_buff_pos[3].mem, manual_buff_pos[4].mem, manual_buff_pos[5].mem);
				// tft_prints(0,7,"A:%d S:%d D:%d", manual_buff_pos[12].mem, manual_buff_pos[13].mem, manual_buff_pos[14].mem);
				// tft_prints(0,8,"Z:%d X:%d C:%d", manual_buff_pos[6].mem, manual_buff_pos[7].mem, manual_buff_pos[8].mem);
				// tft_prints(0,9,"Z:%d X:%d C:%d", manual_buff_pos[15].mem, manual_buff_pos[16].mem, manual_buff_pos[17].mem);

				
				tft_update();
				LED_blink(LED1);
			}
			external_control();
		}
	}	
}	

	



