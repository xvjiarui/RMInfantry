#include "main.h"
#include "function_list.h"
#include "PID_s.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "customized_function.h"
#include "external_control.h"
#include "flash.h"
#include "BSP_TIM.h"

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
	BSP_TIM_InitConfig();
}

float float_debug = 0;
int16_t int_debug = 0;
int16_t int_debug2 = 0;
s32 target_angle = 0;
s32 current_angle = 0;
s32 last_angle = 0;
int16_t M_wheel_result[4] = {0, 0, 0, 0};
PID wheels_pos_pid[4];
PID wheels_speed_pid[4];
PID angle_pid;
PID buffer_pid;
PID gimbal_speed_pid[2];
PID gimbal_pos_pid[2];
PID gimbal_reset_pid;
u8 str[256];
float init_yaw_pos;
float init_pitch_pos;
union u32ANDint16_t manual_buff_pos[18];
int16_t read_buff_pos[18];
uint8_t Chassis_Connected = 1;
uint8_t Gimbal_Connected = 1;
uint8_t DBUS_Connected = 1;
uint8_t GUN_ENCODER_Connected = 1;
int16_t chassis_ch2 = 0;
const float YAW_SPEED_TO_CHASSIS_CH2 = (float)31 / (float)220;
//following 4 variable will be init in 
int16_t last_ch_input[4];
int16_t ch_input[4];
int16_t mouse_input[2];
int16_t last_mouse_input[2];
uint8_t gimbal_follow = 1;
uint8_t chassis_already_auto_stop = 0;
uint8_t buff_mode = 0;
uint8_t gimbal_in_buff_pos = 0;
uint8_t buff_pressed =0;
uint8_t clearing_ammo = 0;
uint8_t fast_turning = 0;
const float YAW_ANGLE_RATIO = 27;
const float YAW_LEFT_BOUND_REAL_ANGLE = 65;
const float YAW_RIGHT_BOUND_REAL_ANGLE = -65;
const float YAW_LEFT_BOUND = (YAW_ANGLE_RATIO * YAW_LEFT_BOUND_REAL_ANGLE);
const float YAW_RIGHT_BOUND = (YAW_ANGLE_RATIO * YAW_RIGHT_BOUND_REAL_ANGLE);
const float PITCH_ANGLE_RATIO = 19;
const float PITCH_UPPER_BOUND_REAL_ANGLE = 45;
const float PITCH_UPPER_BOUND = (PITCH_ANGLE_RATIO * PITCH_UPPER_BOUND_REAL_ANGLE);
const float GYRO_ANGLE_RATIO = 10;

int main(void)
{	
	init();
	PID_init_all();
	input_init_all();
	init_yaw_pos = GMYawEncoder.ecd_angle;
	init_pitch_pos = GMPitchEncoder.ecd_angle + 5 * PITCH_ANGLE_RATIO;
	for (int i = 0; i < 18; ++i)
	{
		manual_buff_pos[i].flash = readFlash(i);
	}
	BSP_TIM_Start();
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			if (ticks_msimg % 20 == 0)
			{
				// Chassis_Connected = CanCheckConnection_for_Chassis();
				// Gimbal_Connected = CanCheckConnection_for_Gimbal();
				// DBUS_Connected = DBUS_CheckConnection();
				// GUN_ENCODER_Connected = ENCODER_CheckConnection();
				// GUN_EncoderUpdate();

				// if (!GUN_ENCODER_Connected)
				// {
				// 	GUN_SetFree();
				// }
				// else GUN_PokeControl();
			}

			//TODO: nothing:-)
			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				// tft_prints(0,2,"Infantry V1.3");
				if (!buff_mode)
				{
					if (DBUS_ReceiveData.rc.switch_left != 2)
					{
						/*
						tft_prints(0, 2,"r:%d", DBUS_ReceiveData.rc.switch_right);
						tft_prints(0, 3,"Pr:%f", InfantryJudge.RealVoltage * InfantryJudge.RealCurrent);
						tft_prints(0, 4,"Buffer:%f", InfantryJudge.RemainBuffer);
						tft_prints(0, 5,"ED:%d, P:%d", ENCODER_Data, DBUS_ReceiveData.mouse.press_left);
						tft_prints(0, 6,"Chassis:%d", Chassis_Connected);
						tft_prints(0, 7,"Gimbal:%d", Gimbal_Connected);
						tft_prints(0, 8,"DBUS:%d", DBUS_Connected);
						tft_prints(0, 9, "GUN %d", GUN_ENCODER_Connected);
						tft_prints(0, 10, "out:%d err:%d", GUN_Data.pokeOutput, GUN_PokeErr);
						tft_prints(0, 11, "ang %d", GUN_Data.pokeAngle);
						*/
						tft_prints(0, 8, "pid:%f", float_debug);
						tft_prints(0,9, "yaw_speed:%d", int_debug);
					}
					else
					{
						tft_prints(0, 2, "HART:%d", InfantryJudge.LastHartID);
						tft_prints(0, 3,"Buffer:%f", InfantryJudge.RemainBuffer);
						tft_prints(0, 4, "ShotSpeed:%f", InfantryJudge.LastShotSpeed);
						tft_prints(0, 5, "ShotFreq:%f", InfantryJudge.LastShotFreq);
						tft_prints(0, 6, "HP:%d", InfantryJudge.LastBlood);
						tft_prints(0, 7, "Armor:%d %f", InfantryJudge.ArmorDecrease, InfantryJudge.ArmorDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 8, "OS:%d %f", InfantryJudge.OverShootSpeedDecrease, InfantryJudge.OverShootSpeedDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 9, "OF:%d %f", InfantryJudge.OverShootFreqDecrease, InfantryJudge.OverShootFreqDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 10, "OP:%d %f", InfantryJudge.OverPowerDecrease, InfantryJudge.OverPowerDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 11, "MO:%d %f", InfantryJudge.ModuleOfflineDecrease, InfantryJudge.ModuleOfflineDecrease/(1500.0f - InfantryJudge.LastBlood));
					}

					// tft_prints(0,4,"fDebug:%f", float_debug);
					// tft_prints(0,5,"iDebug:%d", int_debug);
					
					// tft_prints(0,10,"Cur:%d E:",current_angle, DBUS_CheckPushNow(KEY_E));
					// tft_prints(0,11,"Tar:%d Q:",target_angle, DBUS_CheckPushNow(KEY_Q));
					// tft_prints(0,12,"Out:%d",output_angle);
					// tft_prints(0,4,"initp:%f", init_pitch_pos);
					// tft_prints(0,5,"targetY:%f", manual_buff_pos[0].mem);
					// tft_prints(0,6,"targetP:%f", manual_buff_pos[9].mem);
					// tft_prints(0,7,"currentY:%f", GMYawEncoder.ecd_angle - init_yaw_pos);
					// tft_prints(0,8,"currentP:%f", GMPitchEncoder.ecd_angle - init_pitch_pos);
				}
				else
				{
					for (int i = 0; i < 9; ++i)
					{
						read_buff_pos[i] = -manual_buff_pos[i].mem / YAW_ANGLE_RATIO;
					}
					for (int i = 9; i < 18; ++i)
					{
						read_buff_pos[i] = manual_buff_pos[i].mem / PITCH_ANGLE_RATIO;
					}
					// tft_prints(0, 4, "Q(%d,%d) W(%d,%d)", read_buff_pos[0], read_buff_pos[9], read_buff_pos[1], read_buff_pos[10]);
					
					tft_prints(0,2,"Buff Mode:%d", DBUS_ReceiveData.rc.switch_left);
					tft_prints(0,3,"Q:(%d,%d)", read_buff_pos[0], read_buff_pos[9]);
					tft_prints(0,4,"W:(%d,%d)", read_buff_pos[1], read_buff_pos[10]);
					tft_prints(0,5,"E:(%d,%d)", read_buff_pos[2], read_buff_pos[11]);
					tft_prints(0,6,"A:(%d,%d)", read_buff_pos[3], read_buff_pos[12]);
					tft_prints(0,7,"S:(%d,%d)", read_buff_pos[4], read_buff_pos[13]);
					tft_prints(0,8,"D:(%d,%d)", read_buff_pos[5], read_buff_pos[14]);
					tft_prints(0,9,"Z:(%d,%d)", read_buff_pos[6], read_buff_pos[15]);
					tft_prints(0,10,"X:(%d,%d)", read_buff_pos[7], read_buff_pos[16]);
					tft_prints(0,11,"C:(%d,%d)", read_buff_pos[8], read_buff_pos[17]);
					
				}
				
				tft_update();
				// LED_blink(LED1);
			}
		}
	}	
}	

	



