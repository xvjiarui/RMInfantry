#include "main.h"
#include "function_list.h"
#include "PID_s.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "customized_function.h"
#include "external_control.h"
#include "BSP_TIM.h"
#include "const.h"
#include "Driver_Manifold.h"
volatile u32 ticks_msimg = (u32)-1;

void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	tft_init(2,BLACK,GREEN,GREEN);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	Judge_InitConfig();
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
	GUN_Init();
	BSP_TIM_InitConfig();
	PID_init_all();
	input_init_all();
	chassis_control_init();
	gimbal_control_init();
	buzzer_init();
	external_control_init();
	Driver_Manifold_init();
	LED_control(LASER, 1);
}

int16_t int_debug;
int16_t int_debug2;
float float_debug;
float float_debug2;



int main(void)
{	
	init();
	BSP_TIM_Start();
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			if (ticks_msimg % 20 == 0)
			{
				
			}

			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				if (!buff_mode)
				{
					if (DBUS_ReceiveData.rc.switch_left != 2)
					{
						
						tft_prints(0, 2,"r:%d", DBUS_ReceiveData.rc.switch_right);
						tft_prints(0, 3,"Pr:%.2f, Bf:%.2f", InfantryJudge.RealVoltage * InfantryJudge.RealCurrent, InfantryJudge.RemainBuffer);
						tft_prints(0, 4,"DBUS:%d Judge:%d", DBUS_Connected, Judge_Connected);
						tft_prints(0, 5,"L:%d R:%d U:%d E:%d", DBUS_ReceiveData.mouse.press_left, DBUS_ReceiveData.mouse.press_right, GUN_Data.usrShot, GUN_Data.emptyCount);
						tft_prints(0, 6,"Chassis:%d %d %d %d %d", Chassis_Connected, can_chassis_connected[0], can_chassis_connected[1], can_chassis_connected[2], can_chassis_connected[3]);
						tft_prints(0, 7,"Gimbal:%d %d %d %d", Gimbal_Connected, can_gimbal_connected[0], can_gimbal_connected[1], can_gimbal_connected[2]);
						tft_prints(0, 8, "d:%d s:%d i:%d", GUN_Direction, GUN_Data.stucked, GUN_DriverInput);
						tft_prints(0, 9, "IT:%d %f", InfantryJudge.LastShotTick, float_debug);
						// tft_prints(0, 10, "GT:%d %d", GUN_Data.last_poke_tick, ticks_msimg);
						tft_prints(0, 10, "%.1f %.1f", -GMYawEncoder.ecd_angle/YAW_ANGLE_RATIO, (GMPitchEncoder.ecd_angle - PITCH_HORIZONTAL_OFFSET)/PITCH_ANGLE_RATIO);
						tft_prints(0, 11, "%.1f %.1f %d", rune_angle_x, rune_angle_y, rune_index);
						// tft_prints(0, 9, "C:%f", GMxEncoder.ecd_angle);
						// tft_prints(0, 10, "T:%f", GUN_TargetPos);
						// tft_prints(0, 10,"C:%d T:%d", current_angle, target_angle);
						// tft_prints(0, 11, "f_d:%f", float_debug);
						// tft_prints(0, 11, "i_d:%d", int_debug);
					}
					else
					{
						tft_prints(0, 2, "HART:%d", InfantryJudge.LastHartID);
						tft_prints(0, 3, "ShotSpeed:%f", InfantryJudge.LastShotSpeed);
						tft_prints(0, 4, "ShotFreq:%f", InfantryJudge.LastShotFreq);
						tft_prints(0, 5, "HP:%d SN:%d", InfantryJudge.LastBlood, InfantryJudge.ShootNum);
						tft_prints(0, 6, "Armor:%d %f", InfantryJudge.ArmorDecrease, InfantryJudge.ArmorDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 7, "Crash:%d %f", InfantryJudge.CrashDecrease,  InfantryJudge.CrashDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 8, "OS:%d %f", InfantryJudge.OverShootSpeedDecrease, InfantryJudge.OverShootSpeedDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 9, "OF:%d %f", InfantryJudge.OverShootFreqDecrease, InfantryJudge.OverShootFreqDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 10, "OP:%d %f", InfantryJudge.OverPowerDecrease, InfantryJudge.OverPowerDecrease/(1500.0f - InfantryJudge.LastBlood));
						tft_prints(0, 11, "MO:%d %f", InfantryJudge.ModuleOfflineDecrease, InfantryJudge.ModuleOfflineDecrease/(1500.0f - InfantryJudge.LastBlood));
					}
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
					
					tft_prints(0,2,"Buff Mode:%d %f", DBUS_ReceiveData.rc.switch_left, InfantryJudge.RealVoltage);
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
			}
		}
	}	
}	

	



