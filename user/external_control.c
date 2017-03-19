#include "function_list.h"
#include "external_control.h"
#include "PID_s.h"
#include "power_control.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "customized_function.h"
#include "global_variable.h"
#include "flash.h"
#include "buzzer_song.h"

void external_control() {
	if (DBUS_ReceiveData.rc.switch_right == 1)
	{
		//emergency stop
		control_car(0, 0, 0); //warning: there is some emergency code inside this function
		control_gimbal_yaw_pos(0);
	}
	else if (DBUS_ReceiveData.rc.switch_right == 3)
	{
		if (DBUS_ReceiveData.rc.switch_left == 1)
		{
			remote_control();
		}
		else remote_buff_adjust();
	}
	else {
		computer_control();
	}
}

void remote_control() {
	is_writing_flash = 0;
	static int16_t last_ch_input[4];
	static int16_t ch_input[4];
	int16_t ch_changes[4] = {0, 0, 0, 0};
	ch_changes[0] = DBUS_ReceiveData.rc.ch0 - last_ch_input[0];
	ch_changes[1] = DBUS_ReceiveData.rc.ch1 - last_ch_input[1];
	ch_changes[2] = -DBUS_ReceiveData.rc.ch2/3 - last_ch_input[2];
	ch_changes[3] = DBUS_ReceiveData.rc.ch3 - last_ch_input[3];
	int16_t max_change = 2;
	int16_t min_change = -2;
	for (int i = 0; i < 4; ++i)
	{
		limit_int_range(&ch_changes[i], max_change, min_change);
		ch_input[i] += ch_changes[i];
		last_ch_input[i] = ch_input[i];
	}
	// ch_input[2] = DBUS_ReceiveData.rc.ch2;
	// control_car(ch_input[0], ch_input[1], ch_input[2]);
	if ((ch_input[2] > 0 && gimbal_exceed_right_bound()) || (ch_input[2] < 0 && gimbal_exceed_left_bound()))
	{
		ch_input[2] = 0;
		last_ch_input[2] = 0;
	}
	if ( ch_input[3] > 19 * 45 && gimbal_exceed_upper_bound() )
	{
		ch_input[3] = 19 * 45;
		last_ch_input[3] = 19 * 45;
	}
	if (ch_input[3] < 0 && gimbal_exceed_lower_bound() )
	{
		ch_input[3] = 0;
		last_ch_input[3] = 0;
	}
	if ( float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 10) != 1) {
		chassis_follow_with_control(ch_input[2], ch_input[3]);
	}
	else {
		control_gimbal(ch_input[2], ch_input[3]);
	}
	control_car(ch_input[0], ch_input[1], 0);
}

void computer_control() {
	static int16_t last_ch_input[4];
	static int16_t ch_input[4];
	int16_t ch_changes[4] = {0, 0, 0, 0};
	int16_t mouse_changes[2] = {0, 0};
	static int16_t mouse_input[2];
	static int16_t last_mouse_input[2];
	static int16_t in_following_flag; //a flag that help
	int16_t buff_mode = 0;
	//keyboard part
	float ratio = 0.75;
	ratio += 0.25 * (DBUS_CheckPush(KEY_SHIFT) - DBUS_CheckPush(KEY_CTRL));
	ch_changes[0] = (DBUS_CheckPush(KEY_D) - DBUS_CheckPush(KEY_A)) * 660 * ratio - last_ch_input[0];
	ch_changes[1] = (DBUS_CheckPush(KEY_W) - DBUS_CheckPush(KEY_S)) * 660 * ratio - last_ch_input[1];
	////////////////////////////
	/////////////May be ratio in this line can be deleted
	ch_changes[2] = (DBUS_CheckPush(KEY_E) - DBUS_CheckPush(KEY_Q)) * 660 * ratio - last_ch_input[2];
	int16_t max_change = 2;
	int16_t min_change = -2;
	for (int i = 0; i < 3; ++i)
	{
		limit_int_range(&ch_changes[i], max_change, min_change);
		ch_input[i] += ch_changes[i];
		last_ch_input[i] = ch_input[i];
	}
	//mouse part
	mouse_changes[0] = - 3 * DBUS_ReceiveData.mouse.x - last_mouse_input[0];
	mouse_changes[1] = - 2 * DBUS_ReceiveData.mouse.y_position - last_mouse_input[1];
	int16_t max_mouse_change = 4;
	int16_t min_mouse_change = -4;
	// limit_int_range(&mouse_changes[0], max_mouse_change, min_mouse_change);
	mouse_input[0] += mouse_changes[0];
	last_mouse_input[0] = mouse_input[0];
	mouse_input[1] += mouse_changes[1];
	last_mouse_input[1] = mouse_input[1];
	if ((mouse_input[0] > 0 && gimbal_exceed_right_bound()) || (mouse_input[0] < 0 && gimbal_exceed_left_bound()))
	{
		mouse_input[0] = 0;
		last_mouse_input[0] = 0;
	}
	if ( mouse_input[1] > 19 * 45 && gimbal_exceed_upper_bound() )
	{
		mouse_input[1] = 19 * 45;
		last_mouse_input[1] = 19 * 45;
		DBUS_ReceiveData.mouse.y_position = -mouse_input[1]/2;
	}
	if (mouse_input[1] < 0 && gimbal_exceed_lower_bound() )
	{
		mouse_input[1] = 0;
		last_mouse_input[1] = 0;
		DBUS_ReceiveData.mouse.y_position = -mouse_input[1]/2;
	}

	//shoot check
	if (DBUS_ReceiveData.mouse.press_left)
	{
		//shoot
	}
	else if (DBUS_ReceiveData.mouse.press_right)
	{
		//shooooooooooooooooooooooooooot
	}
	buff_mode = (DBUS_CheckPush(KEY_V));
	if (buff_mode)
	{
		control_car(0, 0, 0);
		buff_switch();
	}
	else if (DBUS_CheckPush(KEY_C))
	{
		chassis_gimbal_relative_angle_with_control(30, mouse_input[0], mouse_input[1]);
		control_car(ch_input[0], ch_input[1], ch_input[2]);
	}
	else 
	{
		if (DBUS_ReceiveData.rc.switch_left == 1) { //left switch up
			//following mode
			//if there is angle difference between the chassis and gimbal, chassis will follow it
			if ( float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 10) != 1) {
				chassis_follow_with_control(mouse_input[0], mouse_input[1]);
			}
			else {
				control_gimbal(mouse_input[0], mouse_input[1]);
			}
		}
		else //left switch middle or down
		{
			if (!in_following_flag && (gimbal_exceed_right_bound() || gimbal_exceed_left_bound() || DBUS_CheckPush(KEY_F) )) {
				in_following_flag = 1;
			}
			if (in_following_flag) {
				if ( float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 10) != 1) {
					chassis_follow_with_control(mouse_input[0], mouse_input[1]);
				}
				else {
					in_following_flag = 0;
					control_gimbal(mouse_input[0], mouse_input[1]);
				}
			}
			else control_gimbal(mouse_input[0], mouse_input[1]);
		}
		control_car(ch_input[0], ch_input[1], ch_input[2]);
	}
}

void remote_buff_adjust() {
	Set_CM_Speed(CAN1, 0, 0, 0, 0);
	static int16_t ch_input[3];
	ch_input[0] = DBUS_ReceiveData.rc.ch0;
	ch_input[1] = DBUS_ReceiveData.rc.ch1;
	ch_input[2] = DBUS_ReceiveData.rc.ch2;
	if (ch_input[2] > 600 )
	{
		if (ch_input[0] < -220)
		{
			if (ch_input[1] > 220)
			{
				manual_buff_pos[0].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[9].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}
			else if (ch_input[1] < -220)
			{
				manual_buff_pos[6].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[15].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}
			else 
			{
				manual_buff_pos[3].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[12].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}		
		}
		else if (ch_input[0] > 220)
		{
			if (ch_input[1] > 220)
			{
				manual_buff_pos[2].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[11].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}
			else if (ch_input[1] < -220)
			{
				manual_buff_pos[8].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[17].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}
			else 
			{
				manual_buff_pos[5].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[14].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}
		}
		else 
		{
			if (ch_input[1] > 220)
			{
				manual_buff_pos[1].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[10].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}
			else if (ch_input[1] < -220)
			{
				manual_buff_pos[7].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[16].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}
			else 
			{
				manual_buff_pos[4].mem = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
				manual_buff_pos[13].mem = (GMPitchEncoder.ecd_angle - init_pitch_pos)/19;
			}
		}
	}
	// writing into the flash, takes about 30s
	if (DBUS_ReceiveData.rc.switch_left == 2 && !is_writing_flash)
	{
		is_writing_flash = 1;
		u32 data[18];
		for (int i = 0; i < 18; ++i)
		{
			data[i] = manual_buff_pos[i].flash;
		}
		writeFlash(data, 18);
		FAIL_MUSIC;
	}
}
