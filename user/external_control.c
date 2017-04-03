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
#include "Driver_Gun.h"

void external_control() {
	if (DBUS_Connected)
	{
		chassis_disconnect_init_flag = 0;
	}
	if (!(DBUS_ReceiveData.rc.switch_left ==3 && DBUS_ReceiveData.rc.switch_right == 3))
	{
		buff_mode = 0;
	}
	switch (DBUS_ReceiveData.rc.switch_right)
	{
		case 3:
			if (!DBUS_Connected)
			{
				DBUS_disconnect_handler();
			}
			else
			{
				switch (DBUS_ReceiveData.rc.switch_left)
				{
				case 1:
					remote_control();
					break;
				default:
					buff_mode = 1;
					remote_buff_adjust();
					break;
				}
			}
			break;
		case 2:
			if (!DBUS_Connected || (!Chassis_Connected && !Gimbal_Connected))
			{
				DBUS_disconnect_handler();
			}
			else if (!Chassis_Connected)
			{
				chassis_disconnect_handler();
				process_mouse_data();
				control_gimbal(mouse_input[0], mouse_input[1]);
			}
			else if (!Gimbal_Connected)
			{
				gimbal_disconnect_handler();
				process_keyboard_data();
				// control_car_speed(ch_input[0], ch_input[1], ch_input[2]);
				control_car(ch_input[0], ch_input[1], ch_input[2], NORMAL);
			}
			else
			{
				//when everything goes normal
				computer_control();
			}
			break;
		default:
			Set_CM_Speed(CAN1, 0, 0, 0, 0);
			Set_CM_Speed(CAN2, 0, 0, 0, 0);
			target_angle = current_angle;
			PID_init_all();
			input_init_all();
			DBUS_ReceiveData.mouse.x_position = 0;
			DBUS_ReceiveData.mouse.y_position = 0;
			break;
	}
}

void remote_control() {
	int16_t ch_changes[4] = {0, 0, 0, 0};
	ch_changes[0] = DBUS_ReceiveData.rc.ch0 - last_ch_input[0];
	ch_changes[1] = DBUS_ReceiveData.rc.ch1 - last_ch_input[1];
	ch_changes[2] = -DBUS_ReceiveData.rc.ch2 / 8 - last_ch_input[2];
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
	// control_car_speed(ch_input[0], ch_input[1], ch_input[2]);
	if ((ch_input[2] < 0 && gimbal_exceed_right_bound()) || (ch_input[2] > 0 && gimbal_exceed_left_bound()))
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
	if ( !gimbal_yaw_back()) {
		control_gimbal_with_chassis_following(ch_input[2], ch_input[3]);
	}
	else
	{
		control_gimbal(ch_input[2], ch_input[3]);
	}
	// control_car_speed(ch_input[0], ch_input[1], 0);
	control_car(ch_input[0], ch_input[1], 0, NORMAL);
}

void computer_control() {
	static int16_t in_following_flag; //a flag that help
	//keyboard part
	process_keyboard_data();
	//mouse part
	process_mouse_data();

	gimbal_in_buff_pos = 0;

	if (!GUN_ENCODER_Connected)
	{
		if (DBUS_ReceiveData.mouse.press_left)
		{
			POKE_SET_PWM(5000);
		}
		else POKE_SET_PWM(0);
		GUN_SetFree();
	}
	if (DBUS_CheckPush(KEY_V))
	{
		// in buff mode
		control_car(0, 0, 0, NORMAL);
		if (DBUS_ReceiveData.rc.switch_left == 3)
		{
			buff_switch();
		}
	}
	else if (DBUS_CheckPush(KEY_R))
	{
		// enter supply deport, open loop control
		mouse_input[0] = 0;
		last_mouse_input[0] = 0;
		mouse_input[1] = 0;
		last_mouse_input[1] = 0;
		DBUS_ReceiveData.mouse.y_position = 0;
		control_gimbal_pos(0, 0);
		// control_car_speed_open_loop(ch_input[0], ch_input[1], ch_input[2]);
		control_car(ch_input[0], ch_input[1], ch_input[2], OPEN_LOOP);
	}
	else if (DBUS_CheckPush(KEY_G))
	{
		// enter supply deport, semi-open loop control
		mouse_input[0] = 0;
		last_mouse_input[0] = 0;
		mouse_input[1] = 0;
		last_mouse_input[1] = 0;
		DBUS_ReceiveData.mouse.y_position = 0;
		control_gimbal_pos(0, 0);
		// control_car_speed_semi_closed_loop(ch_input[0], ch_input[1], ch_input[2]);
		control_car(ch_input[0], ch_input[1], ch_input[2], SEMI_CLOSED_LOOP);
	}
	else if (DBUS_CheckPush(KEY_C))
	{
		dancing_mode();
	}
	else
	{
		if (DBUS_ReceiveData.rc.switch_left == 1) { //left switch up
			//following mode
			//if there is angle difference between the chassis and gimbal, chassis will follow it

			if (!gimbal_yaw_back()) { //the q and e may cause bugs
				control_gimbal_with_chassis_following(mouse_input[0], mouse_input[1]);
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
				if (!gimbal_yaw_back()) {
					control_gimbal_with_chassis_following(mouse_input[0], mouse_input[1]);
				}
				else {
					in_following_flag = 0;
					control_gimbal(mouse_input[0], mouse_input[1]);
				}
			}
			else {
				control_gimbal(mouse_input[0], mouse_input[1]);
			}
		}
		// control_car_speed(ch_input[0], ch_input[1], ch_input[2]);
		control_car(ch_input[0], ch_input[1], ch_input[2], NORMAL);
	}
}
void process_mouse_data(void)
{
	int16_t mouse_changes[2] = {0, 0};
	mouse_changes[0] = - 4 * DBUS_ReceiveData.mouse.x - last_mouse_input[0];
	mouse_changes[1] = - 2 * DBUS_ReceiveData.mouse.y_position - last_mouse_input[1];
	int16_t max_mouse_change = 4;
	int16_t min_mouse_change = -4;
	limit_int_range(&mouse_changes[0], max_mouse_change, min_mouse_change);
	limit_int_range(&mouse_changes[1], max_mouse_change, min_mouse_change);
	mouse_input[0] += mouse_changes[0];
	last_mouse_input[0] = mouse_input[0];
	mouse_input[1] += mouse_changes[1];
	last_mouse_input[1] = mouse_input[1];
	if ((mouse_input[0] < 0 && gimbal_exceed_right_bound()) || (mouse_input[0] > 0 && gimbal_exceed_left_bound()))
	{
		mouse_input[0] = 0;
		last_mouse_input[0] = 0;
	}
	if ( mouse_input[1] > 19 * 45 && gimbal_exceed_upper_bound() )
	{
		mouse_input[1] = 19 * 45;
		last_mouse_input[1] = 19 * 45;
		DBUS_ReceiveData.mouse.y_position = -mouse_input[1] / 2;
	}
	if (mouse_input[1] < 0 && gimbal_exceed_lower_bound() )
	{
		mouse_input[1] = 0;
		last_mouse_input[1] = 0;
		DBUS_ReceiveData.mouse.y_position = -mouse_input[1] / 2;
	}
}


void process_keyboard_data(void)
{
	int16_t ch_changes[4] = {0, 0, 0, 0};
	float ratio = 1;
	ratio += 0.5 * (DBUS_CheckPush(KEY_SHIFT) - DBUS_CheckPush(KEY_CTRL));
	ch_changes[0] = (DBUS_CheckPush(KEY_D) - DBUS_CheckPush(KEY_A)) * 660 * ratio - last_ch_input[0];
	ch_changes[1] = (DBUS_CheckPush(KEY_W) - DBUS_CheckPush(KEY_S)) * 660 * ratio - last_ch_input[1];
	ch_changes[2] = (DBUS_CheckPush(KEY_E) - DBUS_CheckPush(KEY_Q)) * 660 * ratio - last_ch_input[2];

	int16_t max_change = 2;
	int16_t min_change = -2;
	for (int i = 0; i < 3; ++i)
	{
		limit_int_range(&ch_changes[i], max_change, min_change);
		ch_input[i] += ch_changes[i];
		last_ch_input[i] = ch_input[i];
	}
}

void remote_buff_adjust() {
	Set_CM_Speed(CAN1, 0, 0, 0, 0);
	static int16_t ch_input[3];
	ch_input[0] = DBUS_ReceiveData.rc.ch0;
	ch_input[1] = DBUS_ReceiveData.rc.ch1;
	ch_input[2] = DBUS_ReceiveData.rc.ch2;
	static uint8_t is_writing_flash = 0;
	if (ch_input[2] > 600 )
	{
		float yaw_pos = (GMYawEncoder.ecd_angle - init_yaw_pos);
		float pitch_pos = (GMPitchEncoder.ecd_angle - init_pitch_pos);
		limit_float_range(&yaw_pos, YAW_LEFT_BOUND, YAW_RIGHT_BOUND);
		limit_float_range(&pitch_pos, PITCH_UPPER_BOUND, 0);
		
		if (ch_input[0] < -220)
		{
			if (ch_input[1] > 220)
			{
				manual_buff_pos[0].mem = yaw_pos;
				manual_buff_pos[9].mem = pitch_pos;
			}
			else if (ch_input[1] < -220)
			{
				manual_buff_pos[6].mem = yaw_pos;
				manual_buff_pos[15].mem = pitch_pos;
			}
			else
			{
				manual_buff_pos[3].mem = yaw_pos;
				manual_buff_pos[12].mem = pitch_pos;
			}
		}
		else if (ch_input[0] > 220)
		{
			if (ch_input[1] > 220)
			{
				manual_buff_pos[2].mem = yaw_pos;
				manual_buff_pos[11].mem = pitch_pos;
			}
			else if (ch_input[1] < -220)
			{
				manual_buff_pos[8].mem = yaw_pos;
				manual_buff_pos[17].mem = pitch_pos;
			}
			else
			{
				manual_buff_pos[5].mem = yaw_pos;
				manual_buff_pos[14].mem = pitch_pos;
			}
		}
		else
		{
			if (ch_input[1] > 220)
			{
				manual_buff_pos[1].mem = yaw_pos;
				manual_buff_pos[10].mem = pitch_pos;
			}
			else if (ch_input[1] < -220)
			{
				manual_buff_pos[7].mem = yaw_pos;
				manual_buff_pos[16].mem = pitch_pos;
			}
			else
			{
				manual_buff_pos[4].mem = yaw_pos;
				manual_buff_pos[13].mem = pitch_pos;
			}
		}
	}
	// writing into the flash, takes about 2s
	if (DBUS_ReceiveData.rc.switch_left == 3)
	{
		is_writing_flash = 0;
	}
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

void dancing_mode(void)
{
	static int16_t dir = 1;
	float r;
	int16_t target_chassis_ch2_speed = 900;
	int16_t target_yaw_filter_rate = target_chassis_ch2_speed * YAW_SPEED_TO_CHASSIS_CH2;
	if (GMYawEncoder.ecd_angle - init_yaw_pos < (YAW_RIGHT_BOUND * 2 / 3))
	{
		r = (((YAW_RIGHT_BOUND) - (GMYawEncoder.ecd_angle - init_yaw_pos)) / (YAW_RIGHT_BOUND * 1 / 3));
	}
	else if (GMYawEncoder.ecd_angle - init_yaw_pos > (YAW_LEFT_BOUND * 2 / 3))
	{
		r = (((YAW_LEFT_BOUND) - (GMYawEncoder.ecd_angle - init_yaw_pos)) / (YAW_LEFT_BOUND * 1 / 3));
	}
	else
	{
		r = 1;
	}
	limit_float_range(&r, 1, 0.1);

	if (gimbal_exceed_left_bound())
	{
		dir = -1;
	}
	else if (gimbal_exceed_right_bound())
	{
		dir = 1;
	}

	control_gimbal(dir * r * target_yaw_filter_rate + mouse_input[0], mouse_input[1]);
	// control_car_speed_dancing(ch_input[0], ch_input[1], dir * r * target_chassis_ch2_speed);
	control_car(ch_input[0], ch_input[1], dir * r * target_chassis_ch2_speed, DANCING);
}

void chassis_disconnect_handler(void)
{
	input_init_ch();
	if (chassis_disconnect_init_flag == 0)
	{
		if (CM1Encoder.filter_rate != 0 || CM2Encoder.filter_rate != 0 || CM3Encoder.filter_rate != 0 || CM4Encoder.filter_rate != 0)
		{
			// control_car_speed(0, 0, 0);
			control_car(0, 0, 0, NORMAL);
		}
		else
		{
			chassis_disconnect_init_flag = 1;
		}
	}
	else
	{
		Set_CM_Speed(CAN2, 0, 0, 0, 0);
		PID_init_chassis();
	}
}

void gimbal_disconnect_handler(void)
{
	PID_init_gimbal();
	input_init_mouse();
	Set_CM_Speed(CAN1, 0, 0, 0, 0);
}

void DBUS_disconnect_handler(void)
{
	DBUS_ResetBuffer();
	chassis_disconnect_handler();
	gimbal_disconnect_handler();
}
