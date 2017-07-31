#define EXTERNAL_CONTROL_FILE
#include "function_list.h"
#include "external_control.h"
#include "PID_s.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "customized_function.h"
#include "global_variable.h"
#include "flash.h"
#include "buzzer_song.h"
#include "Driver_Gun.h"
#include "Driver_Manifold.h"
#include "param.h"
#include "const.h"

extern volatile u32 ticks_msimg;

void external_control_init(void)
{
	Chassis_Connected = 1;
	Gimbal_Connected = 1;
	DBUS_Connected = 1;
	Judge_Connected = 0;
	chassis_ch2 = 0;
	memset(last_ch_input, 0, sizeof(last_ch_input));
	memset(ch_input, 0, sizeof(ch_input));
	memset(mouse_input, 0 , sizeof(mouse_input));
	memset(last_ch_input, 0, sizeof(last_ch_input));
	dancing = 0;
	rune = 0;
	chassis_ch2_dancing_input = 0;
	status = 0;

	int_debug = 0;
	int_debug2 = 0;
	float_debug = 0;
	float_debug2 = 0;
	InfantryID = readFlash(0);
	// GOGOGO 0
	// RUNRUNRUN 1
	// BACKBACKBACK 2
}

void external_control(void) {
	if (DBUS_Connected)
	{
		chassis_already_auto_stop = 0;
	}
	if (!(DBUS_ReceiveData.rc.switch_left == 3 && DBUS_ReceiveData.rc.switch_right == 3))
	{
		buff_mode = 0;
	}
	status = ticks_msimg > 10000? DBUS_ReceiveData.rc.switch_right : 1;
	switch (status)
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
				// buff_mode = 1;
				remote_shooting_adjust();
				break;
			}
		}
		break;
	case 2:
		if (!DBUS_Connected)
		{
			DBUS_disconnect_handler();
		}
		else if (!Chassis_Connected && !Gimbal_Connected)
		{
			chassis_disconnect_handler();
			gimbal_disconnect_handler();
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
		PID_Reset_all();
		input_init_all();
		DBUS_ReceiveData.mouse.x_position = 0;
		DBUS_ReceiveData.mouse.y_position = 0;
		break;
	}
}

void remote_control(void) {
	int16_t ch_changes[4] = {0, 0, 0, 0};
	ch_changes[0] = CONTROLLER_RWLW_RATIO * DBUS_ReceiveData.rc.ch0 - last_ch_input[0];
	ch_changes[1] = CONTROLLER_FWBW_RATIO * DBUS_ReceiveData.rc.ch1 - last_ch_input[1];
	ch_changes[2] = -CONTROLLER_YAW_RATIO * DBUS_ReceiveData.rc.ch2 - last_ch_input[2];
	ch_changes[3] = CONTROLLER_PITCH_RATIO * DBUS_ReceiveData.rc.ch3 - last_ch_input[3];
	int16_t max_change = RWLW_ACCELERATION;
	int16_t min_change = -RWLW_ACCELERATION;
	for (int i = 0; i < 4; ++i)
	{
		limit_int_range(&ch_changes[i], max_change, min_change);
		ch_input[i] += ch_changes[i];
		last_ch_input[i] = ch_input[i];
	}
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
	control_car(ch_input[0], ch_input[1], 0, NORMAL);
	int_debug = ch_input[2];
}

void computer_control(void) {
	static int16_t in_following_flag; //a flag that help
	static uint8_t in_countering_flag = 0; //marking COUNTER mode
	static uint8_t pushed_Z_this_time = 0;
	//keyboard part
	process_keyboard_data();
	//mouse part
	process_mouse_data();

	gimbal_in_buff_pos = 0;

	if (!DBUS_CheckPush(KEY_Z))
	{
		pushed_Z_this_time = 0;
	}

	if (DBUS_CheckPush(KEY_C))
	{
		dancing = 1;
	}
	if (DBUS_CheckPush(KEY_V) && gimbal_yaw_back())
	{
		rune = 1;
		LED_control(LASER, 0);
	}

	if (DBUS_CheckPush(KEY_Z) || in_countering_flag == 1)
	{
		in_countering_flag = 1;
		static float dir = -1;
		if (DBUS_CheckPush(KEY_Z) && pushed_Z_this_time == 0)
		{
			dir = -dir;
			pushed_Z_this_time = 1;
		}
		if (!gimbal_yaw_back_angle(YAW_ANGLE_RATIO * 45 * dir)) {
			control_gimbal_with_chassis_following_angle(mouse_input[0], mouse_input[1], YAW_ANGLE_RATIO * 45 * dir);
		}
		else {
			control_gimbal(mouse_input[0], mouse_input[1]);
		}
		control_car(ch_input[0], ch_input[1], ch_input[2], COUNTER);
		if (DBUS_CheckPush(KEY_F)) in_countering_flag = 0;
	}
	else if (rune)
	{
		rune_mode();
	}
	else if (DBUS_CheckPush(KEY_R))
	{
		// enter supply deport, open loop control
		control_gimbal_pos(0, 0);
    control_car(ch_input[0], ch_input[1], -mouse_input[0]*20, OPEN_LOOP);

		/*mouse_input[0] = 0;*/
		/*last_mouse_input[0] = 0;*/
		mouse_input[1] = 0;
		last_mouse_input[1] = 0;
		DBUS_ReceiveData.mouse.y_position = 0;
		// control_car_speed_open_loop(ch_input[0], ch_input[1], ch_input[2]);
		/*control_car(ch_input[0], ch_input[1], ch_input[2], OPEN_LOOP);*/
	}
	else if (dancing)
	{
		dancing_mode();
	}
	// following logic
	else
	{
		if (1) {
			// disabble semi-auto follow mode
			//left switch up
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
	// int16_t temp = ABS(CM1Encoder.filter_rate);
	// int_debug = int_debug > temp ? int_debug : temp;
}
void process_mouse_data(void)
{
	int16_t mouse_changes[2] = {0, 0};
	mouse_changes[0] = - MOUSE_YAW_RATIO * DBUS_ReceiveData.mouse.x - last_mouse_input[0];
	mouse_changes[1] = - MOUSE_PITCH_RATIO * DBUS_ReceiveData.mouse.y_position - last_mouse_input[1];
	limit_int_range(&mouse_changes[0], YAW_ACCELERATION, -YAW_ACCELERATION);
	limit_int_range(&mouse_changes[1], PITCH_ACCELERATION, -PITCH_ACCELERATION);
	mouse_input[0] += mouse_changes[0];
	last_mouse_input[0] = mouse_input[0];
	mouse_input[1] += mouse_changes[1];
	last_mouse_input[1] = mouse_input[1];
	float ratio = PID_UpdateValue(&mouse_input_pid, 1755, fabs(GMYawEncoder.ecd_angle));
	mouse_input[0] *= ratio;
	if ((mouse_input[0] < 0 && gimbal_exceed_right_bound()) || (mouse_input[0] > 0 && gimbal_exceed_left_bound()))
	{
		mouse_input[0] = 0;
		last_mouse_input[0] = 0;
	}
	if ( mouse_input[1] > 19 * 45 && gimbal_exceed_upper_bound() )
	{
		mouse_input[1] = 19 * 45;
		last_mouse_input[1] = 19 * 45;
		DBUS_ReceiveData.mouse.y_position = -mouse_input[1] / MOUSE_PITCH_RATIO;
	}
	if (mouse_input[1] < 0 && gimbal_exceed_lower_bound() )
	{
		mouse_input[1] = 0;
		last_mouse_input[1] = 0;
		DBUS_ReceiveData.mouse.y_position = -mouse_input[1] / MOUSE_PITCH_RATIO;
	}
}


void process_keyboard_data(void)
{
	int16_t ch_changes[4] = {0, 0, 0, 0};
	float ratio = 1;
	ratio += 0.6666666666 * (DBUS_CheckPush(KEY_SHIFT) - DBUS_CheckPush(KEY_CTRL));
	ch_changes[0] = (DBUS_CheckPush(KEY_D) - DBUS_CheckPush(KEY_A)) * 660 * KEYBOARD_RWLW_RATIO * ratio - last_ch_input[0];
	ch_changes[1] = (DBUS_CheckPush(KEY_W) - DBUS_CheckPush(KEY_S)) * 660 * KEYBOARD_FWBW_RATIO * ratio - last_ch_input[1];
	ch_changes[2] = (DBUS_CheckPush(KEY_E) - DBUS_CheckPush(KEY_Q)) * 660 * ratio - last_ch_input[2];
	ch_changes[2] = 0;
	int16_t max_change = 1;
	int16_t min_change = -1;
	limit_int_range(&ch_changes[0], RWLW_ACCELERATION, -RWLW_ACCELERATION);
	limit_int_range(&ch_changes[1], FWBW_ACCELERATION, -FWBW_ACCELERATION);
	limit_int_range(&ch_changes[2], max_change, min_change);
	for (int i = 0; i < 3; ++i)
	{
		// limit_int_range(&ch_changes[i], max_change, min_change);
		ch_input[i] += ch_changes[i];
		last_ch_input[i] = ch_input[i];
	}
}

void remote_buff_adjust(void) {
	send_to_gimbal(0, 0);
	static int16_t ch_input[4];
	ch_input[0] = DBUS_ReceiveData.rc.ch0;
	ch_input[1] = DBUS_ReceiveData.rc.ch1;
	ch_input[2] = DBUS_ReceiveData.rc.ch2;
	ch_input[3] = DBUS_ReceiveData.rc.ch3;
	static uint8_t is_writing_flash = 0;
	if (ch_input[3] > 600)
	{
		clearing_ammo = 1;
	}
	if (clearing_ammo)
	{
		if (ch_input[3] > 600)
		{
			GUN_DriverInput = 500;
		}
		else
		{
			GUN_DriverInput = 0;
			clearing_ammo = 0;
			PID_Reset_driver();
		}
	}
	if (ch_input[3] < -600)
	{
		GUN_DriverInput = 0;
	}
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
		// u32 data[18];
		// for (int i = 0; i < 18; ++i)
		// {
		// 	data[i] = manual_buff_pos[i].flash;
		// }
		u32 data[1];
		data[0] = InfantryID;
		writeFlash(data, 1);
		FAIL_MUSIC;
	}
}

void remote_shooting_adjust(void) {
	send_to_gimbal(0, 0);
	chassis_disconnect_handler();
	static uint8_t is_writing_flash = 0;
	// writing into the flash, takes about 2s
	if (DBUS_ReceiveData.rc.switch_left == 3)
	{
		is_writing_flash = 0;
	}
	if (DBUS_ReceiveData.rc.switch_left == 2 && !is_writing_flash)
	{
		is_writing_flash = 1;
		// u32 data[18];
		// for (int i = 0; i < 18; ++i)
		// {
		// 	data[i] = manual_buff_pos[i].flash;
		// }
		u32 data[1];
		data[0] = InfantryID;
		writeFlash(data, 1);
		FAIL_MUSIC;
	}
}
void dancing_mode(void)
{
	static int16_t dir = 1;
	if (gimbal_approach_left_bound())
	{
		dir = -1;
	}
	else if (gimbal_approach_right_bound())
	{
		dir = 1;
	}
	int16_t chassis_ch2_target = DANCING_SPEED * dir;
	int16_t chassis_ch2_change = chassis_ch2_target - chassis_ch2_dancing_input;
	limit_int_range(&chassis_ch2_change, 5, -5);
	chassis_ch2_dancing_input += chassis_ch2_change;
	int16_t yaw_filter_rate_input = chassis_ch2_dancing_input * YAW_SPEED_TO_CHASSIS_CH2;
	control_gimbal(yaw_filter_rate_input + mouse_input[0], mouse_input[1]);
	control_car(ch_input[0], ch_input[1], chassis_ch2_dancing_input, DANCING);
	if (!DBUS_CheckPush(KEY_C) && chassis_ch2_dancing_input == 0)
	{
		dancing = 0;
	}
}

void rune_mode(void)
{
	static uint8_t runeGPIO = 0;
	static int last_rune_index = -1;
	static float input_yaw_pos = 0;
	static float input_pitch_pos = 0;
	float target_yaw_pos = rune_angle_x * YAW_ANGLE_RATIO;
	float target_pitch_pos = rune_angle_y * PITCH_ANGLE_RATIO + PITCH_HORIZONTAL_OFFSET - 5 * PITCH_ANGLE_RATIO; 
#ifdef RELOAD_BULLET
	if (!DBUS_CheckPush(KEY_V) || lastRuneTick + 550 < ticks_msimg)
	{
		target_yaw_pos = 0;
		target_pitch_pos = 0; 
	}
#else 
	if (!DBUS_CheckPush(KEY_V))
	{
		target_yaw_pos = 0;
		target_pitch_pos = 0; 
	}

#endif
	float yaw_pos_change = target_yaw_pos - input_yaw_pos;
	float pitch_pos_change = target_pitch_pos - input_pitch_pos;
	limit_float_range(&yaw_pos_change, YAW_ACCELERATION, -YAW_ACCELERATION);
	limit_float_range(&pitch_pos_change, PITCH_ACCELERATION, -PITCH_ACCELERATION);
	input_yaw_pos += yaw_pos_change;
	input_pitch_pos += pitch_pos_change;
	if (DBUS_CheckPush(KEY_SHIFT))
	{
		gimbal_in_buff_pos = last_rune_index != -1 && gimbal_check_pos(target_yaw_pos, target_pitch_pos, 7.0);
	}
	else gimbal_in_buff_pos = last_rune_index != -1 && gimbal_check_pos(target_yaw_pos, target_pitch_pos, 1.0);
	if (!DBUS_CheckPush(KEY_V))
	{	
		rune = !gimbal_check_pos(0, 0, 1.0);
		if (!rune)
		{
			LED_control(LASER, 1);
		}
	} 
	control_gimbal_pos(input_yaw_pos, input_pitch_pos);
	if (rune_index != -1 && isNewRuneAngle 
		&& ( (!DBUS_CheckPush(KEY_SHIFT) && rune_index != last_rune_index) 
		|| DBUS_CheckPush(KEY_SHIFT) ) )
	{
		shootRune = 1;
		isNewRuneAngle = 0;
		last_rune_index = rune_index;
	}
	if (DBUS_CheckPush(KEY_SHIFT) && runeGPIO == 0)
	{
		runeGPIO = 1;
		Manifold_Rune_Select(runeGPIO);
	}

	if (!DBUS_CheckPush(KEY_SHIFT) && runeGPIO == 1)
	{
		runeGPIO = 0;
		Manifold_Rune_Select(runeGPIO);
	}

	control_car(0, 0, 0, NORMAL);
}

void chassis_disconnect_handler(void)
{
	input_init_ch();
	Set_CM_Speed(CAN2, 0, 0, 0, 0);
	PID_Reset_chassis();
}

void gimbal_disconnect_handler(void)
{
	PID_Reset_gimbal();
	input_init_mouse();
	Set_CM_Speed(CAN1, 0, 0, 0, 0);
}

void DBUS_disconnect_handler(void)
{
	gimbal_disconnect_handler();
	input_init_ch();
	chassis_auto_stop();
}

void chassis_auto_stop(void)
{
	if (!chassis_already_auto_stop)
	{
		if (CM1Encoder.filter_rate != 0 || CM2Encoder.filter_rate != 0 || CM3Encoder.filter_rate != 0 || CM4Encoder.filter_rate != 0)
		{
			control_car(0, 0, 0, NORMAL);
		}
		else
		{
			chassis_already_auto_stop = 1;
		}
	}
	else
	{
		chassis_disconnect_handler();
		DBUS_ResetBuffer();
		PID_Reset_all();
	}
}
