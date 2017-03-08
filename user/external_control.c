#include "function_list.h"
#include "external_control.h"
#include "PID_s.h"
#include "power_control.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "customized_function.h"

extern int16_t following_mode;
extern int16_t stable_mode;

void external_control(){
	static int16_t last_ch_input[4];
	static int16_t ch_input[4];
	if (DBUS_ReceiveData.rc.switch_right == 1) 
	{
			//emergency stop
			control_car(0, 0, 0); //warning: there is some emergency code inside this function
			control_gimbal_yaw_pos(0);
	}
	else if (DBUS_ReceiveData.rc.switch_right == 3)
	{
			int16_t ch_changes[4];
			ch_changes[0] = DBUS_ReceiveData.rc.ch0 - last_ch_input[0];
			ch_changes[1] = DBUS_ReceiveData.rc.ch1 - last_ch_input[1];
			/////////////////////
			//if test1 passes, delete following 2 lines
			int16_t max_change = 2;
			int16_t min_change = -2;
			for (int i = 0; i < 2; ++i)
			{
					if (ch_changes[i] > max_change)
					{
							ch_input[i] += max_change;
					}
					else if (ch_changes[i] < min_change)
					{
							ch_input[i] += min_change;
					}
					else ch_input[i] += ch_changes[i];
					last_ch_input[i] = ch_input[i];
					
					/////////////////////////////
					//// test1
					/*
					limit_int_range(&ch_changes[i], 2, -2);
					ch_input[i] += ch_changes[i];
					last_ch_input[i] = ch_input[i];
					*/
			}
			ch_input[2] = DBUS_ReceiveData.rc.ch2;
			control_car(ch_input[0], ch_input[1], ch_input[2]);
	}
	else {
		//to clearly organize the code
			int16_t ch_changes[4];
			int16_t mouse_changes[2];
			static int16_t mouse_input[2];
			static int16_t last_mouse_input[2];
		
			ch_changes[0] = (DBUS_CheckPush(KEY_D) - DBUS_CheckPush(KEY_A)) * 660 - last_ch_input[0];
			ch_changes[1] = (DBUS_CheckPush(KEY_W) - DBUS_CheckPush(KEY_S)) * 660 - last_ch_input[1];
			ch_changes[2] = (DBUS_CheckPush(KEY_E) - DBUS_CheckPush(KEY_Q)) * 660 - last_ch_input[2];
			////////////////////////////////
			//////// test2 pass, del 2 lines
			int16_t max_change = 2;
			int16_t min_change = -2;
			for (int i = 0; i < 3; ++i)
			{
					if (ch_changes[i] > max_change)
					{
							ch_input[i] += max_change;
					}
					else if (ch_changes[i] < min_change)
					{
							ch_input[i] += min_change;
					}
					else ch_input[i] += ch_changes[i];
					last_ch_input[i] = ch_input[i];
					
					
					////////////////////
					/////test2
					/*
					limit_int_range(&ch_changes[i], 2, -2);
					ch_input[i] += ch_changes[i];
					last_ch_input[i] = ch_input[i];
					*/
			}
			mouse_changes[0] = - 2 * DBUS_ReceiveData.mouse.x - last_mouse_input[0];
			mouse_changes[1] = - 2 * DBUS_ReceiveData.mouse.y - last_mouse_input[1];
			/////////////////////////
			//////test3 pass, del 2 lines
			int16_t max_mouse_change = 1;
			int16_t min_mouse_change = -1;
			for (int i = 0; i < 2; ++i)
			{
					if (mouse_changes[i] > max_mouse_change)
					{
							mouse_input[i] += max_mouse_change;
					}
					else if (mouse_changes[i] < min_mouse_change)
					{
							mouse_input[i] += min_mouse_change;
					}
					else mouse_input[i] += mouse_changes[i];
					last_mouse_input[i] = mouse_input[i];
					
					/////////////////////////
					////test 3
					/*
					limit_int_range(&mouse_changes[i], 1, -1);
					mouse_input[i] += mouse_changes[i];
					last_mouse_input[i] = mouse_input[i];
					*/
			}
			if ((mouse_input[0] > 0 && gimbal_exceed_right_bound()) || (mouse_input[0] < 0 && gimbal_exceed_left_bound()))
			{
					mouse_input[0] = 0;
					last_mouse_input[0] = 0;
			}
			
			// control_car(ch_input[0], ch_input[1], ch_input[2]);
			
			if (DBUS_ReceiveData.mouse.press_left)
			{
					// following_mode = 1;
					stable_mode = (stable_mode == 1) ? 0 : 1;
			}
			if (DBUS_ReceiveData.mouse.press_right)
			{
					gimbal_yaw_set(0);
			}
			else if (following_mode) {
					//if there is angle difference between the chassis and gimbal, chassis will follow it
					if ( float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 15) != 1)
					{
							gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
							float step = PID_output(&gimbal_reset_pid, 0);
							target_angle = current_angle + 10 * step;
							control_gimbal_yaw_pos(GMYawEncoder.ecd_angle - init_yaw_pos + step * 27);
					}
					//?????????????
					else following_mode = 0;
			}
			else if (stable_mode)
			{
					s16 gyro_angle_speed = gyro_get_vel();
					int16_t target_yaw_filter_rate = - gyro_angle_speed * 27 * 6144 / 3600 /1000; 
					debug = target_yaw_filter_rate;
					if ((target_yaw_filter_rate > 0 && gimbal_exceed_right_bound()) || (target_yaw_filter_rate < 0 && gimbal_exceed_left_bound()))
					{
							mouse_input[0] = 0;
							last_mouse_input[0] = 0;
							target_yaw_filter_rate = 0;
					}
					control_gimbal_yaw_speed(target_yaw_filter_rate);
			}
			else {
					control_gimbal_yaw_speed(mouse_input[0]);
			}
			control_car(ch_input[0], ch_input[1], ch_input[2]);
			control_car(ch_input[0], ch_input[1], ch_input[2]);
	}
}
