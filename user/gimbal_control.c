#define GIMBAL_CONTROL_FILE

#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
#include "customized_function.h"
#include "gimbal_control.h"
#include "chassis_control.h"
#include "external_control.h"
#include "Driver_Gun.h"
#include "const.h"
#include "flash.h"

void gimbal_control_init(void)
{
	init_yaw_pos = GMYawEncoder.ecd_angle;
	init_pitch_pos = GMPitchEncoder.ecd_angle + 5 * PITCH_ANGLE_RATIO;
	gimbal_follow = 0;
	buff_mode = 0;
	gimbal_in_buff_pos = 0;
	buff_pressed = 0;
	clearing_ammo = 0;
	for (int i = 0; i < 18; ++i)
	{
		manual_buff_pos[i].flash = readFlash(i);
	}
	GUN_DriverInput = 0;
	GUN_TargetPos = GMxEncoder.ecd_angle;
}
//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////////////////CONTROL PART
//////////////////////////////////////////////////
//////////////////////////////////////////////////
void control_gimbal(int16_t target_yaw_speed, int16_t target_pitch_pos) {
	send_to_gimbal(pid_gimbal_yaw_speed(target_yaw_speed), pid_gimbal_pitch_pos(target_pitch_pos));
}

void control_gimbal_yaw_speed(int16_t target_yaw_speed) {
	send_to_gimbal(pid_gimbal_yaw_speed(target_yaw_speed), 0);
}

void control_gimbal_speed(int16_t target_yaw_speed, int16_t target_pitch_speed) {
	send_to_gimbal(pid_gimbal_yaw_speed(target_yaw_speed), pid_gimbal_pitch_speed(target_pitch_speed));
}

void control_gimbal_yaw_pos(int16_t target_yaw_pos) {
	send_to_gimbal(pid_gimbal_yaw_pos(target_yaw_pos), 0);
}

void control_gimbal_pos(int16_t target_yaw_pos, int16_t target_pitch_pos) {
	limit_int_range(&target_yaw_pos, YAW_LEFT_BOUND, YAW_RIGHT_BOUND);
		limit_int_range(&target_pitch_pos, PITCH_UPPER_BOUND, 0);
	send_to_gimbal(pid_gimbal_yaw_pos(target_yaw_pos), pid_gimbal_pitch_pos(target_pitch_pos));
}

void control_gimbal_pos_with_speed(int16_t target_yaw_pos, int16_t target_pitch_pos, int16_t input_yaw_speed)
{
	limit_int_range(&target_yaw_pos, YAW_LEFT_BOUND, YAW_RIGHT_BOUND);
		limit_int_range(&target_pitch_pos, PITCH_UPPER_BOUND, 0);
	int16_t pid_yaw = pid_gimbal_yaw_pos_with_speed(target_yaw_pos, input_yaw_speed);
	int16_t pid_pitch = pid_gimbal_pitch_pos(target_pitch_pos);
	send_to_gimbal(pid_yaw, pid_pitch);
}

void send_to_gimbal(int16_t pid_yaw, int16_t pid_pitch) {
	GUN_Update();
	Set_CM_Speed(CAN1, pid_yaw, pid_pitch, GUN_DriverInput, 0);
}

void control_gimbal_with_chassis_following(int16_t input_yaw_speed, int16_t input_pitch_pos)
{
	control_gimbal_with_chassis_following_angle(input_yaw_speed, input_pitch_pos, 0);
}

void control_gimbal_with_chassis_following_angle(int16_t input_yaw_speed, int16_t input_pitch_pos, float angle)
{
 // gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
 // float step = PID_output(&gimbal_reset_pid, 0);
	float step = PID_UpdateValue(&gimbal_reset_pid, angle, (GMYawEncoder.ecd_angle - init_yaw_pos));
 if (!fast_turning)
 {
 	target_angle = current_angle + 10 * step;
 }
 if (gimbal_follow)
 {
	 input_yaw_speed += chassis_ch2 * YAW_SPEED_TO_CHASSIS_CH2;
 }
 if (fast_turning)
 {
 	control_gimbal(0, input_pitch_pos);
 }	
 else
 {
	 control_gimbal(input_yaw_speed, input_pitch_pos);
 }
 
}
void chassis_follow_with_control_old(int16_t input_yaw_speed, int16_t input_pitch_pos)
{
	// gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
	// float step = PID_output(&gimbal_reset_pid, 0);
	float step = PID_UpdateValue(&gimbal_reset_pid, 0, (GMYawEncoder.ecd_angle - init_yaw_pos));
	target_angle = current_angle + 10 * step;
	control_gimbal_pos_with_speed((GMYawEncoder.ecd_angle - init_yaw_pos + step * 27), input_pitch_pos, input_yaw_speed);
}

// void chassis_follow_pos(int16_t input_yaw_pos, int16_t input_pitch_pos)
// {
// 	gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
// 	float step = PID_output(&gimbal_reset_pid, 0);
// 	target_angle = current_angle + 10 * step;
// 	control_gimbal_pos(GMYawEncoder.ecd_angle - init_yaw_pos + step * 27, input_pitch_pos);
// 	// test2
// 	// float difference = -(GMYawEncoder.ecd_angle - init_yaw_pos)/27;
// 	// target_angle = current_angle + 10 * difference;
// 	// control_gimbal_pos(input_yaw_pos + difference * 27, input_pitch_pos);
// }

// void chassis_gimbal_relative_angle_with_control(int16_t relative_angle, int16_t input_yaw_speed, int16_t input_pitch_pos)
// {
// 	gimbal_relative_angle_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
// 	float step = PID_output(&gimbal_relative_angle_pid, relative_angle);
// 	target_angle = current_angle + 10 * step;
// 	control_gimbal_pos_with_speed((GMYawEncoder.ecd_angle - init_yaw_pos + step * 20), input_pitch_pos, input_yaw_speed);
// }

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////////////////PID PART
//////////////////////////////////////////////////
//////////////////////////////////////////////////

int16_t pid_gimbal_yaw_speed(int16_t target_yaw_speed) {
	// gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
	if ((gimbal_exceed_left_bound() && target_yaw_speed > 0) || (gimbal_exceed_right_bound() && target_yaw_speed < 0))
	{
		target_yaw_speed = 0;
	}
	// return PID_output2(&gimbal_speed_pid[0], target_yaw_speed, 660, -660, 100, 15);;
	return PID_UpdateValue(&gimbal_speed_pid[0], target_yaw_speed, GMYawEncoder.filter_rate);
}

int16_t pid_gimbal_yaw_pos(int16_t target_yaw_pos) {
	return pid_gimbal_yaw_pos_with_speed(target_yaw_pos, 0);
}

int16_t pid_gimbal_yaw_pos_with_speed(int16_t target_yaw_pos, int16_t yaw_speed) {
	target_yaw_pos += init_yaw_pos;
	// gimbal_pos_pid[0].current = GMYawEncoder.ecd_angle;
	// int16_t target_yaw_speed = yaw_speed + PID_output2(&gimbal_pos_pid[0], target_yaw_pos, init_yaw_pos + 660, init_yaw_pos - 660, 100, 30);
	int16_t target_yaw_speed = yaw_speed + PID_UpdateValue(&gimbal_pos_pid[0], target_yaw_pos, GMYawEncoder.ecd_angle);
	return pid_gimbal_yaw_speed(target_yaw_speed);
}

int16_t pid_gimbal_pitch_speed(int16_t target_pitch_speed) {
	// gimbal_speed_pid[1].current = GMPitchEncoder.filter_rate;
	// return PID_output2(&gimbal_speed_pid[1], target_pitch_speed, 660, -660, 100, 15);
	return PID_UpdateValue(&gimbal_speed_pid[1], target_pitch_speed, GMPitchEncoder.filter_rate);
}

int16_t pid_gimbal_pitch_pos(int16_t target_pitch_pos) {
	target_pitch_pos += init_pitch_pos;
	// gimbal_pos_pid[1].current = GMPitchEncoder.ecd_angle;
	// int16_t target_pitch_speed = PID_output2(&gimbal_pos_pid[1], target_pitch_pos, init_pitch_pos + 660, init_pitch_pos - 660, 100, 30);
	int16_t target_pitch_speed = PID_UpdateValue(&gimbal_pos_pid[1], target_pitch_pos, GMPitchEncoder.ecd_angle);
	return pid_gimbal_pitch_speed(target_pitch_speed);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////////////////LOGIC PART
//////////////////////////////////////////////////
//////////////////////////////////////////////////
int16_t gimbal_exceed_right_bound() 
{
	if (GMYawEncoder.ecd_angle < init_yaw_pos + YAW_RIGHT_BOUND || GMYawEncoder.ecd_angle == init_yaw_pos + YAW_RIGHT_BOUND )
	{
		return 1;
	}
	else return 0;
}

int16_t gimbal_exceed_left_bound() 
{
	if (GMYawEncoder.ecd_angle > init_yaw_pos + YAW_LEFT_BOUND || GMYawEncoder.ecd_angle == init_yaw_pos + YAW_LEFT_BOUND )
	{
		return 1;
	}
	else return 0;
}

int16_t gimbal_approach_right_bound()
{
	if (GMYawEncoder.ecd_angle < (init_yaw_pos + YAW_RIGHT_BOUND * 0.6))
	{
		return 1;
	}
	else return 0;
}

int16_t gimbal_approach_left_bound()
{
	if (GMYawEncoder.ecd_angle > (init_yaw_pos + YAW_LEFT_BOUND * 0.6))
	{
		return 1;
	}
	else return 0;
}

int16_t gimbal_exceed_upper_bound() {
	if (GMPitchEncoder.ecd_angle > init_pitch_pos + PITCH_UPPER_BOUND )
	{
		return 1;
	}
	else return 0;
}

int16_t gimbal_exceed_lower_bound() {
	if (GMPitchEncoder.ecd_angle < init_pitch_pos )
	{
		return 1;
	}
	else return 0;
}

uint8_t gimbal_check_pos(int16_t target_yaw_pos, int16_t target_pitch_pos)
{
	uint8_t check_yaw = float_equal((GMYawEncoder.ecd_angle - init_yaw_pos), target_yaw_pos, 27);
	uint8_t check_pitch = float_equal((GMPitchEncoder.ecd_angle - init_pitch_pos), target_pitch_pos, 19);
	return check_yaw && check_pitch;
}

int16_t gimbal_yaw_back(){
	return float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 27);
}

int16_t gimbal_yaw_back_angle(float angle){
	return float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, angle, 27);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////////////////FUNCTION PART
//////////////////////////////////////////////////
//////////////////////////////////////////////////
// void buff_mode_gimbal_pos(int16_t index)
// {
// 	target_angle = current_angle;
// 	if (index != -1)
// 	{
// 		float input_yaw_pos = manual_buff_pos[index].mem;
// 		float input_pitch_pos = manual_buff_pos[index + 9].mem;
// 		limit_float_range(&input_yaw_pos, YAW_LEFT_BOUND, YAW_RIGHT_BOUND);
// 		limit_float_range(&input_pitch_pos, PITCH_UPPER_BOUND, 0);
// 		control_gimbal_pos(input_yaw_pos, input_pitch_pos);
// 	}
// 	else
// 	{
// 		control_gimbal_pos(GMYawEncoder.ecd_angle - init_yaw_pos, GMPitchEncoder.ecd_angle - init_pitch_pos);
// 	}
// }

// void buff_switch()
// {
// 	static int16_t Last_Status = -1;
	
// 	if (DBUS_CheckPush(KEY_Q))
// 	{
// 		Last_Status = 0;
// 		buff_pressed = 1;
// 	}
// 	else if (DBUS_CheckPush(KEY_W))
// 	{
// 		Last_Status = 1;
// 		buff_pressed = 1;
// 	}
// 	else if (DBUS_CheckPush(KEY_E))
// 	{
// 		Last_Status = 2;
// 		buff_pressed = 1;
// 	}
// 	else if (DBUS_CheckPush(KEY_A))
// 	{
// 		Last_Status = 3;
// 		buff_pressed = 1;
// 	}
// 	else if (DBUS_CheckPush(KEY_S))
// 	{
// 		Last_Status = 4;
// 		buff_pressed = 1;
// 	}
// 	else if (DBUS_CheckPush(KEY_D))
// 	{
// 		Last_Status = 5;
// 		buff_pressed = 1;
// 	}
// 	else if (DBUS_CheckPush(KEY_Z))
// 	{
// 		Last_Status = 6;
// 		buff_pressed = 1;
// 	}
// 	else if (DBUS_CheckPush(KEY_X))
// 	{
// 		Last_Status = 7;
// 		buff_pressed = 1;
// 	}
// 	else if (DBUS_CheckPush(KEY_C))
// 	{
// 		Last_Status = 8;
// 		buff_pressed = 1;
// 	}
// 	buff_mode_gimbal_pos(Last_Status);
// 	gimbal_in_buff_pos = gimbal_check_in_buff_pos(Last_Status, buff_pressed);
// }


// uint8_t gimbal_check_in_buff_pos(int16_t status, uint8_t pressed)
// {
// 	if (status == -1 || !pressed)
// 	{
// 		return 0;
// 	}
// 	return gimbal_check_pos(manual_buff_pos[status].mem, manual_buff_pos[status + 9].mem);
// }


