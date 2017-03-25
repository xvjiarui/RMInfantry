#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
#include "customized_function.h"
#include "gimbal_control.h"

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
	send_to_gimbal(pid_gimbal_yaw_pos(target_yaw_pos), pid_gimbal_pitch_pos(target_pitch_pos));
}

void control_gimbal_pos_with_speed(int16_t target_yaw_pos, int16_t target_pitch_pos, int16_t input_yaw_speed)
{
	int16_t pid_yaw = pid_gimbal_yaw_pos_with_speed(target_yaw_pos, input_yaw_speed);
	int16_t pid_pitch = pid_gimbal_pitch_pos(target_pitch_pos);
	send_to_gimbal(pid_yaw, pid_pitch);
}

void send_to_gimbal(int16_t pid_yaw, int16_t pid_pitch) {
	Set_CM_Speed(CAN1, pid_yaw, pid_pitch, 0, 0);
}

void chassis_follow_with_control(int16_t input_yaw_speed, int16_t input_pitch_pos)
{
 gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
 float step = PID_output(&gimbal_reset_pid, 0);
 target_angle = current_angle + 10 * step;
 control_gimbal(chassis_ch2 * YAW_SPEED_TO_CHASSIS_CH2 + input_yaw_speed, input_pitch_pos);
}

void chassis_follow_with_control_old(int16_t input_yaw_speed, int16_t input_pitch_pos)
{
	gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
	float step = PID_output(&gimbal_reset_pid, 0);
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

void chassis_gimbal_relative_angle_with_control(int16_t relative_angle, int16_t input_yaw_speed, int16_t input_pitch_pos)
{
	gimbal_relative_angle_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
	float step = PID_output(&gimbal_relative_angle_pid, relative_angle);
	target_angle = current_angle + 10 * step;
	control_gimbal_pos_with_speed((GMYawEncoder.ecd_angle - init_yaw_pos + step * 20), input_pitch_pos, input_yaw_speed);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////////////////PID PART
//////////////////////////////////////////////////
//////////////////////////////////////////////////

int16_t pid_gimbal_yaw_speed(int16_t target_yaw_speed) {
	gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
	return PID_output2(&gimbal_speed_pid[0], target_yaw_speed, 660, -660, 100, 15);;
}

int16_t pid_gimbal_yaw_pos(int16_t target_yaw_pos) {
	return pid_gimbal_yaw_pos_with_speed(target_yaw_pos, 0);
}

int16_t pid_gimbal_yaw_pos_with_speed(int16_t target_yaw_pos, int16_t yaw_speed) {
	target_yaw_pos += init_yaw_pos;
	gimbal_pos_pid[0].current = GMYawEncoder.ecd_angle;
	int16_t target_yaw_speed = yaw_speed + PID_output2(&gimbal_pos_pid[0], target_yaw_pos, init_yaw_pos + 660, init_yaw_pos - 660, 100, 30);
	return pid_gimbal_yaw_speed(target_yaw_speed);
}

int16_t pid_gimbal_pitch_speed(int16_t target_pitch_speed) {
	gimbal_speed_pid[1].current = GMPitchEncoder.filter_rate;
	return PID_output2(&gimbal_speed_pid[1], target_pitch_speed, 660, -660, 100, 15);
}

int16_t pid_gimbal_pitch_pos(int16_t target_pitch_pos) {
	target_pitch_pos += init_pitch_pos;
	gimbal_pos_pid[1].current = GMPitchEncoder.ecd_angle;
	int16_t target_pitch_speed = PID_output2(&gimbal_pos_pid[1], target_pitch_pos, init_pitch_pos + 660, init_pitch_pos - 660, 100, 30);
	return pid_gimbal_pitch_speed(target_pitch_speed);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////////////////LOGIC PART
//////////////////////////////////////////////////
//////////////////////////////////////////////////
int16_t gimbal_exceed_right_bound() {
	if (GMYawEncoder.ecd_angle < init_yaw_pos + YAW_RIGHT_BOUND )
	{
		return 1;
	}
	else return 0;
}

int16_t gimbal_exceed_left_bound() {
	if (GMYawEncoder.ecd_angle > init_yaw_pos + YAW_LEFT_BOUND )
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

int16_t gimbal_yaw_back(){
	return float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 27);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////////////////FUNCTION PART
//////////////////////////////////////////////////
//////////////////////////////////////////////////
void buff_mode_gimbal_pos(int16_t index)
{
	target_angle = current_angle;
	if (index != -1)
	{
		int16_t input_yaw_pos = -27 * manual_buff_pos[index].mem;
		int16_t input_pitch_pos = 19 * manual_buff_pos[index + 9].mem;
		input_yaw_pos = (input_yaw_pos > 1620) ? 1620 : input_yaw_pos;
		input_yaw_pos = (input_yaw_pos < -1620) ? -1620 : input_yaw_pos;
		input_pitch_pos = (input_pitch_pos > 855) ? 855 : input_pitch_pos;
		input_pitch_pos = (input_pitch_pos < 0) ? 0 : input_pitch_pos;
		control_gimbal_pos(input_yaw_pos, input_pitch_pos);
	}
	else
	{
		control_gimbal_pos(GMYawEncoder.ecd_angle - init_yaw_pos, GMPitchEncoder.ecd_angle - init_pitch_pos);
	}
}

void buff_switch()
{
	if (DBUS_CheckPush(KEY_Q))
	{
		buff_mode_gimbal_pos(0);
	}
	else if (DBUS_CheckPush(KEY_W))
	{
		buff_mode_gimbal_pos(1);
	}
	else if (DBUS_CheckPush(KEY_E))
	{
		buff_mode_gimbal_pos(2);
	}
	else if (DBUS_CheckPush(KEY_A))
	{
		buff_mode_gimbal_pos(3);
	}
	else if (DBUS_CheckPush(KEY_S))
	{
		buff_mode_gimbal_pos(4);
	}
	else if (DBUS_CheckPush(KEY_D))
	{
		buff_mode_gimbal_pos(5);
	}
	else if (DBUS_CheckPush(KEY_Z))
	{
		buff_mode_gimbal_pos(6);
	}
	else if (DBUS_CheckPush(KEY_X))
	{
		buff_mode_gimbal_pos(7);
	}
	else if (DBUS_CheckPush(KEY_C))
	{
		buff_mode_gimbal_pos(8);
	}
	else buff_mode_gimbal_pos(-1);
}


