#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
#include "customized_function.h"

// inpur relative position
// range (-5, 5)
void control_gimbal(int16_t ch2, int16_t ch3)
{
    int16_t target_yaw_speed = ch2;
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    int16_t yaw_input = PID_output2(&gimbal_speed_pid[0], target_yaw_speed, 660, -660, 100, 15);
    int16_t target_pitch_position = ch3 + init_pitch_pos;
    gimbal_pos_pid[1].current = GMPitchEncoder.ecd_angle;
    int16_t target_pitch_speed = PID_output2(&gimbal_pos_pid[1], target_pitch_position, init_pitch_pos + 660, init_pitch_pos - 660, 100, 30);
    gimbal_speed_pid[1].current = GMPitchEncoder.filter_rate;
    int16_t pitch_input = PID_output2(&gimbal_speed_pid[1], target_pitch_speed, 660, -660, 100, 15);
    Set_CM_Speed(CAN1, yaw_input, pitch_input, 0, 0);
}

void control_gimbal_yaw_pos(int16_t ch2) {
    int16_t ratio = 1;
    int16_t target_position = ratio * ch2 + init_yaw_pos;
    gimbal_pos_pid[0].current = GMYawEncoder.ecd_angle;
    int16_t target_speed = PID_output2(&gimbal_pos_pid[0], target_position, init_yaw_pos + ratio * 660, init_yaw_pos - ratio * 660, 100, 30);
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    int16_t input = PID_output2(&gimbal_speed_pid[0], target_speed, 660, -660, 100, 15);
    Set_CM_Speed(CAN1, input, 0, 0, 0);
}

void control_gimbal_yaw_speed(int16_t ch2) {
    int16_t target_speed = ch2;
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    int16_t input = PID_output2(&gimbal_speed_pid[0], target_speed, 660, -660, 100, 15);
    Set_CM_Speed(CAN1, input, 0, 0, 0);
}

void control_gimbal_pos(int16_t ch2, int16_t ch3) {
    int16_t target_position[2] = {ch2, ch3};
    gimbal_pos_pid[0].current = GMYawEncoder.ecd_angle;
    gimbal_pos_pid[1].current = GMPitchEncoder.ecd_angle;
    int16_t target_speed[2] ={0, 0};
    target_speed[0] = PID_output2(&gimbal_pos_pid[0], target_position[0], 660, -660, 100, 30);
    target_speed[1] = PID_output2(&gimbal_pos_pid[1], target_position[1], 660, -600, 100, 30);
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    gimbal_speed_pid[1].current = GMPitchEncoder.filter_rate;
    int16_t input[2] = {0, 0};
    input[0] = PID_output2(&gimbal_speed_pid[0], target_speed[0], 660, -660, 100, 15);
    input[1] = PID_output2(&gimbal_speed_pid[1], target_speed[1], 660, -660, 100, 15);
    Set_CM_Speed(CAN1, input[0], input[1], 0, 0);
}
void control_gimbal_speed(int16_t ch2, int16_t ch3)
{
    int16_t target_speed[2] = {ch2, ch3};
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    gimbal_speed_pid[1].current = GMPitchEncoder.filter_rate;
    int16_t input[2] = {0, 0};
    input[0] = PID_output2(&gimbal_speed_pid[0], target_speed[0], 660, -660, 100, 15);
    input[1] = PID_output2(&gimbal_speed_pid[1], target_speed[1], 660, -660, 100, 15);
    Set_CM_Speed(CAN1, input[0], input[1], 0, 0);
}

int16_t gimbal_exceed_right_bound() {
    if (GMYawEncoder.ecd_angle > init_yaw_pos + 1620 )
    {
        return 1;
    }
    else return 0;
}

int16_t gimbal_exceed_left_bound() {
    if (GMYawEncoder.ecd_angle < init_yaw_pos - 1620 )
    {
        return 1;
    }
    else return 0;
}

int16_t gimbal_exceed_upper_bound() {
    if (GMPitchEncoder.ecd_angle > init_yaw_pos + 19 * 45 )
    {
        return 1;
    }
    else return 0;
}

int16_t gimbal_exceed_lower_bound() {
    if (GMPitchEncoder.ecd_angle < init_yaw_pos )
    {
        return 1;
    }
    else return 0;
}

void gimbal_yaw_set(float target_pos)
{ //feel like this function is useless because it takes time to run
    while (float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, target_pos, 10) != 1) {
        pause(2);
        control_gimbal_yaw_pos(target_pos);
    }
}

int16_t chassis_follow()
{
    if ( float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 10) != 1) {
        gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
        float step = PID_output(&gimbal_reset_pid, 0);
        target_angle = current_angle + 10 * step;
        control_gimbal_yaw_pos(GMYawEncoder.ecd_angle - init_yaw_pos + step * 27);
        return 1;
    }
    else return 0;
}

void control_gimbal_yaw_pos_with_speed(int16_t ch2, int16_t input_speed) 
{
    int16_t ratio = 1;
    int16_t target_position = ratio * ch2 + init_yaw_pos;
    gimbal_pos_pid[0].current = GMYawEncoder.ecd_angle;
    int16_t target_speed = input_speed + PID_output2(&gimbal_pos_pid[0], target_position, init_yaw_pos + ratio * 660, init_yaw_pos - ratio * 660, 100, 30);
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    int16_t input = PID_output2(&gimbal_speed_pid[0], target_speed, 660, -660, 100, 15);
    Set_CM_Speed(CAN1, input, 0, 0, 0);
}

void control_gimbal_pos_with_speed(int16_t input_yaw_pos, int16_t input_pitch_pos, int16_t input_yaw_speed)
{
    int16_t target_position[2] = {input_yaw_pos, input_pitch_pos};
    gimbal_pos_pid[0].current = GMYawEncoder.ecd_angle;
    gimbal_pos_pid[1].current = GMPitchEncoder.ecd_angle;
    int16_t target_speed[2] ={0, 0};
    target_speed[0] = input_yaw_speed + PID_output2(&gimbal_pos_pid[0], target_position[0], 660, -660, 100, 30);
    target_speed[1] = PID_output2(&gimbal_pos_pid[1], target_position[1], 660, -600, 100, 30);
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    gimbal_speed_pid[1].current = GMPitchEncoder.filter_rate;
    int16_t input[2] = {0, 0};
    input[0] = PID_output2(&gimbal_speed_pid[0], target_speed[0], 660, -660, 100, 15);
    input[1] = PID_output2(&gimbal_speed_pid[1], target_speed[1], 660, -660, 100, 15);
    Set_CM_Speed(CAN1, input[0], input[1], 0, 0);
}

int16_t chassis_follow_with_control(int16_t input_yaw_speed, int16_t input_pitch_pos)
{
    if ( float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 10) != 1) {
        gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
        float step = PID_output(&gimbal_reset_pid, 0);
        debug = step;
        target_angle = current_angle + 10 * step;
        // control_gimbal_yaw_pos_with_speed((GMYawEncoder.ecd_angle - init_yaw_pos + step * 27), ch2);
        control_gimbal_pos_with_speed((GMYawEncoder.ecd_angle - init_yaw_pos + step * 27), input_pitch_pos, input_yaw_speed);
        return 1;
    }
    else return 0;
}

void buff_mode_gimbal_yaw_pos(int16_t index)
{
    target_angle = current_angle;
    if (index != -1)
    {
        control_gimbal_yaw_pos(-27 * buff_yaw_pos[index]);
    }
    else control_gimbal_yaw_pos(GMYawEncoder.ecd_angle - init_yaw_pos);
}

void buff_mode_gimbal_pos(int16_t index)
{
    target_angle = current_angle;
    if (index != -1)
    {
        control_gimbal_pos( -27 * buff_yaw_pos[index % 3], 19 * buff_pitch_pos[index / 3]);
    }
    else control_gimbal_pos(GMYawEncoder.ecd_angle - init_yaw_pos, GMPitchEncoder.ecd_angle - init_pitch_pos);
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


