#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
#include "customized_function.h"

/*
void control_gimbal(int16_t ch2, int16_t ch3) {
    int16_t target_speed[2];
    target_speed[0]=ch2;
    target_speed[1]=ch3;
    gimbal_pid[0].current=GMYawEncoder.filter_rate;
    gimbal_pid[1].current=GMPitchEncoder.filter_rate;
    int16_t input[2];
    for (int i = 0; i < 2; ++i)
    {
        input[i]=PID_output(&gimbal_pid[i], target_speed[i]);
    }
    Set_Gimbal_Current(CAN2,input[0],0,0);
}
*/
// inpur relative position
// range (-5, 5)

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

int16_t gimbal_exceed_right_bound() {
    if (GMYawEncoder.ecd_angle > init_yaw_pos + 2430 )
    {
        return 1;
    }
    else return 0;
}

int16_t gimbal_exceed_left_bound() {
    if (GMYawEncoder.ecd_angle < init_yaw_pos - 2430)
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

void instant_stabilize_gimbal() {
    s16 gyro_angle_speed = gyro_get_vel();
    int16_t target_yaw_filter_rate = - gyro_angle_speed * 27 * 6144 / 3600 / 1000;
    debug = target_yaw_filter_rate;
    if ((target_yaw_filter_rate > 0 && gimbal_exceed_right_bound()) || (target_yaw_filter_rate < 0 && gimbal_exceed_left_bound()))
    {
        target_yaw_filter_rate = 0;
    }
    control_gimbal_yaw_speed(target_yaw_filter_rate);
}

int16_t chassis_follow()
{
    if ( float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 15) != 1) {
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

int16_t chassis_follow_with_control(int16_t ch2)
{
    if ( float_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 15) != 1) {
        gimbal_reset_pid.current = (GMYawEncoder.ecd_angle - init_yaw_pos);
        float step = PID_output(&gimbal_reset_pid, 0);
        target_angle = current_angle + 10 * step;
        control_gimbal_yaw_pos_with_speed((GMYawEncoder.ecd_angle - init_yaw_pos + step * 27), ch2);
        return 1;
    }
    else return 0;
}




