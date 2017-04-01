
#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
#include "chassis_control.h"
#include "customized_function.h"

void send_to_chassis(int16_t wheel_speed_0, int16_t wheel_speed_1, int16_t wheel_speed_2, int16_t wheel_speed_3) {
    Set_CM_Speed(CAN2, wheel_speed_0, wheel_speed_1, wheel_speed_2, wheel_speed_3);
}

void M_wheel_analysis(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0, float ratio1, float ratio2, int16_t delta) {
    //potential bug exists
    //the data from gyro will accumulate and may be overflow?
    if (ch2 < 1 && ch2 > -1) {
        ch2 = delta;
        gimbal_follow = 1;
    }
    else
    {
        target_angle = current_angle;
        gimbal_follow = 0;
    }
    chassis_ch2 = ch2;
    ch0 *= ratio0;
    ch1 *= ratio1;
    ch2 *= ratio2;
    M_wheel_result[0] = ch1 + ch0 + ch2;
    M_wheel_result[1] = -(ch1 - ch0 - ch2);
    M_wheel_result[2] = -(ch1 + ch0 - ch2);
    M_wheel_result[3] = ch1 - ch0 + ch2;
}

void M_wheel_analysis_dancing(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0, float ratio1, float ratio2) {
    int16_t theta = -(GMYawEncoder.ecd_angle - init_yaw_pos) * GYRO_ANGLE_RATIO / YAW_ANGLE_RATIO;
    int16_t ch0_temp = ch0;
    int16_t ch1_temp = ch1;
    ch0 = ch1_temp * sin_val(theta) + ch0_temp * cos_val(theta);
    ch1 = ch1_temp * cos_val(theta) - ch0_temp * sin_val(theta);
    ch0 *= ratio0;
    ch1 *= ratio1;
    ch2 *= ratio2;
    M_wheel_result[0] = ch1 + ch0 + ch2;
    M_wheel_result[1] = -(ch1 - ch0 - ch2);
    M_wheel_result[2] = -(ch1 + ch0 - ch2);
    M_wheel_result[3] = ch1 - ch0 + ch2;
}

void update_wheel_pid()
{
    current_angle = output_angle;
    angle_pid.current = current_angle;
    buffer_pid.current = buffer_remain;
    wheels_speed_pid[0].current = CM1Encoder.filter_rate;
    wheels_speed_pid[1].current = CM2Encoder.filter_rate;
    wheels_speed_pid[2].current = CM3Encoder.filter_rate;
    wheels_speed_pid[3].current = CM4Encoder.filter_rate;
}

void update_wheel_pid_semi_closed_loop()
{
    current_angle = output_angle;
    angle_pid.current = current_angle;
    buffer_pid.current = buffer_remain;
    wheels_speed_semi_closed_pid[0].current = CM1Encoder.filter_rate;
    wheels_speed_semi_closed_pid[1].current = CM2Encoder.filter_rate;
    wheels_speed_semi_closed_pid[2].current = CM3Encoder.filter_rate;
    wheels_speed_semi_closed_pid[3].current = CM4Encoder.filter_rate;
}

float buffer_decay()
{
    float ratio = 1;
    if (buffer_remain < 60)
    {
        ratio = 1 - PID_output(&buffer_pid, 60);
        ratio = ratio > 0 ? ratio : -ratio;
    }
    return ratio;
}

void control_car(int16_t ch0, int16_t ch1, int16_t ch2, CarMode mode)
{
    if (mode == SEMI_CLOSED_LOOP)
    {
        update_wheel_pid_semi_closed_loop();
    }
    else update_wheel_pid();

    if (mode != NORMAL)
    {
        target_angle = current_angle;
        gimbal_follow = 0;
    }
		//ch2=0.5 的时候玄学参数非常完美
    if (mode == DANCING)
    {
        M_wheel_analysis_dancing(ch0, ch1, ch2, 0.5, 0.5, 0.5);
    }
    else M_wheel_analysis(ch0, ch1, ch2, 0.7, 1, 0.5, PID_output2(&angle_pid, target_angle, 800, -800, 30, -30));

    float ratio = buffer_decay();
    int16_t input[4] = {0, 0, 0, 0};
    int16_t target_speed[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
    {
        target_speed[i] = M_wheel_result[i];
        target_speed[i] *= ratio;
    }

    for (int i = 0; i < 4; i++) {

        switch (mode)
        {
            case OPEN_LOOP:
                PID_output2(&wheels_speed_pid[i], target_speed[i], 660, -660, 100, 15);
                input[i] = target_speed[i] * 3;
                break;
            case SEMI_CLOSED_LOOP:
                input[i] = PID_output2(&wheels_speed_semi_closed_pid[i], target_speed[i], 660, -660, 100, 15);
                break;
            default :
                input[i] = PID_output2(&wheels_speed_pid[i], target_speed[i], 660, -660, 100, 15);
                break;
        }
        
    }
    send_to_chassis(input[0], input[1], input[2], input[3]);

}
