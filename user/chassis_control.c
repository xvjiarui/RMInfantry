
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
    buffer_pid.current = InfantryJudge.RemainBuffer;
    wheels_speed_pid[0].current = CM1Encoder.filter_rate;
    wheels_speed_pid[1].current = CM2Encoder.filter_rate;
    wheels_speed_pid[2].current = CM3Encoder.filter_rate;
    wheels_speed_pid[3].current = CM4Encoder.filter_rate;
}

int16_t get_wheel_filter_rate(uint8_t index)
{
    switch(index)
    {
        case 0:
            return CM1Encoder.filter_rate;
        break;
        case 1:
            return CM2Encoder.filter_rate;
        break;
        case 2:
            return CM3Encoder.filter_rate;
        break;
        case 3:
            return CM4Encoder.filter_rate;
        break;
    }
}


float buffer_decay()
{
    float ratio = 0;
    // ratio = PID_output3(&buffer_pid, 60, 100, -100, 100, -100, 0.65);
    // ratio = PID_output4(&buffer_pid, 60);
    ratio = PID_UpdateValue(&buffer_pid, 60, InfantryJudge.RemainBuffer);
    ratio = ratio > 0 ? ratio : 0;
    ratio = 1- ratio;
    return ratio;
}

void control_car(int16_t ch0, int16_t ch1, int16_t ch2, CarMode mode)
{

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
    // else M_wheel_analysis(ch0, ch1, ch2, 1, 1, 0.5, PID_output2(&angle_pid, target_angle, 800, -800, 30, -30));
    else M_wheel_analysis(ch0, ch1, ch2, 1, 1, 0.5, PID_UpdateValue(&angle_pid, target_angle, current_angle));

    static float ratio = 1;
    if (InfantryJudge.Updated)
    {
        ratio = buffer_decay();
        InfantryJudge.Updated = 0;
    }
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
                PID_UpdateValue(&wheels_speed_pid[i], wheels_speed_pid[i].current, get_wheel_filter_rate(i));
                input[i] = target_speed[i] * 3;
                break;
            default :
                // input[i] = PID_output2(&wheels_speed_pid[i], target_speed[i], 990, -990, 100, 15);
                input[i] = PID_UpdateValue(&wheels_speed_pid[i], target_speed[i], get_wheel_filter_rate(i));
                break;
        }
        
    }
    send_to_chassis(input[0], input[1], input[2], input[3]);

}

extern volatile u32 ticks_msimg;
void chassis_SetMotion(void)
{
    // uint8_t rotateRight = 0
    // uint8_t rotateLeft = 0;
    // uint8_t rotateRightPress = 0;
    // uint8_t rotateLeftPress = 0;
    static int32_t rightLastTick = 0;
    static int32_t leftLastTick = 0;
    static int32_t hartLastTick = 0;
    int32_t time_interval = 500;
    int16_t rotateAngle = 900;

    uint8_t rightNow = DBUS_CheckPushNow(KEY_E);
    uint8_t rightLast = DBUS_CheckPushLast(KEY_E);
    uint8_t rotateRightPress = rightNow && !rightLast;
    uint8_t rotateRight = rotateRightPress && (ticks_msimg - rightLastTick > time_interval);

    uint8_t leftNow = DBUS_CheckPushNow(KEY_Q);
    uint8_t leftLast = DBUS_CheckPushLast(KEY_Q);
    uint8_t rotateLeftPress = leftNow && !leftLast;
    uint8_t rotateLeft = rotateLeftPress && (ticks_msimg - leftLastTick > time_interval);

    uint8_t hartNow = DBUS_CheckPushNow(KEY_X);
    uint8_t hartLast = DBUS_CheckPushLast(KEY_X);
    uint8_t rotateHartPress = hartNow && !hartLast;
    uint8_t rotateHart = rotateHartPress && (ticks_msimg - hartLastTick > time_interval);

    if (rotateHart)
    {
        switch (InfantryJudge.LastHartID)
        {
            case 1:
                target_angle -= rotateAngle;
                break;
            case 2: 
                target_angle -= 2 * rotateAngle;
                break;
            case 3: 
                target_angle += rotateAngle;
                break;
        }
        hartLastTick = ticks_msimg;
        fast_turning = 1;
    }

    if (rotateRight)
    {
        target_angle += rotateAngle;
        rightLastTick = ticks_msimg;
        fast_turning = 1;
    }

    if (rotateLeft)
    {
        target_angle -= rotateAngle;
        leftLastTick = ticks_msimg;
        fast_turning = 1;
    }

    if (!rotateRight && !rotateLeft && !rotateHart && (ticks_msimg - rightLastTick > time_interval) && (ticks_msimg - leftLastTick > time_interval) && (ticks_msimg - hartLastTick > time_interval))
    {
        fast_turning = 0;
    }
}
