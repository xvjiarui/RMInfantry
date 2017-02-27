#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
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
void control_gimbal_yaw_pos(int16_t ch2) {
    // init_yaw_pos = GMYawEncoder.ecd_angle;
    int16_t ratio = 2;
    int16_t target_position = ratio * ch2 + init_yaw_pos;
	gimbal_pos_pid[0].current = GMYawEncoder.ecd_angle;
    //gimbal_pos_pid[0].current = GMYawEncoder.ecd_angle - init_yaw_pos;
    int16_t target_speed = PID_output2(&gimbal_pos_pid[0], target_position, init_yaw_pos + ratio * 660, init_yaw_pos - ratio * 660, 100, 30);
    // int16_t target_speed = ch3;
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    //int16_t input = PID_output(&wheels_speed_pid[0],target_speed);
    int16_t input = PID_output2(&gimbal_speed_pid[0], target_speed, 660, -660, 100, 15);
    Set_CM_Speed(CAN1, input, 0, 0, 0);
}
void control_gimbal_yaw_speed(int16_t ch2) {
    //int16_t target_speed = PID_output2(&gimbal_pos_pid[0], target_position, init_yaw_pos + ratio * 660, init_yaw_pos - ratio * 660, 100, 30);
    int16_t target_speed = ch2;
    gimbal_speed_pid[0].current = GMYawEncoder.filter_rate;
    //int16_t input = PID_output(&wheels_speed_pid[0],target_speed);
    int16_t input = PID_output2(&gimbal_speed_pid[0], target_speed, 660, -660, 100, 15);
    Set_CM_Speed(CAN1, input, 0, 0, 0);
}

int16_t exceed_range_right() {
    if (GMYawEncoder.ecd_angle > init_yaw_pos + 2430 )
    {
        return 1;
    }
    else return 0;
}

int16_t exceed_range_left() {
    if (GMYawEncoder.ecd_angle < init_yaw_pos - 2430)
    {
        return 1;
    }
    else return 0;
}

int16_t yaw_pos_equal(float x, float y, float delta)
{
    if (x - y < delta && y - x < delta)
    {
        return 1;
    }
    else return 0;
}

void gimbal_yaw_set(float target_pos)
{
    u32 ticks = (u32) -1;
    while (yaw_pos_equal(GMYawEncoder.ecd_angle - init_yaw_pos, target_pos, 10) != 1) {
        if (ticks != get_ms_ticks())
        {
            ticks = get_ms_ticks();
            if (ticks % 3 == 0)
            {
                 control_gimbal_yaw_pos(target_pos);
            }
        } 
        // control_gimbal_yaw_pos(target_pos);
        // ticks = get_ms_ticks();
        // while(get_ms_ticks() < ticks + 2); // pause for 10ms
        // tft_prints(1, 5, "init:%f", init_yaw_pos);
        // tft_prints(1, 6, "curr:%f", GMYawEncoder.ecd_angle);
        // tft_prints(1, 7, "5:%d 6:%d", GMYawEncoder.filter_rate, GMPitchEncoder.filter_rate);
        // // tft_prints(1, 8, "buffer: %f", buffer_remain);
        // tft_prints(1, 8, "equal:%d", yaw_pos_equal(GMYawEncoder.ecd_angle - init_yaw_pos, 0, 100));
        // tft_update();
        // if (DBUS_ReceiveData.mouse.press_left){
        //     while(1);
        // }
    }
}



