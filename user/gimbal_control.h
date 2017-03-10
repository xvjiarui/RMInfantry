#include "global_variable.h"

//void control_gimbal(int16_t ch2, int16_t ch3);
void control_gimbal_yaw_pos(int16_t ch2);
void control_gimbal_yaw_speed(int16_t ch2);
int16_t gimbal_exceed_right_bound();
int16_t gimbal_exceed_left_bound();
void gimbal_yaw_set(float target_pos);
void instant_stabilize_gimbal();
int16_t chassis_follow();