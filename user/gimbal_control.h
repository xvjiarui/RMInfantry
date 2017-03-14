#include "global_variable.h"

void control_gimbal(int16_t ch2, int16_t ch3);//ch2 is yaw speed, ch3 is pitch position
void control_gimbal_yaw_pos(int16_t ch2);
void control_gimbal_yaw_speed(int16_t ch2);
void control_gimbal_pos(int16_t ch2, int16_t ch3); // ch2 is yaw pos, ch3 is pitch pos
void control_gimbal_speed(int16_t ch2, int16_t ch3);//ch2 is yas speed, ch3 is pitch speed
int16_t gimbal_exceed_right_bound();
int16_t gimbal_exceed_left_bound();
int16_t gimbal_exceed_upper_bound();
int16_t gimbal_exceed_lower_bound();
void gimbal_yaw_set(float target_pos);
int16_t chassis_follow();
void control_gimbal_yaw_pos_with_speed(int16_t ch2, int16_t input_speed);
void control_gimbal_pos_with_speed(int16_t ch2, int16_t ch3, int16_t yaw_input_speed);
int16_t chassis_follow_with_control(int16_t input_yaw_speed, int16_t input_pitch_pos);
void buff_mode_gimbal_yaw_pos(int16_t index);
void buff_mode_gimbal_pos(int16_t index);
void buff_switch();