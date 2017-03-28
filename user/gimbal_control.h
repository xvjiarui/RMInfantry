#ifndef GIMBAL_CONTROL
#define GIMBAL_CONTROL

#include "global_variable.h"

int16_t pid_gimbal_yaw_pos(int16_t target_yaw_pos);
int16_t pid_gimbal_yaw_speed(int16_t target_yaw_speed);
int16_t pid_gimbal_yaw_pos_with_speed(int16_t target_yaw_pos, int16_t yaw_speed);
int16_t pid_gimbal_pitch_speed(int16_t target_pitch_speed);
int16_t pid_gimbal_pitch_pos(int16_t target_pitch_pos);
void send_to_gimbal(int16_t pid_yaw, int16_t pid_pitch);

void control_gimbal_yaw_pos(int16_t target_yaw_pos);
void control_gimbal_pos(int16_t target_yaw_pos, int16_t target_pitch_pos); // ch2 is yaw pos, ch3 is pitch pos
void control_gimbal(int16_t target_yaw_speed, int16_t target_pitch_pos);//ch2 is yaw speed, ch3 is pitch position
void control_gimbal_yaw_speed(int16_t ch2);
void control_gimbal_speed(int16_t ch2, int16_t ch3);//ch2 is yas speed, ch3 is pitch speed
void control_gimbal_pos_with_speed(int16_t ch2, int16_t ch3, int16_t yaw_input_speed);

int16_t gimbal_exceed_right_bound();
int16_t gimbal_exceed_left_bound();
int16_t gimbal_exceed_upper_bound();
int16_t gimbal_exceed_lower_bound();
int16_t gimbal_approach_right_bound();
int16_t gimbal_approach_left_bound();
int16_t gimbal_yaw_back();
int16_t chassis_follow();
void chassis_follow_with_control(int16_t input_yaw_speed, int16_t input_pitch_pos);
void chassis_gimbal_relative_angle_with_control(int16_t relative_angle, int16_t input_yaw_speed, int16_t input_pitch_pos);
void chassis_follow_pos(int16_t input_yaw_pos, int16_t input_pitch_pos);
void buff_mode_gimbal_pos(int16_t index);
void buff_switch();

//const area
#define YAW_ANGLE_RATIO 27
#define YAW_LEFT_BOUND (YAW_ANGLE_RATIO * 60)
#define YAW_RIGHT_BOUND (-YAW_ANGLE_RATIO * 60)
#define PITCH_ANGLE_RATIO 19
#define PITCH_UPPER_BOUND (PITCH_ANGLE_RATIO * 45)
#endif