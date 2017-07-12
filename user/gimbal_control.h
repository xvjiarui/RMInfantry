#ifndef GIMBAL_CONTROL
#define GIMBAL_CONTROL

#include "global_variable.h"
#include "PID_s.h"
#include "customized_type.h"

#ifndef GIMBAL_CONTROL_FILE
 	#define GIMBAL_CONTROL_EXT extern
 #else 
	#define GIMBAL_CONTROL_EXT 
#endif

GIMBAL_CONTROL_EXT PID gimbal_speed_pid[2];
GIMBAL_CONTROL_EXT PID gimbal_pos_pid[2];
GIMBAL_CONTROL_EXT PID gimbal_reset_pid;
GIMBAL_CONTROL_EXT PID gun_driver_speed_pid;
GIMBAL_CONTROL_EXT PID gun_driver_pos_pid;
GIMBAL_CONTROL_EXT PID mouse_input_pid;
GIMBAL_CONTROL_EXT float init_yaw_pos;
GIMBAL_CONTROL_EXT float init_pitch_pos;//extern const float init_pitch_pos;
GIMBAL_CONTROL_EXT uint8_t gimbal_follow;
GIMBAL_CONTROL_EXT uint8_t buff_mode;
GIMBAL_CONTROL_EXT uint8_t gimbal_in_buff_pos;
GIMBAL_CONTROL_EXT uint8_t buff_pressed;
GIMBAL_CONTROL_EXT uint8_t clearing_ammo;
GIMBAL_CONTROL_EXT union u32ANDint16_t manual_buff_pos[18];
GIMBAL_CONTROL_EXT int16_t read_buff_pos[18];
GIMBAL_CONTROL_EXT int16_t GUN_DriverInput;
GIMBAL_CONTROL_EXT float GUN_TargetPos;
GIMBAL_CONTROL_EXT int16_t GUN_Direction;
GIMBAL_CONTROL_EXT uint8_t shootRune;

void gimbal_control_init(void);

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
uint8_t gimbal_check_pos(int16_t target_yaw_pos, int16_t target_pitch_pos);
int16_t gimbal_yaw_back();
int16_t gimbal_yaw_back_angle(float angle);
int16_t chassis_follow();
void control_gimbal_with_chassis_following(int16_t input_yaw_speed, int16_t input_pitch_pos);
void control_gimbal_with_chassis_following_angle(int16_t input_yaw_speed, int16_t input_pitch_pos, float angle);
void chassis_gimbal_relative_angle_with_control(int16_t relative_angle, int16_t input_yaw_speed, int16_t input_pitch_pos);
void chassis_follow_pos(int16_t input_yaw_pos, int16_t input_pitch_pos);
// void buff_mode_gimbal_pos(int16_t index);
// void buff_switch();
// uint8_t gimbal_check_in_buff_pos(int16_t status, uint8_t pressed);


#endif