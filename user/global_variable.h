#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "PID_s.h"
#include "customized_type.h"

extern int16_t int_debug;
extern int16_t int_debug2;
extern float float_debug;
extern s32 target_angle;
extern s32 current_angle;
extern s32 last_angle;
extern int16_t M_wheel_result[4];
extern PID wheels_pos_pid[4];
extern PID wheels_speed_pid[4];
extern PID angle_pid;
extern PID buffer_pid;
extern PID gimbal_speed_pid[2];
extern PID gimbal_pos_pid[2];
extern PID gimbal_reset_pid;
extern u8 str[256];
extern float init_yaw_pos;
extern float init_pitch_pos;//extern const float init_pitch_pos;
extern int16_t buff_yaw_pos[3];
extern int16_t buff_pitch_pos[3];
extern union u32ANDint16_t manual_buff_pos[18];
extern int16_t read_buff_pos[18];
extern uint8_t Chassis_Connected;
extern uint8_t Gimbal_Connected;
extern uint8_t DBUS_Connected;
extern uint8_t GUN_ENCODER_Connected;
extern int16_t chassis_ch2;
extern int16_t last_ch_input[4];
extern int16_t ch_input[4];
extern int16_t mouse_input[2];
extern int16_t last_mouse_input[2];
extern uint8_t gimbal_follow;
extern uint8_t chassis_already_auto_stop;
extern uint8_t buff_mode;
extern uint8_t gimbal_in_buff_pos;
extern uint8_t buff_pressed;
extern uint8_t clearing_ammo;
extern uint8_t fast_turning;
//const area
extern const float YAW_SPEED_TO_CHASSIS_CH2;
extern const float YAW_ANGLE_RATIO;
extern const float YAW_LEFT_BOUND_REAL_ANGLE;
extern const float YAW_RIGHT_BOUND_REAL_ANGLE;
extern const float YAW_LEFT_BOUND;
extern const float YAW_RIGHT_BOUND;
extern const float PITCH_ANGLE_RATIO;
extern const float PITCH_UPPER_BOUND;
extern const float PITCH_UPPER_BOUND_REAL_ANGLE;
extern const float GYRO_ANGLE_RATIO;
#endif
