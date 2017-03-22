#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "PID_s.h"
typedef union u32ANDint16_t{
	u32 flash;
	int16_t mem;
} u32ANDint16_t;
extern float debug;
extern s32 target_angle;
extern s32 current_angle;
extern int16_t M_wheel_result[4];
extern PID wheels_pos_pid[4];
extern PID wheels_speed_pid[4];
extern PID wheels_speed_semi_closed_pid[4];
extern PID angle_pid;
/*
extern PID ang_vel_pid;
extern PID power_pid;
*/

extern PID buffer_pid;
extern PID gimbal_speed_pid[2];
extern PID gimbal_pos_pid[2];
extern PID gimbal_reset_pid;
extern PID gimbal_relative_angle_pid;
extern u8 str[256];
extern float buffer_remain;
extern float init_yaw_pos;
extern float init_pitch_pos;
extern int16_t buff_yaw_pos[3];
extern int16_t buff_pitch_pos[3];
extern union u32ANDint16_t manual_buff_pos[18];
extern int16_t is_writing_flash;

#endif