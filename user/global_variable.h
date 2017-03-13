#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "PID_s.h"

extern s32 debug;
extern s32 target_angle;
extern s32 current_angle;
extern s32 last_angle;
extern int16_t result[4];
extern PID wheels_pos_pid[4];
extern PID wheels_speed_pid[4];
extern PID angle_pid;
/*
extern PID ang_vel_pid;
extern PID power_pid;
*/
extern PID buffer_pid;
extern PID gimbal_speed_pid[2];
extern PID gimbal_pos_pid[2];
extern PID gimbal_reset_pid;
extern u8 str[256];
extern float buffer_remain;
extern float init_yaw_pos;
extern float init_pitch_pos;
extern int16_t buff_yaw_pos[3];
extern int16_t buff_pitch_pos[3];

#endif
