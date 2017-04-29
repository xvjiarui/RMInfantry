#include "function_list.h"
#include "customized_type.h"
#include "PID_s.h"
#ifndef CHASSIS_CONTROL_FILE
    #define CHASSIS_CONTROL_EXT extern
#else
    #define CHASSIS_CONTROL_EXT 
#endif

CHASSIS_CONTROL_EXT s32 target_angle;
CHASSIS_CONTROL_EXT s32 current_angle;
CHASSIS_CONTROL_EXT s32 last_angle;
CHASSIS_CONTROL_EXT int16_t M_wheel_result[4];
CHASSIS_CONTROL_EXT PID wheels_pos_pid[4];
CHASSIS_CONTROL_EXT PID wheels_speed_pid[4];
CHASSIS_CONTROL_EXT PID angle_pid;
CHASSIS_CONTROL_EXT PID buffer_pid;
CHASSIS_CONTROL_EXT uint8_t fast_turning;
CHASSIS_CONTROL_EXT uint8_t chassis_already_auto_stop;

void chassis_control_init(void);

int16_t* control_remoter(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2, int16_t delta);
void control_car(int16_t ch0, int16_t ch1, int16_t ch2, CarMode mode);
void chassis_SetMotion(void);
