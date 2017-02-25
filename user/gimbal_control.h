#include "global_variable.h"

//void control_gimbal(int16_t ch2, int16_t ch3);
void control_gimbal_yaw_pos(int16_t ch2);
void control_gimbal_yaw_speed(int16_t ch2);
int16_t exceed_range_right();
int16_t exceed_range_left();
void gimbal_yaw_set(float target_pos);
int16_t yaw_pos_equal(float x, float y, float delta);
