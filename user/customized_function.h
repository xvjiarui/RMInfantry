#ifndef CUSTOMIZED_FUNCTION_H
#define CUSTOMIZED_FUNCTION_H

void pause(int ms);
void limit_int_range(int16_t* num, int16_t upper_bound, int16_t lower_bound);
void limit_float_range(float* num, float upper_bound, float lower_bound);
int16_t float_equal(float x, float y, float delta);
void PID_init_all();
// sin and cos function 
// input range -3600 to 3600
float sin_val(int16_t theta);
float cos_val(int16_t theta);
void input_init_all();

#endif