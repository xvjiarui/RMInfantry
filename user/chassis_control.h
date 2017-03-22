#include "function_list.h"


int16_t* control_remoter(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2, int16_t delta);
void buffer_call(void);
void control_car(int16_t ch0, int16_t ch1, int16_t ch2);
void control_car_open_loop(int16_t ch0, int16_t ch1, int16_t ch2);
void control_car_semi_closed_loop(int16_t ch0, int16_t ch1, int16_t ch2);

