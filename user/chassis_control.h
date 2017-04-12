#include "function_list.h"
#include "customized_type.h"

int16_t* control_remoter(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2, int16_t delta);
void control_car(int16_t ch0, int16_t ch1, int16_t ch2, CarMode mode);
void chassis_SetMotion(void);
