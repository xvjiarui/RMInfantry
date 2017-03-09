#include "function_list.h"

void pause(u32 ms){
	u32 ticks = get_ms_ticks();
	while ((get_ms_ticks() - ticks) % ms != 0);
	return;
}

void limit_int_range(int16_t* num, int16_t upper_bound, int16_t lower_bound){
	if (*num > upper_bound)
		*num = upper_bound;
	else if (*num < lower_bound)
		*num = lower_bound;
}

int16_t float_equal(float x, float y, float delta)
{
    if (x - y < delta && y - x < delta)
    {
        return 1;
    }
    else return 0;
}