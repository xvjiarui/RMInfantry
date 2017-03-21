#include "function_list.h"
#include "global_variable.h"

void pause(u32 ms){
	u32 ticks = get_ms_ticks();
	while (get_ms_ticks() == ticks);
	while ((get_ms_ticks() - ticks) % ms != 0);
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

void PID_init_all()
{
	for (int i = 0; i < 4; i++) {
			//PID_init(&wheels_pos_pid[i],130, 0.004, 0, 5000);//30 0.001 5 65 0.001 5
			PID_init(&wheels_pos_pid[i], 0.15, 0, 0, 20000);
			PID_init(&wheels_speed_pid[i], 80, 5, 100, 20000); //0.00001
	}
	PID_init(&gimbal_speed_pid[0], 80, 5, 100, 20000);
	PID_init(&gimbal_speed_pid[1], 80, 5, 100, 20000);
	PID_init(&gimbal_pos_pid[0], 0.15, 0, 0, 20000);
	PID_init(&gimbal_pos_pid[1], 0.35, 0, 0, 20000);
	PID_init(&angle_pid, 4, 0, 0, 330); //4 is best
	PID_init(&buffer_pid, 0.02, 0, 0, 60);
	PID_init(&gimbal_reset_pid, 0.02, 0, 0, 50);
	PID_init(&gimbal_relative_angle_pid, 0.02, 0, 0, 50);
}