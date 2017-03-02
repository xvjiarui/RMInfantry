#include "function_list.h"

void pause(int ms){
	u32 ticks = get_ms_ticks();
	while (get_ms_ticks() - ticks < ms);
	return;
}