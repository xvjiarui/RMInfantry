
#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
#include "power_control.h"

void buffer_call(void){
	buffer_remain = buffer_remain - (InfantryJudge.RealVoltage * InfantryJudge.RealCurrent - 80.0f) * 0.02f;
	if (buffer_remain<0.0f)
	{
		buffer_remain=0;
	}
	else if (buffer_remain>60.0f)
	{
		buffer_remain=60;
	}
}
