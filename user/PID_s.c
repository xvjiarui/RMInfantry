#include "PID_s.h"

void PID_init(PID* pid, float Kp_val, float Ki_val, float Kd_val, float limit){
	pid->Kp=Kp_val;
	pid->Ki=Ki_val;
	pid->Kd=Kd_val;
	pid->limit=limit;
	pid->target=0;
	pid->current=0;
	pid->last_err=0;
	pid->p=0;
	pid->i=0;
	pid->d=0;
	memset(pid->err_array, 0, sizeof(pid->err_array));
	pid->err_index = 0;
}
float PID_output(PID* pid, float target_val){
	pid->target=target_val;
	pid->p=pid->target - pid->current;
	pid->i+=pid->p;
	pid->d=pid->p - pid->last_err;
	pid->last_err=pid->p;
	pid->output=pid->Kp * pid->p + pid->Ki * pid->i + pid->Kd * pid->d;
	pid->output = pid->output>pid->limit? pid->limit:pid->output;
	pid->output = pid->output<(-pid->limit)? -pid->limit:pid->output;
	return pid->output;
}
float PID_output2(PID* pid, float target_val, float umax, float umin, float emax, float emin){
	float index=1;
	pid->target=target_val;
	pid->p=pid->target - pid->current;
	if(pid->current>umax){
		if(fabs(pid->p)>emax){
			index=0;
		}
		else if(fabs(pid->p)<emin){
			index=1;
			pid->i += pid->p < 0? pid->p:0;
		}
		else{
			index=(emax-fabs(pid->p))/(emax-emin);
			pid->i += pid->p < 0? pid->p:0;
		}
	}
	else if(pid->current<umin){
		if(fabs(pid->p)>emax){
			index=0;
		}
		else if(fabs(pid->p)<emin){
			index=1;
			pid->i += pid->p > 0? pid->p:0;
		}
		else{
			index=(emax-fabs(pid->p))/(emax-emin);
			pid->i += pid->p > 0? pid->p:0;
		}
	}
	else {
		if(fabs(pid->p)>emax){
			index=0;
		}
		else if(fabs(pid->p)<emin){
			index=1;
			pid->i += pid->p;
		}
		else{
			index=(emax-fabs(pid->p))/(emax-emin);
			pid->i += pid->p;
		}
	}
	pid->d=pid->p - pid->last_err;
	pid->last_err=pid->p;
	pid->output=pid->Kp * pid->p + index * pid->Ki * pid->i + pid->Kd * pid->d;
	pid->output = pid->output>pid->limit? pid->limit:pid->output;
	pid->output = pid->output<(-pid->limit)? -pid->limit:pid->output;
	return pid->output;
}
float PID_output3(PID* pid, float target_val, float pmax, float pmin, float imax, float imin, float decay_factor)
{
	pid->target=target_val;
	pid->p=pid->target - pid->current;
	pid->p = (pid->p > pmax)? pmax:pid->p;
	pid->p = (pid->p < pmin)? pmin:pid->p;
	pid->i = pid->i*decay_factor + pid->p;
	pid->i = (pid->i > imax)? imax:pid->i;
	pid->i = (pid->i < imin)? imin:pid->i;
	pid->d=pid->p - pid->last_err;
	pid->last_err=pid->p;
	pid->output=pid->Kp * pid->p + pid->Ki * pid->i + pid->Kd * pid->d;
	pid->output = pid->output>pid->limit? pid->limit:pid->output;
	pid->output = pid->output<(-pid->limit)? -pid->limit:pid->output;
	return pid->output;
}

float PID_output4(PID* pid, float target_val)
{
	pid->target=target_val;
	pid->p=pid->target - pid->current;
	pid->err_array[pid->err_index]=pid->p;
	pid->err_index++;
	pid->err_index%=10;
	pid->i=0;
	for (int index = 0; index < 10; ++index)
	{
		pid->i+=pid->err_array[index];
	}
	pid->d=pid->p - pid->last_err;
	pid->last_err=pid->p;
	pid->output=pid->Kp * pid->p + pid->Ki * pid->i + pid->Kd * pid->d;
	pid->output = pid->output>pid->limit? pid->limit:pid->output;
	pid->output = pid->output<(-pid->limit)? -pid->limit:pid->output;
	return pid->output;
}
