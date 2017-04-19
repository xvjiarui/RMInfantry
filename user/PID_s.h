#ifndef PID_S_H
#define PID_S_H

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float target;
	float current;
	float last_err;
	float output;
	float p;
	float i;
	float d;
	float limit;
	float err_array[10];
	char err_index;
}PID;
void PID_init(PID* pid, float Kp_val, float Ki_val, float Kd_val, float limit);
float PID_output(PID* pid, float target_val);
float PID_output2(PID* pid, float target_val, float umax, float umin, float emax, float emin);
float PID_output3(PID* pid, float target_val, float pmax, float pmin, float imax, float imin, float decay_factor);
float PID_output4(PID* pid, float target_val);

#endif
