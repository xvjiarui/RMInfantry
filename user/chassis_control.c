
#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
#include "chassis_control.h"

int16_t* control_dir(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2, int16_t delta){
	float theta=0;
	theta=(target_angle-current_angle)%3600/10;
	int16_t ch0_temp=ch0;
	int16_t ch1_temp=ch1;
	ch0=ch1_temp*sin(theta)+ch0_temp*cos(theta);
	ch1=ch1_temp*cos(theta)+ch0_temp*sin(theta);
	ch0 *= ratio0;
	ch1 *= ratio1;
	ch2 *= ratio2;
	result[0]=ch1+ch0+ch2;
	result[1]=-(ch1-ch0-ch2);
	result[2]=-(ch1+ch0-ch2);
	result[3]=ch1-ch0+ch2;
	return result;
}

int16_t* control_remoter(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2, int16_t delta){
	
	if(ch2<4 && ch2>-4){
		if(target_angle-current_angle>900){
			target_angle=current_angle+900;
		}
		ch2=delta;
	}
	else target_angle=output_angle;
	ch0 *= ratio0;
	ch1 *= ratio1;
	ch2 *= ratio2;
	result[0]=ch1+ch0+ch2;
	result[1]=-(ch1-ch0-ch2);
	result[2]=-(ch1+ch0-ch2);
	result[3]=ch1-ch0+ch2;
	return result;
}

void control_car(int16_t ch0, int16_t ch1, int16_t ch2){
	current_angle = output_angle;
	angle_pid.current=current_angle;
	buffer_pid.current=buffer_remain;
	//ang_vel_pid.current=curr_ang_vel;
	//power_pid.current=InfantryJudge.RealVoltage * InfantryJudge.RealCurrent;
	//int16_t* target_speed=control_remoter(ch0,ch1,ch2,1,1,1,0);
	int16_t* target_speed;
	if(DBUS_ReceiveData.rc.switch_left==2){
		target_speed=control_dir(ch0,ch1,100,0.5,0.5,0.5,0);
		target_angle=current_angle;
	}
	else if(DBUS_ReceiveData.rc.switch_right==3){
		target_speed=control_remoter(ch0,ch1,ch2,1,1,1,PID_output2(&angle_pid,target_angle,800,-800,30,-30));
	}
	else target_speed=control_remoter(ch0,ch1,ch2,1,1,1,PID_output(&angle_pid,target_angle));
	//target_speed=control_remoter(ch0,ch1,ch2,1,1,0.01,PID_output(ang_vel_pid, PID_output(angle_pid,target_angle)));
	wheels_speed_pid[0].current=CM1Encoder.filter_rate;
	wheels_speed_pid[1].current=CM2Encoder.filter_rate;
	wheels_speed_pid[2].current=CM3Encoder.filter_rate;
	wheels_speed_pid[3].current=CM4Encoder.filter_rate;
	int16_t input[4];
	for (int i = 0; i < 4; i++){
		float ratio=1;
		if (buffer_remain<60)
		{
			ratio = 1 - PID_output(&buffer_pid, 60);
			ratio= ratio>0? ratio:-ratio;
		}
		target_speed[i] *= ratio;	
		input[i]=PID_output2(&wheels_speed_pid[i],target_speed[i],660, -660, 100, 15);
		//sprintf(str," %d ",input[i]);
		//DataMonitor_Send(str,sizeof(str));
		
	}
	Set_CM_Speed(CAN2, input[0],input[1], input[2], input[3]);
}
