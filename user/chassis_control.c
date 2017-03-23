
#include "function_list.h"
#include "PID_s.h"
#include "global_variable.h"
#include "chassis_control.h"

int16_t* control_dir(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2) {
    float theta = -(GMYawEncoder.ecd_angle - init_yaw_pos)*10/27;
    int16_t ch0_temp=ch0;
    int16_t ch1_temp=ch1;
    ch0=ch1_temp*sin_val(theta)+ch0_temp*cos_val(theta);
    ch1=ch1_temp*cos_val(theta)+ch0_temp*sin_val(theta);
    ch0 *= ratio0;
    ch1 *= ratio1;
    ch2 *= ratio2;
    M_wheel_result[0]=ch1+ch0+ch2;
    M_wheel_result[1]=-(ch1-ch0-ch2);
    M_wheel_result[2]=-(ch1+ch0-ch2);
    M_wheel_result[3]=ch1-ch0+ch2;
    return M_wheel_result;
}

int16_t* control_remoter(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2, int16_t delta) {
    //potential bug exists
    //the data from gyro will accumulate and may be overflow?
    if(ch2<1 && ch2>-1) {
        ch2=delta;
    }
    else target_angle=output_angle;
    ch0 *= ratio0;
    ch1 *= ratio1;
    ch2 *= ratio2;
    M_wheel_result[0]=ch1+ch0+ch2;
    M_wheel_result[1]=-(ch1-ch0-ch2);
    M_wheel_result[2]=-(ch1+ch0-ch2);
    M_wheel_result[3]=ch1-ch0+ch2;
    return M_wheel_result;
}

void control_car(int16_t ch0, int16_t ch1, int16_t ch2) {
    current_angle = output_angle;
    angle_pid.current=current_angle;
    buffer_pid.current=buffer_remain;
    int16_t* target_speed;
    // should be take care
    target_speed = control_remoter(ch0,ch1,ch2,1,1,0.5,PID_output2(&angle_pid,target_angle,800,-800,30,-30));
        
    wheels_speed_pid[0].current=CM1Encoder.filter_rate;
    wheels_speed_pid[1].current=CM2Encoder.filter_rate;
    wheels_speed_pid[2].current=CM3Encoder.filter_rate;
    wheels_speed_pid[3].current=CM4Encoder.filter_rate;
    int16_t input[4];
    float ratio=1;
    if (buffer_remain<60)
    {
        ratio = 1 - PID_output(&buffer_pid, 60);
        ratio= ratio>0? ratio:-ratio;
    }
    for (int i = 0; i < 4; i++) {
       
        target_speed[i] *= ratio;
        input[i]=PID_output2(&wheels_speed_pid[i],target_speed[i],660, -660, 100, 15);
    }
    Set_CM_Speed(CAN2, input[0],input[1], input[2], input[3]);
}
void control_car_open_loop(int16_t ch0, int16_t ch1, int16_t ch2)
{
    current_angle = output_angle;
    target_angle = current_angle;
    angle_pid.current = current_angle;
    buffer_pid.current=buffer_remain;
    int16_t* target_speed;
    // should be take care
    target_speed = control_remoter(ch0,ch1,ch2,1,1,0.5,PID_output2(&angle_pid,target_angle,800,-800,30,-30));    
    wheels_speed_pid[0].current=CM1Encoder.filter_rate;
    wheels_speed_pid[1].current=CM2Encoder.filter_rate;
    wheels_speed_pid[2].current=CM3Encoder.filter_rate;
    wheels_speed_pid[3].current=CM4Encoder.filter_rate;
    int16_t input[4];
    float ratio=1;
    if (buffer_remain<60)
    {
        ratio = 1 - PID_output(&buffer_pid, 60);
        ratio= ratio>0? ratio:-ratio;
    }
    for (int i = 0; i < 4; i++) {
        PID_output2(&wheels_speed_pid[i],wheels_speed_pid[i].current,660, -660, 100, 15);
        target_speed[i] *= ratio;
        input[i] = target_speed[i] * 3;
    }
    Set_CM_Speed(CAN2, input[0],input[1], input[2], input[3]);
}

void control_car_semi_closed_loop(int16_t ch0, int16_t ch1, int16_t ch2) {
    current_angle = output_angle;
    angle_pid.current=current_angle;
    buffer_pid.current=buffer_remain;
    int16_t* target_speed;
    // should be take care
    target_speed = control_remoter(ch0,ch1,ch2,1,1,0.5,PID_output2(&angle_pid,target_angle,800,-800,30,-30));
        
    wheels_speed_semi_closed_pid[0].current=CM1Encoder.filter_rate;
    wheels_speed_semi_closed_pid[1].current=CM2Encoder.filter_rate;
    wheels_speed_semi_closed_pid[2].current=CM3Encoder.filter_rate;
    wheels_speed_semi_closed_pid[3].current=CM4Encoder.filter_rate;
    int16_t input[4];
    float ratio=1;
    if (buffer_remain<60)
    {
        ratio = 1 - PID_output(&buffer_pid, 60);
        ratio= ratio>0? ratio:-ratio;
    }
    for (int i = 0; i < 4; i++) {
       
        target_speed[i] *= ratio;
        input[i]=PID_output2(&wheels_speed_semi_closed_pid[i],target_speed[i],660, -660, 100, 15);
    }
    Set_CM_Speed(CAN2, input[0],input[1], input[2], input[3]);
}


