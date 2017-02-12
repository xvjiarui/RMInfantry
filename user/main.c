#include "main.h"
#include "function_list.h"
#include "PID_s.h"
#include "power_control.h"
#include "chassis_control.h"
#include "gimbal_control.h"
static u32 ticks_msimg = (u32)-1;

void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,BLACK,GREEN,GREEN);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	Quad_Encoder_Configuration();
	Encoder_Start1();
	Encoder_Start2();
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
}

s32 target_angle=0;
s32 current_angle=0;
int16_t result[4]={0,0,0,0};
//int16_t result[4]={0,0,0,0};
PID wheels_pos_pid[4];
PID wheels_speed_pid[4];
PID angle_pid;
PID ang_vel_pid;
PID power_pid;
PID buffer_pid;
PID gimbal_pid[2];
u8 str[256];
float buffer_remain;
float init_pitch_pos;
int main(void)
{	
// u8 dum[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '\n'};
	init();
	for (int i = 0; i < 4; i++){
		//PID_init(&wheels_pos_pid[i],130, 0.004, 0, 5000);//30 0.001 5 65 0.001 5
		PID_init(&wheels_pos_pid[i],0.15, 0, 0, 20000);
		PID_init(&wheels_speed_pid[i],80,5,100,20000);//0.00001
	}
	PID_init(&gimbal_pid[0], 50, 0, 0,5000);
	PID_init(&gimbal_pid[1], 1, 0, 0,5000);
	PID_init(&angle_pid,3,0,0,660);//5 20
	//PID_init(&ang_vel_pid,0.01,0,0,1000);
	//PID_init(&power_pid,0.1,0,0,100);
	PID_init(&buffer_pid,0.02,0,0,60);
	int16_t ch_input[4]={0,0,0,0};
	buffer_remain=60;
	init_pitch_pos=CM3Encoder.ecd_angle;
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			//buzzer_check();			
			
			//if (ticks_msimg%500==0)
				//DataMonitor_Send(dum, sizeof(dum));
			
			//TODO: nothing:-)
			if (ticks_msimg%20==0)
			{
				buffer_call();
			}
			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				tft_prints(0,2,"Infantry V1.1");
				tft_clear();
				//sprintf(str,"%d",i);
				tft_prints(1,2,"rc0:%d rc1:%d",DBUS_ReceiveData.rc.ch0,DBUS_ReceiveData.rc.ch1);
				tft_prints(1,3,"rc2:%d rc3:%d",DBUS_ReceiveData.rc.ch2,DBUS_ReceiveData.rc.ch3);
				tft_prints(1,4,"Pr:%f",InfantryJudge.RealCurrent * InfantryJudge.RealVoltage);
				//tft_prints(1,2,"rc0:%d",DBUS_ReceiveData.rc.switch_right);
				//tft_prints(1,3,"pidav:%d",pid_output_angle_speed);
				//tft_prints(1,4,"cav:%d",curr_ang_vel);
				// tft_prints(1,5,"1:%d 2:%d",CM1Encoder.filter_rate,CM2Encoder.filter_rate);
				// tft_prints(1,6,"4:%d 3:%d",CM4Encoder.filter_rate,CM3Encoder.filter_rate);
				// tft_prints(1,5,"1:%d 2:%d",(int16_t) (wheels_pos_pid[0].target - wheels_pos_pid[0].current),(int16_t) (wheels_pos_pid[1].target - wheels_pos_pid[1].current));
				// tft_prints(1,6,"4:%d 3:%d",(int16_t) (wheels_pos_pid[3].target - wheels_pos_pid[3].current),(int16_t) (wheels_pos_pid[2].target - wheels_pos_pid[2].current));
				tft_prints(1,5,"cur:%f",wheels_speed_pid[0].current);
				tft_prints(1,6,"err:%f",wheels_speed_pid[0].target - wheels_speed_pid[0].current);
				tft_prints(1,7,"5:%d 6:%d",GMYawEncoder.filter_rate,GMPitchEncoder.filter_rate);
				//tft_prints(1,7,"0:%d 1:%d 2%d",ch_input[0],ch_input[1],ch_input[2]);
				//tft_prints(1,7,"mouse: %d %d",DBUS_ReceiveData.mouse.x, DBUS_CheckPush(KEY_W));
				tft_prints(1,8,"buffer: %f", buffer_remain);
				// tft_prints(1,9,"cur:%d",current_angle);
				// tft_prints(1,10,"tar:%d",target_angle);
				// tft_prints(1,11,"pid:%f",yaw_pid_output_angle);
				tft_prints(1,9,"cur:%f",init_pitch_pos);
				tft_prints(1,10,"tar:%f",init_pitch_pos + DBUS_ReceiveData.rc.ch3);
				tft_prints(1,11,"ecd:%f",CM1Encoder.ecd_angle);
				tft_update();
				LED_blink(LED1);
			}
			if(DBUS_ReceiveData.rc.switch_left==1){
				control_car(0,0,0);
			}
			else if (DBUS_ReceiveData.rc.switch_right==3)
			{	int16_t ch_changes[4];
				ch_input[0]=LASTDBUS_ReceiveData.rc.ch0;
				ch_input[1]=LASTDBUS_ReceiveData.rc.ch1;
				ch_input[2]=LASTDBUS_ReceiveData.rc.ch2;
				ch_input[3]=LASTDBUS_ReceiveData.rc.ch3;
				ch_changes[0]=DBUS_ReceiveData.rc.ch0 - LASTDBUS_ReceiveData.rc.ch0;
				ch_changes[1]=DBUS_ReceiveData.rc.ch1 - LASTDBUS_ReceiveData.rc.ch1;
				ch_changes[2]=DBUS_ReceiveData.rc.ch2 - LASTDBUS_ReceiveData.rc.ch2;
				ch_changes[3]=DBUS_ReceiveData.rc.ch3 - LASTDBUS_ReceiveData.rc.ch3;
				int16_t max_change=1;
				int16_t min_change=-1;
				for (int i = 0; i < 4; ++i)
				{
					if (ch_changes[i]>max_change)
					{
						ch_input[i]+=max_change;
					}
					else if (ch_changes[i] < min_change)
					{
						ch_input[i]+=min_change;
					}
					else ch_input[i]+=ch_changes[i];
				}
				control_car(ch_input[0],ch_input[1],ch_input[2]);
				// control_gimbal(DBUS_ReceiveData.rc.ch2/20,DBUS_ReceiveData.rc.ch3/20);
				// control_gimbal_pitch(ch_input[3]);
			}
			else{
				int16_t key_input_ch0 = (DBUS_CheckPush(KEY_D)-DBUS_CheckPush(KEY_A)) * 660;
				int16_t key_input_ch1 = (DBUS_CheckPush(KEY_W)-DBUS_CheckPush(KEY_S)) *660;
				int16_t key_input_ch2 =	DBUS_ReceiveData.mouse.x;
				control_car(key_input_ch0, key_input_ch1, key_input_ch2);
			} 
			//control_car(DBUS_ReceiveData.rc.ch0, DBUS_ReceiveData.rc.ch1, DBUS_ReceiveData.rc.ch2);
			
			
		}
	}
}	

	



