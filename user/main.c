#include "main.h"
#include "function_list.h"
#include "PID_s.h"
#include "power_control.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "customized_function.h"
#include "external_control.h"
static u32 ticks_msimg = (u32) - 1;

void init() {
    SysTick_Init();
    Dbus_init();//usart1
    //buzzer_init();     //initialization of buzzer
    tft_init(2, BLACK, GREEN, GREEN);
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
    TIM5_Int_Init(24, 13124); // 256hz //3.9xx ms for gyro usage
    DataMonitor_Init();
}

s32 debug = 0;
s32 target_angle = 0;
s32 current_angle = 0;
s32 last_angle = 0;
int16_t result[4] = {0, 0, 0, 0};
PID wheels_pos_pid[4];
PID wheels_speed_pid[4];
PID angle_pid;
PID buffer_pid;
PID gimbal_speed_pid[2];
PID gimbal_pos_pid[2];
PID gimbal_reset_pid;
u8 str[256];
float buffer_remain;
float init_yaw_pos;
int16_t buff_yaw_pos[3] = {-40, 0, 40};

int main(void)
{
// u8 dum[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '\n'};
    init();
    for (int i = 0; i < 4; i++) {
        //PID_init(&wheels_pos_pid[i],130, 0.004, 0, 5000);//30 0.001 5 65 0.001 5
        PID_init(&wheels_pos_pid[i], 0.15, 0, 0, 20000);
        PID_init(&wheels_speed_pid[i], 80, 5, 100, 20000); //0.00001
    }
    PID_init(&gimbal_speed_pid[0], 80, 5, 100, 20000);
    PID_init(&gimbal_speed_pid[1], 80, 5, 100, 20000);
    PID_init(&gimbal_pos_pid[0], 0.15, 0, 0, 20000);
    PID_init(&gimbal_pos_pid[1], 0.15, 0, 0, 20000);
    PID_init(&angle_pid, 4, 0, 0, 660); //5 20
    PID_init(&buffer_pid, 0.02, 0, 0, 60);
    PID_init(&gimbal_reset_pid, 0.02, 0, 0, 50);
    buffer_remain = 60;;
    init_yaw_pos = GMYawEncoder.ecd_angle;
    while (1)  {

        if (ticks_msimg != get_ms_ticks())
        {
            ticks_msimg = get_ms_ticks();  //maximum 1000000
            //buzzer_check();

            //if (ticks_msimg%500==0)
            //DataMonitor_Send(dum, sizeof(dum));

            //TODO: nothing:-)
            if (ticks_msimg % 20 == 0)
            {
                buffer_call();
            }
            if (ticks_msimg % 50 == 0)
            {
                tft_clear();
                tft_prints(0, 2, "Infantry V1.1");
                tft_clear();
                tft_prints(1, 2, "rc0:%d rc1:%d", DBUS_ReceiveData.rc.ch0, DBUS_ReceiveData.rc.ch1);
                tft_prints(1, 3, "rc2:%d rc3:%d", DBUS_ReceiveData.rc.ch2, DBUS_ReceiveData.rc.ch3);
                tft_prints(1, 4, "Pr:%f", InfantryJudge.RealCurrent * InfantryJudge.RealVoltage);
                tft_prints(1, 5, "init:%f", init_yaw_pos);
                tft_prints(1, 6, "curr:%f", GMYawEncoder.ecd_angle);
                tft_prints(1, 7, "5:%d 6:%d", GMYawEncoder.filter_rate, GMPitchEncoder.filter_rate);
                tft_prints(1, 8, "debug:%f", debug);
                tft_prints(1, 9, "cur:%d", current_angle);
                tft_prints(1, 10, "tar:%d", target_angle);
                tft_prints(1, 11, "Mouse:%d right:%d", DBUS_ReceiveData.mouse.x, DBUS_ReceiveData.mouse.press_right);
                tft_update();
                LED_blink(LED1);
            }
            external_control();
        }
    }
		
}
