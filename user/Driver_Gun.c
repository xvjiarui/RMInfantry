#define GUN_FILE
#define ADD_BULLET
// #define GUN_SHOOTING_ADJUST

#include "Dbus.h"
#include "global_variable.h"
#include "param.h"
#include "customized_function.h"
#include "gimbal_control.h"
#include "judge.h"
#include "external_control.h"
#include "Driver_Gun.h"

#include <string.h>

float GUN_PokeErr = 0;
int32_t GUN_PokeOutput = 0;
float GUN_PokeIntegral = 0;
// static PID_Controller PokeSpeedController;
// static PID_Controller PokeAngleController;

void GUN_BSP_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;

    // Friction Wheel
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // TIM1 (friction wheel, 400Hz)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   9999;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   41; // 1MHz clock
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode       =   TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse        =   4000;
    TIM_OCInitStructure.TIM_OutputState  =   TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState =   TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity   =   TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  =   TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  =   TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState =   TIM_OCNIdleState_Set;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

}

void GUN_Init(void) {
    GUN_BSP_Init();

    memset((char*)&GUN_Data, 0, sizeof(GUN_Data));
    GUN_Direction = 1;
#ifdef GUN_SHOOTING_ADJUST
    GUN_Data.frictionPWM = 1000;
#endif

}

extern volatile u32 ticks_msimg;
void GUN_SetMotion(void) {
    static char shoot = 0;
    static char jumpPress = 0, jumpRelease = 0;
    static int32_t lastTick = 0;
    static int32_t pressCount = 0;
    static uint8_t hasPending = 0;

    // friction wheel
    if (status != 1 && DBUS_Connected) {
        uint16_t friction_wheel_pwm = Friction_Wheel_PWM();
        static float ratio = 1;
        if (InfantryJudge.OverShootSpeedLastTime)
        {
            ratio *= 0.95;
            InfantryJudge.OverShootSpeedLastTime = 0;
        }
        friction_wheel_pwm *= ratio;
#ifdef GUN_SHOOTING_ADJUST
        FRIC_SET_THRUST_L(GUN_Data.frictionPWM);
        FRIC_SET_THRUST_R(GUN_Data.frictionPWM);
        FRIC_SET_THRUST_M(GUN_Data.frictionPWM);
#else
        GUN_Data.frictionPWM = friction_wheel_pwm;
        FRIC_SET_THRUST_L(friction_wheel_pwm);
        FRIC_SET_THRUST_R(friction_wheel_pwm);
        FRIC_SET_THRUST_M(friction_wheel_pwm);
#endif
    }
    else {
        FRIC_SET_THRUST_L(0);
        FRIC_SET_THRUST_R(0);
        FRIC_SET_THRUST_M(0);
    }

    uint8_t KeyNow = DBUS_CheckPushNow(KEY_V) && (DBUS_CheckPushNow(KEY_Q) || DBUS_CheckPushNow(KEY_W) || DBUS_CheckPushNow(KEY_E) || DBUS_CheckPushNow(KEY_A) || DBUS_CheckPushNow(KEY_S) || DBUS_CheckPushNow(KEY_D) || DBUS_CheckPushNow(KEY_Z) || DBUS_CheckPushNow(KEY_X) || DBUS_CheckPushNow(KEY_C));
    uint8_t KeyLast = DBUS_CheckPushLast(KEY_V) && (DBUS_CheckPushLast(KEY_Q) || DBUS_CheckPushLast(KEY_W) || DBUS_CheckPushLast(KEY_E) || DBUS_CheckPushLast(KEY_A) || DBUS_CheckPushLast(KEY_S) || DBUS_CheckPushLast(KEY_D) || DBUS_CheckPushLast(KEY_Z) || DBUS_CheckPushLast(KEY_X) || DBUS_CheckPushLast(KEY_C));

    uint8_t keyJumpPress = KeyNow && !KeyLast;
    uint8_t keyJumpRelease = !KeyNow && KeyLast;

    uint8_t adjustNow = (DBUS_ReceiveData.rc.ch2 < -500 && DBUS_ReceiveData.rc.switch_right == 3 && DBUS_ReceiveData.rc.switch_left == 3) ? 1 : 0;
    uint8_t adjustLast = (LASTDBUS_ReceiveData.rc.ch2 < -500 && DBUS_ReceiveData.rc.switch_right == 3 && DBUS_ReceiveData.rc.switch_left == 3) ? 1 : 0;

    uint8_t adjustJumpPress = adjustNow && !adjustLast;
    uint8_t adjustJumpRelease = !adjustNow && adjustLast;

    uint8_t shootDecreaseNow = (DBUS_ReceiveData.rc.ch3 < -500 && DBUS_ReceiveData.rc.switch_right == 3 && DBUS_ReceiveData.rc.switch_left == 3) ? 1 : 0;
    uint8_t shootDecreaseLast = (LASTDBUS_ReceiveData.rc.ch3 < -500 && DBUS_ReceiveData.rc.switch_right == 3 && DBUS_ReceiveData.rc.switch_left == 3) ? 1 : 0;

    uint8_t shootDecreaseJumpPress = shootDecreaseNow && !shootDecreaseLast;
    uint8_t shootDecreaseJumpRelease = !shootDecreaseNow && shootDecreaseLast;

    uint8_t shootIncreaseNow = (DBUS_ReceiveData.rc.ch3 < 500 && DBUS_ReceiveData.rc.switch_right == 3 && DBUS_ReceiveData.rc.switch_left == 3) ? 1 : 0;
    uint8_t shootIncreaseLast = (LASTDBUS_ReceiveData.rc.ch3 < 500 && DBUS_ReceiveData.rc.switch_right == 3 && DBUS_ReceiveData.rc.switch_left == 3) ? 1 : 0;

    uint8_t shootIncreaseJumpPress = shootIncreaseNow && !shootIncreaseLast;
    uint8_t shootIncreaseJumpRelease = !shootIncreaseNow && shootIncreaseLast;
    if (shootDecreaseJumpPress)
    {
        GUN_Data.frictionPWM--;
    }
    if (shootIncreaseJumpPress)
    {
        GUN_Data.frictionPWM++;
    }
    // poke motor
    jumpPress = DBUS_ReceiveData.mouse.press_left &&
                !LASTDBUS_ReceiveData.mouse.press_left;
    jumpRelease = !DBUS_ReceiveData.mouse.press_left &&
                  LASTDBUS_ReceiveData.mouse.press_left;

    jumpPress = jumpPress || keyJumpPress || adjustJumpPress;
    jumpRelease = jumpRelease || keyJumpRelease || adjustJumpRelease;


    if (jumpRelease) pressCount = 0;
    if (GUN_Data.usrShot && (GUN_Data.last_poke_tick + 220 < ticks_msimg))
    {
        if (GUN_Data.last_poke_tick > InfantryJudge.LastShotTick)
        {
            if (GUN_Data.emptyLastTick + 220 < ticks_msimg)
            {
                ++GUN_Data.emptyCount;
                GUN_Data.emptyLastTick = ticks_msimg;
            }
            if (GUN_Data.emptyCount >= 5)
            {
                // No bullet
                GUN_SetFree();
                pressCount = 0;
            }
        }
        GUN_Data.usrShot = 0;
    }
    if (GUN_Data.last_poke_tick <= InfantryJudge.LastShotTick + 220)
    {
        GUN_Data.emptyCount = 0;
    }
    if (GUN_Data.stuckedLastTick <= InfantryJudge.LastShotTick + 220)
    {
        GUN_Data.stucked = 0;
    }
#ifdef ADD_BULLET
    if (DBUS_ReceiveData.mouse.press_left
            || (GUN_Data.emptyCount && GUN_Data.emptyLastTick + 220 < ticks_msimg) || (GUN_Data.stucked && GUN_Data.stuckedLastTick + 220 < ticks_msimg))
    {
        ++pressCount;
    }
#else
    if (DBUS_ReceiveData.mouse.press_left)
    {
        ++pressCount;
    }
#endif

    shoot = jumpPress || (((pressCount & 0x000FU) == 0) && pressCount);
    shoot = shoot || shootRune;
    shoot = shoot || (DBUS_ReceiveData.rc.switch_right == 3 && DBUS_ReceiveData.rc.switch_left == 3 && DBUS_ReceiveData.rc.ch2 > 500);
    shoot = shoot && (DBUS_ReceiveData.rc.switch_right != 1);
    shoot = shoot && (ticks_msimg - lastTick > 300);
    if (DBUS_ReceiveData.mouse.press_right)
    {
        GUN_SetFree();
        shoot = 0;
        pressCount = 0;
    }
    if (shoot && !hasPending) {
        if (shootRune) //keyJumpPress
        {
            hasPending = 1;
            shootRune = 0;
        }
    }
    if (hasPending)
    {
        if (gimbal_in_buff_pos) {
            GUN_ShootOne();
            hasPending = 0;
            lastTick = ticks_msimg;
            lastRuneTick = ticks_msimg;
            buff_pressed = 0;
        }
    }
    else if (shoot)
    {
        GUN_ShootOne();
        lastTick = ticks_msimg;
    }
}

void GUN_ShootOne(void)
{
    if (GUN_Direction == 1)
    {
        GUN_TargetPos += 36 * 72;
    }
    else GUN_TargetPos -= 36 * 72;
    GUN_Data.last_ecd_angle = GMxEncoder.ecd_angle;
    GUN_Data.last_poke_tick = ticks_msimg;
    GUN_Data.usrShot = 1;

}

void GUN_SetFree(void)
{
    PID_Reset_driver();
    GUN_Data.last_ecd_angle = GMxEncoder.ecd_angle;
    GUN_Data.emptyLastTick = ticks_msimg + 220;
    GUN_Data.last_poke_tick = InfantryJudge.LastShotTick - 220;
    GUN_Data.emptyCount = 0;
    GUN_Data.usrShot = 0;
    GUN_Data.stucked = 0;
}

void GUN_Update(void)
{
    // if (fabs(GMxEncoder.ecd_angle) > 216000 || fabs(GUN_TargetPos) > 216000)
    // {
    //     GMxEncoder.ecd_angle = 0;
    //     GUN_TargetPos = 0;
    // }
    float temp = ABS(gun_driver_speed_pid.Ki * gun_driver_speed_pid.i);
    float_debug = float_debug > temp ? float_debug : temp;

    if (ABS(gun_driver_speed_pid.Ki * gun_driver_speed_pid.i) > 6000
            || ((ticks_msimg - GUN_Data.last_poke_tick) > 220
                && (ticks_msimg - GUN_Data.last_poke_tick) < 240
                && float_equal(GMxEncoder.ecd_angle, GUN_Data.last_ecd_angle, 36)))
    {
        GUN_Direction *= -1;
        GUN_SetFree();
        GUN_Data.stucked = 1;
        GUN_Data.stuckedLastTick = ticks_msimg;
    }

    GUN_DriverInput = PID_UpdateValue(&gun_driver_speed_pid,
                                      PID_UpdateValue(&gun_driver_pos_pid,
                                              GUN_TargetPos,
                                              GMxEncoder.ecd_angle),
                                      GMxEncoder.filter_rate);
}

uint16_t Friction_Wheel_PWM(void)
{
    uint16_t result = 0;
    switch (InfantryID)
    {
    case 0:
        if (InfantryJudge.RealVoltage > 24.5)
        {
            result = 1000; //650
        }
        else if (InfantryJudge.RealVoltage > 24)
        {
            result = 1055; //670
        }
        else if (InfantryJudge.RealVoltage > 23.5)
        {
            result = 1075; //690
        }
        else if (InfantryJudge.RealVoltage > 23)
        {
            result = 1095; //700
        }
        else if (InfantryJudge.RealVoltage > 22.5)
        {
            result = 1105; //715
        }
        else if (InfantryJudge.RealVoltage > 22)
        {
            result = 1125; //730
        }
        else if (InfantryJudge.RealVoltage > 21.5)
        {
            result = 1145; //745
        }
        else
        {
            result = 1165; // 750
        }
        break;
    case 1:
        if (InfantryJudge.RealVoltage > 24.5)
        {
            result = 1110; //650
        }
        else if (InfantryJudge.RealVoltage > 24)
        {
            result = 1130; //670
        }
        else if (InfantryJudge.RealVoltage > 23.5)
        {
            result = 1150; //690
        }
        else if (InfantryJudge.RealVoltage > 23)
        {
            result = 1170; //700
        }
        else if (InfantryJudge.RealVoltage > 22.5)
        {
            result = 1190; //715
        }
        else if (InfantryJudge.RealVoltage > 22)
        {
            result = 1210; //730
        }
        else if (InfantryJudge.RealVoltage > 21.5)
        {
            result = 1240; //745
        }
        else
        {
            result = 1260; // 750
        }
        break;
    case 2:
        if (InfantryJudge.RealVoltage > 24.5)
        {
            result = 1060; //650
        }
        else if (InfantryJudge.RealVoltage > 24)
        {
            result = 1100; //670
        }
        else if (InfantryJudge.RealVoltage > 23.5)
        {
            result = 1120; //690
        }
        else if (InfantryJudge.RealVoltage > 23)
        {
            result = 1140; //700
        }
        else if (InfantryJudge.RealVoltage > 22.5)
        {
            result = 1150; //715
        }
        else if (InfantryJudge.RealVoltage > 22)
        {
            result = 1170; //730
        }
        else if (InfantryJudge.RealVoltage > 21.5)
        {
            result = 1185; //745
        }
        else
        {
            result = 1200; // 750
        }
        break;
    default :
        if (InfantryJudge.RealVoltage > 24.5)
        {
            result = 1040; //650
        }
        else if (InfantryJudge.RealVoltage > 24)
        {
            result = 1055; //670
        }
        else if (InfantryJudge.RealVoltage > 23.5)
        {
            result = 1075; //690
        }
        else if (InfantryJudge.RealVoltage > 23)
        {
            result = 1085; //700
        }
        else if (InfantryJudge.RealVoltage > 22.5)
        {
            result = 1105; //715
        }
        else if (InfantryJudge.RealVoltage > 22)
        {
            result = 1125; //730
        }
        else if (InfantryJudge.RealVoltage > 21.5)
        {
            result = 1145; //745
        }
        else
        {
            result = 1165; // 750
        }
        break;
    }

    int delta_t = ticks_msimg - InfantryJudge.LastShotTick;
    uint16_t b = 60;
    int interval = 900;
    uint16_t offset;
    if (delta_t > interval)
    {
        offset = 0;
    }
    else
    {
        offset = b - b * delta_t / interval;
    }
    result += offset;
    return result;
}
