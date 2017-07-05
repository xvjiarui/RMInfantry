#ifndef DRIVER_GUN
#define DRIVER_GUN

#include "stm32f4xx.h"

#ifndef GUN_FILE
    #define GUN_EXT extern
#else
    #define GUN_EXT
#endif

#define POKE_DIR_PORT                           GPIOA
#define POKE_DIR_PIN                            GPIO_Pin_1
#define POKE_DIR                                1
#define FRIC_SET_THRUST_L(x)                    TIM_SetCompare1(TIM1, 1000+(x))
#define FRIC_SET_THRUST_R(x)                    TIM_SetCompare2(TIM1, 1000+(x))
#define FRIC_SET_THRUST_M(x)                    TIM_SetCompare3(TIM1, 1000+(x))

typedef struct {
	float last_ecd_angle;
	u32 last_poke_tick;
	uint8_t emptyCount;
	uint8_t usrShot;
	u32 emptyLastTick;

} GUN_DataTypeDef;

GUN_EXT volatile GUN_DataTypeDef GUN_Data;

void GUN_Init(void);
void GUN_SetMotion(void);
void GUN_SetFree(void);
void GUN_Update(void);

void GUN_ShootOne(void);

uint16_t Friction_Wheel_PWM(void);
	
#endif