#ifndef DRIVER_ENCODER
#define DRIVER_ENCODER

#include "stm32f4xx.h"

// 0: normal  1: reverse
#define ENCODER_DIR                             1
#define ENCODER_TIM                             TIM3

#ifndef ENCODER_FILE
    #define ENCODER_EXT extern
#else
    #define ENCODER_EXT
#endif

ENCODER_EXT int32_t ENCODER_Data;
extern float GUN_PokeErr;
extern int32_t GUN_PokeOutput;
extern float GUN_PokeIntegral;
		
void ENCODER_BSP_Init(void);
void ENCODER_Init(void);
void ENCODER_Update(void);
uint8_t ENCODER_CheckConnection(void);

#endif