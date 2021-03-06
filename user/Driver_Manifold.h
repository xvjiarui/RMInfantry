#ifndef DRIVER_MANIFOLD
#define DRIVER_MANIFOLD

#include "stm32f4xx.h"
#include "led.h"
#include "gpio.h"

#ifndef DRIVER_MANIFOLD_FILE
 	#define DRIVER_MANIFOLD_EXT extern
 #else 
	#define DRIVER_MANIFOLD_EXT 
#endif

#define Manifold_Buffer_Length 14

DRIVER_MANIFOLD_EXT DMA_InitTypeDef DMA_InitStructure;
DRIVER_MANIFOLD_EXT uint8_t Manifold_Buffer[Manifold_Buffer_Length];
DRIVER_MANIFOLD_EXT float rune_angle_x;
DRIVER_MANIFOLD_EXT float rune_angle_y;
DRIVER_MANIFOLD_EXT int rune_index;
DRIVER_MANIFOLD_EXT float rune_distance;
DRIVER_MANIFOLD_EXT uint8_t isNewRuneAngle;
DRIVER_MANIFOLD_EXT int manifoldCount;

void Driver_Manifold_init();
void Manifold_Rune_Select(u8 rune);
#endif