#include "BSP_TIM.h"
#include "stm32f4xx.h"

void BSP_TIM_InitConfig(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    // TIM7 (main control 1kHz)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   1000-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   (uint32_t) (((168000000 / 2) / 1000000)-1); // 1MHz clock
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;

    // TIM7 (main control)
    NVIC_InitStructure.NVIC_IRQChannel                      =   TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);
}

void BSP_TIM_Start(void) {   
    // TIM7
    TIM_Cmd(TIM7, ENABLE);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
}