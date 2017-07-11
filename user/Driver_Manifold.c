#define DRIVER_MANIFOLD_FILE
#include "Driver_Manifold.h"
#include "Judge.h"


void Driver_Manifold_init() {

    NVIC_InitTypeDef    NVIC_InitStructure;
    USART_InitTypeDef   USART_InitStructure;

    //enable Dbus related clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    //init pin that has been chosen on the RM2017 board
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

    GPIO_InitTypeDef    GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //init for double side transmission
    USART_InitStructure.USART_BaudRate              =   115200;
    USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  =   USART_Mode_Rx;
    USART_InitStructure.USART_Parity                =   USART_Parity_Even;
    USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
    USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
    USART_Init(UART4, &USART_InitStructure);
    USART_Cmd(UART4, ENABLE);

    //init NVIC, interrupt enable
    NVIC_InitStructure.NVIC_IRQChannel                      =   UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);

    //UART4 RX DMA config
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    DMA_DeInit(DMA1_Stream2);
    DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr =   (uint32_t)(&UART4->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(Manifold_Buffer);
    DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize        =   Manifold_Buffer_Length;
    DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize =   DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream2, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream2, ENABLE);


    LED_init(&PA2);

    rune_angle_x = 0;
    rune_angle_y = 0;
    isNewRuneAngle = 0;
    manifoldCount = 0;
}


void Manifold_Rune_Select(u8 rune) { // 1 is big, 0 is small,, BORAD loccation : LEFT UPPER CORNER: BRUSH_M2 pin: beside gnd

    if (rune == 1) {
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
    }
    else if (rune == 0) {

        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    }
}

typedef union {
	uint8_t u[4];
	float f;
} FT_T;

void Manifold_Decoder()
{	
	FT_T FT;
	//manifoldCount++;
	
    if (Manifold_Buffer[0] == 0xa5)
    {
        
        //rune_angle_x = *(float*)(Manifold_Buffer + 1);
        //rune_angle_y = *(float*)(Manifold_Buffer + 5);
			FT.u[0] = Manifold_Buffer[1];
			FT.u[1] = Manifold_Buffer[2];
			FT.u[2] = Manifold_Buffer[3];
			FT.u[3] = Manifold_Buffer[4];
			rune_angle_x = FT.f;
			
			FT.u[0] = Manifold_Buffer[5];
			FT.u[1] = Manifold_Buffer[6];
			FT.u[2] = Manifold_Buffer[7];
			FT.u[3] = Manifold_Buffer[8];
			rune_angle_y = FT.f;
        isNewRuneAngle = 1;
    }
}

u8 clear = 0;
void UART4_IRQHandler(void)  //BOARD location: LEFT UPPER CORNER , Below DBUS PORT BRUSH_M1 (M1 Dir) beside 5v
{

    clear = UART4->DR;
    clear = UART4->SR;

    DMA_Cmd(DMA1_Stream2, DISABLE);
	manifoldCount++;
		
    if (DMA1_Stream2->NDTR == Manifold_Buffer_Length)
    {
        Manifold_Decoder();
    }

    DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
    while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream2, Manifold_Buffer_Length);
    DMA_Cmd(DMA1_Stream2, ENABLE);
}





