#define JUDGE_FILE

#include "judge.h"
#include "main.h"
#include "Driver_Gun.h"
#include "data_monitor.h"

#define GET_BUFFER(x) JUDGE_DataBuffer[(uint8_t)(JUDGE_NextDecodeOffset+(x))]

InfantryJudge_Struct InfantryJudge;

uint8_t JUDGE_DataBuffer[JUDGE_BUFFER_LENGTH];

uint32_t Judge_UART_counter = 0;
uint32_t Judge_UART_last_counter = 0;

//HKUST RM2017 mainboad judge : USART3

void judging_system_init(void) {

    DMA_InitTypeDef     DMA_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_InitTypeDef    GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);


    USART_InitTypeDef   USART_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    USART_InitStructure.USART_BaudRate              =   115200;
    USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  =   USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity                =   USART_Parity_No;
    USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
    USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART3, ENABLE);

    //USART3
    NVIC_InitStructure.NVIC_IRQChannel                      =   USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

    //UART3_RX
    DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr =   (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(JUDGE_DataBuffer);
    DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize        =   JUDGE_BUFFER_LENGTH;
    DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize =   DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode              =   DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1, ENABLE);
    Judge_InitConfig();
}

void Judge_SetSendData(void) {
    InfantryJudge.m_data[0] = InfantryJudge.ShootNum;
}

void Judge_Send(void) {
    static uint8_t *buf = JUDGE_SendBuffer;
    static uint8_t cnt = 0;
    static FormatTrans FT;
    static uint8_t i, j;

    Judge_SetSendData();

    buf[0] = JUDGE_FRAME_HEADER;
    buf[1] = 12;
    buf[2] = 0;
    buf[3] = cnt++;
    Append_CRC8_Check_Sum(buf, JUDGE_FRAME_HEADER_LENGTH);
    buf[5] = 5;
    buf[6] = 0;

    for (i = 0; i < 3; ++i) {
        FT.F = InfantryJudge.m_data[i];
        for (j = 0; j < 4; ++j)
            buf[7+(i<<2)+j] = FT.U[j];
    }

    Append_CRC16_Check_Sum(buf, JUDGE_SEND_LENGTH);

    DataMonitor_Send(JUDGE_SendBuffer, JUDGE_SEND_LENGTH);
}



/***********************************  CRC  ***********************************/
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}


/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength - 1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength - 1] );
}

uint8_t GetCRC8(uint8_t idx, uint8_t len, uint8_t ucCRC8) {
    while (len--)
        ucCRC8 = CRC8_TAB[ucCRC8 ^ JUDGE_DataBuffer[idx++]];
    return ucCRC8;
}

unsigned int VerifyCRC8(uint8_t idx, uint8_t len) {
    uint8_t ucExpected = GetCRC8(idx, len - 1, CRC8_INIT);
    return ucExpected == JUDGE_DataBuffer[(uint8_t)(idx + len - 1)];
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == 0)
    {
        return 0xFFFF;
    }
    while (dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^
                 (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength - 2, CRC_INIT );
    pchMessage[dwLength - 2] = (u8)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (u8)((wCRC >> 8) & 0x00ff);
}
/***********************************  CRC  ***********************************/


/**
  * @brief  ????????'??
  * @param  void
  * @retval void
  */
void Judge_InitConfig(void)
{
    JUDGE_FrameCounter = 0;
    JUDGE_Started = 0;
    JUDGE_RemainByte = 0;

    InfantryJudge.RealVoltage = 25.2F;
    InfantryJudge.RealCurrent = 0;
    InfantryJudge.LastBlood = 1500;
    InfantryJudge.LastShotSpeed = 23.5;
    InfantryJudge.LastShotFreq = 1.0;
    InfantryJudge.RemainBuffer = 60.0;
    InfantryJudge.LastHartID = 0;
    InfantryJudge.ArmorDecrease = 0;
    InfantryJudge.BulletDecrease = 0;
    InfantryJudge.GolfDecrease = 0;
    InfantryJudge.CrashDecrease = 0;
    InfantryJudge.OverShootSpeedDecrease = 0;
    InfantryJudge.OverShootFreqDecrease = 0;
    InfantryJudge.OverPowerDecrease = 0;
    InfantryJudge.ModuleOfflineDecrease = 0;
    InfantryJudge.Updated = 0;
    InfantryJudge.OverShootFreqLastTime = 0;
    InfantryJudge.OverShootSpeedLastTime = 0;
    InfantryJudge.BulletShot = 0;
    InfantryJudge.LastShotTick = 0;
    InfantryJudge.ShootNum = 0;

    float* foo = InfantryJudge.m_data;
    foo[0] = foo[1] = foo[2] = 0;
}

/**
  * @brief  This function handles UART3 interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void) {
    static uint8_t dum;
    dum = USART3->DR;
    dum = USART3->SR;

    JUDGE_Decode(DMA1_Stream1->NDTR);

    UNUSED(dum);
}

uint8_t judgeFrameDataLength;
uint8_t judgeFrameTotalLength;
void JUDGE_Decode(uint32_t length) {
    static uint8_t repeatFlag;

    if (!JUDGE_Started)
        for (uint32_t i = 0; i < JUDGE_BUFFER_LENGTH; ++i)
            if (JUDGE_DataBuffer[i] == JUDGE_FRAME_HEADER
                    && VerifyCRC8(i, JUDGE_FRAME_HEADER_LENGTH)) {
                JUDGE_Started = 1;
                JUDGE_NextDecodeOffset = i;
            }

    if (JUDGE_Started) {
        JUDGE_RemainByte = JUDGE_BUFFER_LENGTH - length - 1;
        JUDGE_RemainByte -= JUDGE_NextDecodeOffset + 1;

        repeatFlag = 1;
        while (JUDGE_RemainByte >= JUDGE_FRAME_HEADER_LENGTH && repeatFlag
                && VerifyCRC8(JUDGE_NextDecodeOffset, JUDGE_FRAME_HEADER_LENGTH)) {
            repeatFlag = 0;
            judgeFrameDataLength = GET_BUFFER(1);
            judgeFrameTotalLength = judgeFrameDataLength + JUDGE_EXTRA_LENGTH;
            if (JUDGE_RemainByte >= judgeFrameTotalLength) {
                ++JUDGE_FrameCounter;
                Judge_UART_counter++;

                /* Decode */
                JUDGE_DecodeFrame(GET_BUFFER(5));

                /* Update data pointer */
                JUDGE_NextDecodeOffset += judgeFrameTotalLength;
                JUDGE_RemainByte -= judgeFrameTotalLength;
                repeatFlag = 1;
            }
        }
    }
}

extern volatile u32 ticks_msimg;
void JUDGE_DecodeFrame(uint8_t type) {
    volatile FormatTrans FT;
    if (type == 1) {
        FT.U[0] = GET_BUFFER(13);
        FT.U[1] = GET_BUFFER(14);
        FT.U[2] = GET_BUFFER(15);
        FT.U[3] = GET_BUFFER(16);
        InfantryJudge.RealVoltage = FT.F;

        FT.U[0] = GET_BUFFER(17);
        FT.U[1] = GET_BUFFER(18);
        FT.U[2] = GET_BUFFER(19);
        FT.U[3] = GET_BUFFER(20);
        InfantryJudge.RealCurrent = FT.F;

        FT.U[0] = GET_BUFFER(38);
        FT.U[1] = GET_BUFFER(39);
        FT.U[2] = GET_BUFFER(40);
        FT.U[3] = GET_BUFFER(41);
        InfantryJudge.RemainBuffer = FT.F;

        InfantryJudge.LastBlood = ((uint16_t)GET_BUFFER(12) << 8) | GET_BUFFER(11);
        InfantryJudge.Updated = 1;
    }
    else if (type == 2) {
        uint8_t way = GET_BUFFER(7) >> 4;
        uint16_t delta = ((uint16_t)GET_BUFFER(9) << 8) | GET_BUFFER(8);
        switch (way) {
        case 0: // bullet damage
            InfantryJudge.LastHartID = GET_BUFFER(7) & 0x0F;
            InfantryJudge.BulletDecrease += delta;
            InfantryJudge.ArmorDecrease += delta;
            break;
        case 1: // golf damage
            InfantryJudge.LastHartID = GET_BUFFER(7) & 0x0F;
            InfantryJudge.GolfDecrease += delta;
            InfantryJudge.ArmorDecrease += delta;
            break;
        case 2: // crash damage
            InfantryJudge.CrashDecrease += delta;
            break;
        case 3: // over speed
            InfantryJudge.OverShootSpeedDecrease += delta;
            InfantryJudge.OverShootSpeedNum++;
            InfantryJudge.OverShootSpeedLastTime = 1;
            break;
        case 4: // over frequency
            InfantryJudge.OverShootFreqDecrease += delta;
            InfantryJudge.OverShootFreqLastTime = 1;
            break;
        case 5: // over power
            InfantryJudge.OverPowerDecrease += delta;
            break;
        case 6: // module offline
            InfantryJudge.ModuleOfflineDecrease += delta;
            break;
        case 7: // violation
            // InfantryJudge.violationDamage += delta;
            break;
        }
    }
    else if (type == 3) {
        FT.U[0] = GET_BUFFER(7);
        FT.U[1] = GET_BUFFER(8);
        FT.U[2] = GET_BUFFER(9);
        FT.U[3] = GET_BUFFER(10);
        InfantryJudge.LastShotSpeed = FT.F;

        FT.U[0] = GET_BUFFER(11);
        FT.U[1] = GET_BUFFER(12);
        FT.U[2] = GET_BUFFER(13);
        FT.U[3] = GET_BUFFER(14);
        InfantryJudge.LastShotFreq = FT.F;
        InfantryJudge.ShootNum++;
        InfantryJudge.BulletShot = 1;
        InfantryJudge.LastShotTick = ticks_msimg;
        GUN_Data.emptyCount = 0;
    }
}

uint8_t Judge_UART_CheckConnection()
{
    if (Judge_UART_last_counter != Judge_UART_counter)
    {
        Judge_UART_last_counter = Judge_UART_counter;
        return 1;
    }
    else return 0;
}