#ifndef JUDGE_H
#define JUDGE_H

#include "stm32f4xx.h"

#define JUDGE_BUFFER_LENGTH           256
#define JUDGE_INFO_FRAME_LENGTH       ((uint8_t)44)
#define JUDGE_BLOOD_FRAME_LENGTH      ((uint8_t)12)
#define JUDGE_SHOOT_FRAME_LENGTH      ((uint8_t)25)
#define JUDGE_FRAME_HEADER_LENGTH     ((uint8_t)5)
#define JUDGE_EXTRA_LENGTH            ((uint8_t)9)

#define JUDGE_FRAME_HEADER            0xA5

#define JUDGE_SEND_LENGTH             21

#ifndef JUDGE_FILE
    #define JUDGE_EXT extern
#else
    #define JUDGE_EXT
#endif

#define UNUSED(x) ((void)(x))

JUDGE_EXT volatile uint32_t JUDGE_FrameCounter;
JUDGE_EXT volatile uint8_t JUDGE_Started, JUDGE_RemainByte, JUDGE_NextDecodeOffset;
JUDGE_EXT uint8_t JUDGE_SendBuffer[JUDGE_SEND_LENGTH];

void judging_system_init(void);
void Judge_InitConfig(void);
void Judge_SetSendData(void);
void Judge_Send(void); // max freq 200Hz

void JUDGE_Decode(uint32_t length);
void JUDGE_DecodeFrame(uint8_t type);
uint8_t GetCRC8(uint8_t idx, uint8_t len, uint8_t ucCRC8);
unsigned int VerifyCRC8(uint8_t idx, uint8_t len);
uint8_t Judge_UART_CheckConnection();

void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

typedef struct
{
    uint32_t RemainTime;
    float RealVoltage;
    float RealCurrent;
    int16_t LastBlood;
    uint8_t LastHartID;
    float LastShotSpeed;
    float LastShotFreq;
    float RemainBuffer;
	int16_t ArmorDecrease;
    int16_t BulletDecrease;
    int16_t GolfDecrease;
    int16_t CrashDecrease;
    int16_t OverShootSpeedDecrease;
    int16_t OverShootFreqDecrease;
    int16_t OverPowerDecrease;
    int16_t ModuleOfflineDecrease;
    uint8_t Updated;
    uint8_t OverShootFreqLastTime;
    uint8_t OverShootSpeedLastTime;
    uint8_t BulletShot;
    u32 LastShotTick;
    int ShootNum;
    int OverShootSpeedNum;

    float m_data[3];
} InfantryJudge_Struct;

// format transformation union
typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FormatTrans;

//the data will be stored in the following struct
extern InfantryJudge_Struct InfantryJudge;

#endif
