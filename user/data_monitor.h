#ifndef DATA_MONITOR_H
#define DATA_MONITOR_H

#include "stm32f4xx.h"

// only for uart3 Tx
void DataMonitor_Init(void);

u8 DataMonitor_Send(u8 *buffer, u32 length);

#endif // DATA_MONITOR_H
