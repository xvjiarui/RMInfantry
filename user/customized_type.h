#include "function_list.h"
#ifndef CUSTOMIZED_TYPE_H
#define CUSTOMIZED_TYPE_H
typedef union u32ANDint16_t{
	u32 flash;
	int16_t mem;
} u32ANDint16_t;
typedef enum CarMode
{
    NORMAL,
    OPEN_LOOP,
    SEMI_CLOSED_LOOP,
    DANCING
} CarMode;

#endif