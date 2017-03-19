#ifndef	__FLASH_H
#define	__FLASH_H

#include "stm32f4xx_flash.h"
#include "stm32f4xx.h"


void writeFlash(u32 *data, u8 num);
u32 readFlash(u16 address_num);


#endif /* __FLASH_H */
