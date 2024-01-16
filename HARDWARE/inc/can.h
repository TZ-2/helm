#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx.h"                  // Device header
#include "sys.h"
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);





#endif

