#ifndef __GPIO_H
#define __GPIO_H

#include "sys.h"

//LED端口定义
#define LED1 PDout(2)	// DS0
#define LED2 PDout(3)	// DS0
#define LED3 PDout(4)	// DS0

#define Lifting PEout(13)
#define Clamping PEout(14)
#define Push PEout(15)



void gpio_Init(void);//初始化		 				    
#endif
