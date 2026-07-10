#ifndef __LED_H
#define __LED_H
#include "sys.h"
//LED 端口定义
#define LED PAout(4) // PB13
void Led_Flash(u16 time);

#endif
