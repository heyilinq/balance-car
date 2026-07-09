#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"
#define ENCODER_TIM_PERIOD (u16)(65535)   //ɴ65535 ΪF103Ķʱ16λġ
void Encoder_Init_TIM2(void);             //TIM2ʼΪӿģʽ
void Encoder_Init_TIM4(void);             //TIM3ʼΪӿģʽ
int Read_Encoder(u8 TIMX);               //λʱȡ
#endif
