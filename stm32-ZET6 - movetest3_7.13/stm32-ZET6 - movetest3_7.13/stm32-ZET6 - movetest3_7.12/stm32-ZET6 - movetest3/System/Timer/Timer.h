#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"                  // Device header

extern FlagStatus Timer_Flag;
extern FlagStatus Timer4_Flag;

void Timer2_Init(uint16_t ARR, uint16_t PSC);
void Timer3_Init(uint16_t ARR, uint16_t PSC);
void Timer4_Init(uint16_t ARR, uint16_t PSC);

extern uint16_t Flag99;
#endif
