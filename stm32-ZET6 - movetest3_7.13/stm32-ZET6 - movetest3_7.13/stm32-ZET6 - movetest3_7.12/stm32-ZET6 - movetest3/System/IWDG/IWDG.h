#ifndef __IWDG_H
#define __IWDG_H
#include "HeaderFiles.h"


void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void);

#endif
