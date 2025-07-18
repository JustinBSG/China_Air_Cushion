#ifndef __TICK_H__
#define __TICK_H__

#include "headfile.h"

#define TICK_TIM TIM_0

extern uint32 tick;

void tick_init(void);
uint32 get_tick(void);

#endif  // __TICK_H__