#include "tick.h"

uint32 tick = 0;

void tick_init(void) {
  pit_timer_ms(TICK_TIM, 1);
}

uint32 get_tick(void) {
  return tick;
}