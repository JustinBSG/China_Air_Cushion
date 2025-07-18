#ifndef __MAIN_H__
#define __MAIN_H__

#include "headfile.h"

#define TEST_PWM PWMA_CH4P_P66

typedef enum {
  FALSE = 0,
  TRUE = !FALSE
} bool;

#endif  // __MAIN_H__