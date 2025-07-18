#ifndef __TASK_H__
#define __TASK_H__

#include "fan.h"
#include "headfile.h"
#include "pid.h"

void task1(void);
void task2(void);
void task3(void);

extern uint8 task1_stage;
extern uint8 task2_stage;
extern uint8 task3_stage;

#endif  // __TASK_H__