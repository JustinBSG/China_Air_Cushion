#ifndef __FAN_H__
#define __FAN_H__

#include "headfile.h"

typedef struct {
    int speed;
    PWMCH_enum pin;
} Fan;

#define FAN_FREQ 50

#define FAN_MIN_SPEED_PWM 500
#define FAN_MAX_SPEED_PWM 1000

#define FAN_0_SPEED 0
#define FAN_FULL_SPEED 500
#define FAN_MID_SPEED (FAN_0_SPEED + FAN_FULL_SPEED) / 2

void initial_all_fan(void);
void fan_set_speed(Fan *fan, uint32 speed);

extern Fan test_fan;
extern Fan fans[4];

#endif  // __FAN_H__