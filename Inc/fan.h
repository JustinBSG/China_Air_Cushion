#ifndef __FAN_H__
#define __FAN_H__

#include "headfile.h"

typedef struct {
  int speed;
  PWMCH_enum pin;
} Fan;

#define FAN_FL_PIN
#define FAN_FR_PIN
#define FAN_RL_PIN
#define FAN_RR_PIN
#define FAN_UP1_PIN
#define FAN_UP2_PIN

#define FAN_FREQ 50

#define FAN_0_SPEED_PWM     PWM_DUTY_MAX*0.05
#define FAN_FULL_SPEED_PWM  PWM_DUTY_MAX*0.05*2

void initial_all_fan(void);
void fan_set_speed(Fan *fan, int speed);

#endif // __FAN_H__