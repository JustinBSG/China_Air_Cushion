#include "fan.h"
// TODO: Add array of 6 fans
Fan test_fan = {FAN_0_SPEED_PWM, PWMA_CH4P_P66};

void initial_all_fan(void) {
  pwm_init(PWMA_CH4P_P66, FAN_FREQ, 0);
  pwm_duty(PWMA_CH4P_P66, FAN_0_SPEED_PWM);
}

void fan_set_speed(Fan *fan, int speed) {
  if (speed < FAN_0_SPEED_PWM)
    speed = FAN_0_SPEED_PWM;
  else if (speed > FAN_FULL_SPEED_PWM)
    speed = FAN_FULL_SPEED_PWM;

  pwm_duty(fan->pin, speed);
  fan->speed = speed;
}