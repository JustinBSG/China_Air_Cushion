#include "fan.h"

// TODO: Add array of 6 fans
Fan fans[4] = {{FAN_0_SPEED_PWM, PWMB_CH1_P20}, {FAN_0_SPEED_PWM, PWMB_CH2_P21}, {FAN_0_SPEED_PWM, PWMB_CH3_P22}, {FAN_0_SPEED_PWM, PWMB_CH4_P23}};
Fan test_fan = {FAN_0_SPEED_PWM, PWMA_CH4P_P66};

void initial_all_fan(void) {
  pwm_init(PWMB_CH1_P20, FAN_FREQ, 0);
  pwm_duty(PWMB_CH1_P20, FAN_0_SPEED_PWM);
  pwm_init(PWMB_CH2_P21, FAN_FREQ, 0);
  pwm_duty(PWMB_CH2_P21, FAN_0_SPEED_PWM);
  pwm_init(PWMB_CH3_P22, FAN_FREQ, 0);
  pwm_duty(PWMB_CH3_P22, FAN_0_SPEED_PWM);
  pwm_init(PWMB_CH4_P23, FAN_FREQ, 0);
  pwm_duty(PWMB_CH4_P23, FAN_0_SPEED_PWM);
}

void fan_set_speed(Fan *fan, int speed) {
  if (speed < FAN_0_SPEED_PWM)
    speed = FAN_0_SPEED_PWM;
  else if (speed > FAN_FULL_SPEED_PWM)
    speed = FAN_FULL_SPEED_PWM;

  pwm_duty(fan->pin, speed);
  fan->speed = speed;
}