#include "fan.h"

// TODO: Add array of 6 fans
Fan fans[4] = {{FAN_0_SPEED, PWMB_CH1_P20}, {FAN_0_SPEED, PWMB_CH2_P21}, {FAN_0_SPEED, PWMB_CH4_P23}, {FAN_0_SPEED, PWMB_CH3_P22}};
Fan test_fan = {FAN_0_SPEED, PWMA_CH4P_P66};

void initial_all_fan(void) {
  pwm_init(PWMB_CH1_P20, FAN_FREQ, FAN_MIN_SPEED_PWM);
  pwm_duty(PWMB_CH1_P20, FAN_0_SPEED);
  pwm_init(PWMB_CH2_P21, FAN_FREQ, FAN_MIN_SPEED_PWM);
  pwm_duty(PWMB_CH2_P21, FAN_0_SPEED);
  pwm_init(PWMB_CH3_P22, FAN_FREQ, FAN_MIN_SPEED_PWM);
  pwm_duty(PWMB_CH3_P22, FAN_0_SPEED);
  pwm_init(PWMB_CH4_P23, FAN_FREQ, FAN_MIN_SPEED_PWM);
  pwm_duty(PWMB_CH4_P23, FAN_0_SPEED);
}

void fan_set_speed(Fan *fan, uint32 speed) {
  if (speed < 0)
    speed = 0;
  else if (speed > FAN_FULL_SPEED) 
    speed = FAN_FULL_SPEED;
  
  fan->speed = FAN_MIN_SPEED_PWM + (speed * (FAN_MAX_SPEED_PWM - FAN_MIN_SPEED_PWM) / FAN_MAX_SPEED_PWM);  // Scale to PWM duty cycle
  pwm_duty(fan->pin, fan->speed);
}