#include "pid.h"

PIDController pid_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0};

void pid_reset(PIDController *pid) {
  pid->kp = 0.0f;
  pid->ki = 0.0f;
  pid->kd = 0.0f;
  pid->setpoint = 0.0f;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->last_time = 0;
}

void pid_task1(PIDController *pid, IMUData *imu_data, Fan *fans) {
  if (get_tick() - pid->last_time < PID_MIN_INTERVAL) {
    return;
  }
  pid->last_time = get_tick();

  imu660ra_get_data(imu_data);

  // need to think again
  float error = pid->setpoint - imu_data->z_gyro;

  float p_term = pid->kp * error;

  pid->integral += error * (PID_MIN_INTERVAL / 1000.0f);
  float i_term = pid->ki * pid->integral;

  float d_term = pid->kd * (error - pid->prev_error) / (PID_MIN_INTERVAL / 1000.0f);
  pid->prev_error = error;

  float output = p_term + i_term + d_term;

  int left_fan_speed = FAN_MID_SPEED_PWM - (int)output;
  int right_fan_speed = FAN_MID_SPEED_PWM + (int)output;

  if (left_fan_speed < FAN_0_SPEED_PWM) 
    left_fan_speed = FAN_0_SPEED_PWM;
  else if (left_fan_speed > FAN_FULL_SPEED_PWM) 
    left_fan_speed = FAN_FULL_SPEED_PWM;
  
  if (right_fan_speed < FAN_0_SPEED_PWM) 
    right_fan_speed = FAN_0_SPEED_PWM;
  else if (right_fan_speed > FAN_FULL_SPEED_PWM) 
    right_fan_speed = FAN_FULL_SPEED_PWM;
  
  fan_set_speed(&fans[2], left_fan_speed); // Left fan
  fan_set_speed(&fans[3], right_fan_speed); // Right fan
}

void pid_task2(PIDController *pid, IMUData *imu_data, Fan *fans) {
  if (get_tick() - pid->last_time < PID_MIN_INTERVAL) {
    return;
  }
}

void pid_task3(PIDController *pid, IMUData *imu_data, Fan *fans) {
  // give up
}

void pid_rotate(PIDController *pid, HMC5883L_Data *hmc5883l_data, HMC5883L_Calibration *hmc5883l_cali_data, Fan *fans, int degree) {}