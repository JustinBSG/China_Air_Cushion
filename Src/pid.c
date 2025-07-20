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

void pid_task1(PIDController *pid, Fan *fans) {
  uint32 current_time = get_tick();
  float error = 0, p_term = 0, i_term = 0, d_term = 0, output = 0;
  uint32 left_fan_speed = 0, right_fan_speed = 0;
  if (pid->kp != PID_TASK1_KP)
    pid->kp = PID_TASK1_KP;
  if (pid->ki != PID_TASK1_KI)
    pid->ki = PID_TASK1_KI;
  if (pid->kd != PID_TASK1_KD)
    pid->kd = PID_TASK1_KD;
  if (pid->setpoint != 0.0f)
    pid->setpoint = 0.0f;

  if (current_time - pid->last_time < PID_MIN_INTERVAL) {
    return;
  }
  pid->last_time = current_time;

  // need to think again
  error = pid->setpoint - imu963ra_get_yaw();

  p_term = pid->kp * error;

  pid->integral += error * PID_MIN_INTERVAL;
  i_term = pid->ki * pid->integral;

  d_term = pid->kd * (error - pid->prev_error) / PID_MIN_INTERVAL;
  pid->prev_error = error;

  output = p_term + i_term + d_term;

  left_fan_speed =  (uint32)output;
  right_fan_speed = (uint32)output;

  if (left_fan_speed < FAN_0_SPEED)
    left_fan_speed = FAN_0_SPEED;
  else if (left_fan_speed > FAN_FULL_SPEED)
    left_fan_speed = FAN_FULL_SPEED;

  if (right_fan_speed < FAN_0_SPEED)
    right_fan_speed = FAN_0_SPEED;
  else if (right_fan_speed > FAN_FULL_SPEED)
    right_fan_speed = FAN_FULL_SPEED;

  fan_set_speed(&fans[2], left_fan_speed);   // Left fan
  fan_set_speed(&fans[3], right_fan_speed);  // Right fan
}

void pid_task2(PIDController *pid, Fan *fans) {
  if (get_tick() - pid->last_time < PID_MIN_INTERVAL) {
    return;
  }
}

void pid_task3(PIDController *pid, Fan *fans) {
  // give up
}

void pid_rotate(PIDController *pid, Fan *fans, int degree) {}