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

}

void pid_task2(PIDController *pid, IMUData *imu_data, Fan *fans) {}

void pid_task3(PIDController *pid, IMUData *imu_data, Fan *fans) {
  // give up
}