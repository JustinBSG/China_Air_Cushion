#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"
#include "tick.h"
#include "imu.h"
#include "fan.h"

#define PID_MIN_INTERVAL 10 // Minimum interval for PID tasks in milliseconds

typedef struct {
  float kp;
  float ki;
  float kd;
  float setpoint;
  float integral;
  float prev_error;
  uint32 last_time;
} PIDController;

extern PIDController pid_data;

void pid_reset(PIDController *pid);

void pid_task1(PIDController *pid, IMUData *imu_data, Fan *fans);

void pid_task2(PIDController *pid, IMUData *imu_data, Fan *fans);

void pid_task3(PIDController *pid, IMUData *imu_data, Fan *fans);

#endif // __PID_H__