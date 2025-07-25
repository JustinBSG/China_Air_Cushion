#ifndef __PID_H__
#define __PID_H__

#include "fan.h"
#include "headfile.h"
#include "hmc5883l.h"
#include "imu.h"
#include "tick.h"

// maybe need to reduce
#define PID_MIN_INTERVAL 10  // Minimum interval for PID tasks in milliseconds

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_error;
    uint32 last_time;
} PIDController;

// TODO: need to tune
#define PID_TASK1_KP 0.0
#define PID_TASK1_KI 0.0
#define PID_TASK1_KD 0.0

#define PID_TASK2_KP 0.0
#define PID_TASK2_KI 0.0
#define PID_TASK2_KD 0.0

#define PID_TASK3_KP 0.0
#define PID_TASK3_KI 0.0
#define PID_TASK3_KD 0.0

#define PID_ROTATE_KP 0.0
#define PID_ROTATE_KI 0.0
#define PID_ROTATE_KD 0.0

void pid_reset(PIDController *pid);

void pid_task1(PIDController *pid, Fan *fans);

void pid_task2(PIDController *pid, Fan *fans);

void pid_task3(PIDController *pid, Fan *fans);

void pid_rotate(PIDController *pid, Fan *fans, int degree);

extern PIDController pid_data;

#endif  // __PID_H__