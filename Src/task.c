#include "task.h"

uint8 task1_stage = 0;
uint8 task2_stage = 0;
uint8 task3_stage = 0;

void task1(void) {
  switch (task1_stage) {
    case 0:
      // walk straight line
      // if () // maybe timer expired
      //   task1_stage = 1; // move to next stage

      pid_task1(&pid_data, &imu_data, fans);
      break;
    case 1:
      // rotate 180 degree
      // maybe use hmc to get angle
      // maybe use pid to control fan speed to rotate
      break;
    case 2:
      // walk straight line
      // if () // maybe timer expired
      //   task1_stage = 3; // move to next stage

      pid_task1(&pid_data, &imu_data, fans);
    default:
      break;
  }
}

void task2(void) {
  switch (task2_stage) {
    case 0:
      // walk S curve
      break;
    case 1:
      // rotate 180 degree
      break;
    case 2:
      // walk S curve
    default:
      break;
  }
}

void task3(void) {
  // give up la
}