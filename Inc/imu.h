#ifndef __IMU_H__
#define __IMU_H__

#include "headfile.h"
#include "SEEKFREE_IMU660RA.h"

typedef struct {
  float x_acc;
  float y_acc;
  float z_acc;
  float x_gyro;
  float y_gyro;
  float z_gyro;
} IMUData;

void imu660ra_cali(void);

void imu660ra_get_data(IMUData *imu_data);

extern IMUData imu_data;

extern float acc_x, acc_y, acc_z; // accelerometer data
extern float gyro_x, gyro_y, gyro_z; // gyroscope data

extern float acc_x_err, acc_y_err, acc_z_err; // accelerometer error
extern float gyro_x_err, gyro_y_err, gyro_z_err; // gyroscope error

#endif // __IMU_H__