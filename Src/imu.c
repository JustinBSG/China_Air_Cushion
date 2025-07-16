#include "imu.h"
#include "SEEKFREE_IMU660RA.h"
#include "zf_delay.h"

float acc_x, acc_y, acc_z; // accelerometer data
float gyro_x, gyro_y, gyro_z; // gyroscope data

float acc_x_err, acc_y_err, acc_z_err; // accelerometer error
float gyro_x_err, gyro_y_err, gyro_z_err; // gyroscope error

IMUData imu_data; 

void imu660ra_cali(void) {
  int sample_count = 1000;
  float acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
  float gyro_x_est = 0, gyro_y_est = 0, gyro_z_est = 0;
  float gyro_x_p = 1, gyro_y_p = 1, gyro_z_p = 1; // initial error covariance
  float q = 0.01; // process noise covariance
  float r = 0.1;  // measurement noise covariance
  int i = 0;

  for (i = 0; i < sample_count; i++) {
    imu660ra_get_acc();
    imu660ra_get_gyro();

    // Accumulate accelerometer data
    acc_x_sum += imu660ra_acc_transition(imu660ra_acc_x);
    acc_y_sum += imu660ra_acc_transition(imu660ra_acc_y);
    acc_z_sum += imu660ra_acc_transition(imu660ra_acc_z);

    // Kalman filter for gyroscope data

    // Update estimates for gyro_x
    gyro_x_p += q;
    gyro_x_est += (gyro_x_p / (gyro_x_p + r)) * (imu660ra_gyro_transition(imu660ra_gyro_x) - gyro_x_est);
    gyro_x_p *= (1 - (gyro_x_p / (gyro_x_p + r)));

    // Update estimates for gyro_y
    gyro_y_p += q;
    gyro_y_est += (gyro_y_p / (gyro_y_p + r)) * (imu660ra_gyro_transition(imu660ra_gyro_y) - gyro_y_est);
    gyro_y_p *= (1 - (gyro_y_p / (gyro_y_p + r)));

    // Update estimates for gyro_z
    gyro_z_p += q;
    gyro_z_est += (gyro_z_p / (gyro_z_p + r)) * (imu660ra_gyro_transition(imu660ra_gyro_z) - gyro_z_est);
    gyro_z_p *= (1 - (gyro_z_p / (gyro_z_p + r)));

    delay_ms(10); // delay between samples
  }

  // Calculate mean values for accelerometer errors
  acc_x_err = acc_x_sum / sample_count;
  acc_y_err = acc_y_sum / sample_count;
  acc_z_err = acc_z_sum / sample_count;

  // Assign final estimates for gyroscope errors
  gyro_x_err = gyro_x_est;
  gyro_y_err = gyro_y_est;
  gyro_z_err = gyro_z_est;
}

void imu660ra_get_data(IMUData *imu_data) {
  imu660ra_get_acc(); // get accelerometer data
  imu660ra_get_gyro(); // get gyroscope data
  acc_x = imu660ra_acc_transition(imu660ra_acc_x); // convert to physical data
  acc_y = imu660ra_acc_transition(imu660ra_acc_y);
  acc_z = imu660ra_acc_transition(imu660ra_acc_z);
  gyro_x = imu660ra_gyro_transition(imu660ra_gyro_x); // convert to physical data
  gyro_y = imu660ra_gyro_transition(imu660ra_gyro_y);
  gyro_z = imu660ra_gyro_transition(imu660ra_gyro_z);

  imu_data->x_acc = acc_x - acc_x_err;
  imu_data->y_acc = acc_y - acc_y_err;
  imu_data->z_acc = acc_z - acc_z_err;
  imu_data->x_gyro = gyro_x - gyro_x_err;
  imu_data->y_gyro = gyro_y - gyro_y_err;
  imu_data->z_gyro = gyro_z - gyro_z_err;
}