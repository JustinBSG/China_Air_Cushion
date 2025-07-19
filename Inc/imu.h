#ifndef __IMU_H__
#define __IMU_H__

#include "headfile.h"
#include <math.h>

// 确保数学常量定义
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// IMU数据结构定义
typedef struct
{
    int16 accX;
    int16 accY;
    int16 accZ;
    int16 gyroX;
    int16 gyroY;
    int16 gyroZ;
    int16 magX;
    int16 magY;
    int16 magZ;
} IMU963RA_Data;

// 姿态角数据结构
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} IMU963RA_Angle;

// 四元数数据结构
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} IMU963RA_Quaternion;

// IMU校准数据结构
typedef struct
{
    int16 accX_offset;
    int16 accY_offset;
    int16 accZ_offset;
    int16 gyroX_offset;
    int16 gyroY_offset;
    int16 gyroZ_offset;
    uint8 calibrated;
} IMU963RA_Calibration;

// 外部变量声明
extern IMU963RA_Data imu963ra_data;
extern IMU963RA_Data imu963ra_data_filtered;
extern IMU963RA_Angle imu963ra_angle;
extern IMU963RA_Quaternion imu963ra_quaternion;
extern IMU963RA_Calibration imu963ra_calibration;

// 函数声明
uint8 imu963ra_init_custom(void);
void imu963ra_read_data(void);
void imu963ra_calibrate(void);
void imu963ra_apply_calibration(void);
void imu963ra_update_attitude(void);
void imu963ra_low_pass_filter(void);
float imu963ra_get_roll(void);
float imu963ra_get_pitch(void);
float imu963ra_get_yaw(void);
void imu963ra_get_quaternion(IMU963RA_Quaternion *q);
void imu963ra_reset_attitude(void);

// 数据转换函数
float imu963ra_acc_to_g(int16 acc_value);
float imu963ra_gyro_to_dps(int16 gyro_value);
float imu963ra_mag_to_gauss(int16 mag_value);

// 工具函数
float imu963ra_sqrt(float x);
void imu963ra_normalize_quaternion(IMU963RA_Quaternion *q);

// 主处理函数
void imu963ra_process(void);
void imu963ra_simple_attitude_update(void);

// 配置参数
#define IMU963RA_SAMPLE_RATE_HZ     200.0f      // 采样频率
#define IMU963RA_SAMPLE_TIME        (1.0f / IMU963RA_SAMPLE_RATE_HZ)
#define IMU963RA_CALIBRATION_COUNT  500         // 校准采样次数
#define IMU963RA_FILTER_ALPHA       0.98f       // 互补滤波器系数
#define IMU963RA_GYRO_NOISE_THRESHOLD 5         // 陀螺仪噪声阈值

// PID控制参数
#define IMU963RA_KP                 2.0f
#define IMU963RA_KI                 0.005f

#endif // __IMU_H__
