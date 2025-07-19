#include "imu.h"
#include "SEEKFREE_IMU963RA.h"

extern char buf[256];

// 全局变量定义
IMU963RA_Data imu963ra_data = {0};
IMU963RA_Data imu963ra_data_filtered = {0};
IMU963RA_Angle imu963ra_angle = {0};
IMU963RA_Quaternion imu963ra_quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
IMU963RA_Calibration imu963ra_calibration = {0};

// 内部变量
static float integral_error_x = 0.0f;
static float integral_error_y = 0.0f;
static float integral_error_z = 0.0f;
static uint8 init_flag = 0;

/**
 * @brief 初始化IMU963RA传感器
 * @return 0: 成功, 1: 失败
 */
uint8 imu963ra_init_custom(void)
{
    uint8 result = 0;
    
    // 调用底层初始化函数
    result = imu963ra_init();
    
    if (result == 0) {
        // 初始化四元数
        imu963ra_quaternion.q0 = 1.0f;
        imu963ra_quaternion.q1 = 0.0f;
        imu963ra_quaternion.q2 = 0.0f;
        imu963ra_quaternion.q3 = 0.0f;
        
        // 重置积分误差
        integral_error_x = 0.0f;
        integral_error_y = 0.0f;
        integral_error_z = 0.0f;
        
        // 重置校准标志
        imu963ra_calibration.calibrated = 0;
        
        init_flag = 1;
        
        delay_ms(100);
        // sprintf(buf, "IMU963RA initialization successful\r\n");
        // uart_putbuff(UART_2, buf, strlen(buf));
        printf("IMU963RA initialization successful\r\n");
      } else {
        // sprintf(buf, "IMU963RA initialization failed\r\n");
        // uart_putbuff(UART_2, buf, strlen(buf));
        printf("IMU963RA initialization failed\r\n");
    }
    
    return result;
}

/**
 * @brief 读取IMU原始数据
 */
void imu963ra_read_data(void)
{
    if (!init_flag) return;
    
    // 读取加速度数据
    imu963ra_get_acc();
    imu963ra_data.accX = imu963ra_acc_x;
    imu963ra_data.accY = imu963ra_acc_y;
    imu963ra_data.accZ = imu963ra_acc_z;
    
    // 读取陀螺仪数据
    imu963ra_get_gyro();
    imu963ra_data.gyroX = imu963ra_gyro_x;
    imu963ra_data.gyroY = imu963ra_gyro_y;
    imu963ra_data.gyroZ = imu963ra_gyro_z;
    
    // 读取磁力计数据
    imu963ra_get_mag();
    imu963ra_data.magX = imu963ra_mag_x;
    imu963ra_data.magY = imu963ra_mag_y;
    imu963ra_data.magZ = imu963ra_mag_z;
}

/**
 * @brief 校准IMU传感器（计算零偏）
 */
void imu963ra_calibrate(void)
{
    
    
    int32 sum_acc_x = 0, sum_acc_y = 0, sum_acc_z = 0;
    int32 sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
    uint16 i;
	
	if (!init_flag) return;
    // sprintf(buf, "Starting IMU calibration...\r\n");
    // uart_putbuff(UART_2, buf, strlen(buf));
    printf("Starting IMU calibration...\r\n");
    for (i = 0; i < IMU963RA_CALIBRATION_COUNT; i++) {
        imu963ra_read_data();
        
        sum_gyro_x += imu963ra_data.gyroX;
        sum_gyro_y += imu963ra_data.gyroY;
        sum_gyro_z += imu963ra_data.gyroZ;
        
        delay_ms(2);
        
        if (i % 50 == 0) {
            // sprintf(buf, "Calibration progress: %d%%\r\n", (i * 100) / IMU963RA_CALIBRATION_COUNT);
            // uart_putbuff(UART_2, buf, strlen(buf));
            printf("Calibration progress: %d%%\r\n", (i * 100) / IMU963RA_CALIBRATION_COUNT);
        }
    }
    
    // 计算陀螺仪零偏（加速度计不校准零偏，因为重力影响）
    imu963ra_calibration.gyroX_offset = sum_gyro_x / IMU963RA_CALIBRATION_COUNT;
    imu963ra_calibration.gyroY_offset = sum_gyro_y / IMU963RA_CALIBRATION_COUNT;
    imu963ra_calibration.gyroZ_offset = sum_gyro_z / IMU963RA_CALIBRATION_COUNT;
    
    imu963ra_calibration.calibrated = 1;

    // sprintf(buf, "IMU calibration completed\r\n");
    // uart_putbuff(UART_2, buf, strlen(buf));
    printf("IMU calibration completed\r\n");
    // sprintf(buf, "Gyro offsets: X=%d, Y=%d, Z=%d\r\n", 
    //        imu963ra_calibration.gyroX_offset,
    //        imu963ra_calibration.gyroY_offset,
    //        imu963ra_calibration.gyroZ_offset);
    // uart_putbuff(UART_2, buf, strlen(buf));
    printf("Gyro offsets: X=%d, Y=%d, Z=%d\r\n", 
           imu963ra_calibration.gyroX_offset,
           imu963ra_calibration.gyroY_offset,
           imu963ra_calibration.gyroZ_offset);
}

/**
 * @brief 应用校准参数
 */
void imu963ra_apply_calibration(void)
{
    if (!imu963ra_calibration.calibrated) return;
    
    // 只对陀螺仪数据应用零偏校正
    imu963ra_data.gyroX -= imu963ra_calibration.gyroX_offset;
    imu963ra_data.gyroY -= imu963ra_calibration.gyroY_offset;
    imu963ra_data.gyroZ -= imu963ra_calibration.gyroZ_offset;
}

/**
 * @brief 低通滤波器
 */
void imu963ra_low_pass_filter(void)
{
    const float filter_coefficient = 0.1f;
	const float gyro_filter = 0.3f;
    
    // 对加速度数据进行低通滤波
    imu963ra_data_filtered.accX = imu963ra_data_filtered.accX * (1.0f - filter_coefficient) + 
                                  imu963ra_data.accX * filter_coefficient;
    imu963ra_data_filtered.accY = imu963ra_data_filtered.accY * (1.0f - filter_coefficient) + 
                                  imu963ra_data.accY * filter_coefficient;
    imu963ra_data_filtered.accZ = imu963ra_data_filtered.accZ * (1.0f - filter_coefficient) + 
                                  imu963ra_data.accZ * filter_coefficient;
    
    // 对陀螺仪数据进行轻微滤波
    
    imu963ra_data_filtered.gyroX = imu963ra_data_filtered.gyroX * (1.0f - gyro_filter) + 
                                   imu963ra_data.gyroX * gyro_filter;
    imu963ra_data_filtered.gyroY = imu963ra_data_filtered.gyroY * (1.0f - gyro_filter) + 
                                   imu963ra_data.gyroY * gyro_filter;
    imu963ra_data_filtered.gyroZ = imu963ra_data_filtered.gyroZ * (1.0f - gyro_filter) + 
                                   imu963ra_data.gyroZ * gyro_filter;
    
    // 磁力计数据直接复制（可根据需要添加滤波）
    imu963ra_data_filtered.magX = imu963ra_data.magX;
    imu963ra_data_filtered.magY = imu963ra_data.magY;
    imu963ra_data_filtered.magZ = imu963ra_data.magZ;
}

/**
 * @brief 快速平方根函数（牛顿迭代法）
 */
float imu963ra_sqrt(float x)
{
    float result, xhalf;
    int i;
    
    if (x <= 0.0f) return 0.0f;
    
    // 使用标准库的sqrt函数，更可靠
    return sqrt(x);
    
    // 备用的快速算法（如果需要）
    /*
    result = x;
    xhalf = 0.5f * result;
    i = *(int*)&result;
    i = 0x5f3759df - (i >> 1);
    result = *(float*)&i;
    result = result * (1.5f - xhalf * result * result);
    result = result * (1.5f - xhalf * result * result); // 再迭代一次提高精度
    return result * x;
    */
}

/**
 * @brief 四元数归一化
 */
void imu963ra_normalize_quaternion(IMU963RA_Quaternion *q)
{
    float norm, inv_norm;
    
    norm = sqrt(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
    
    if (norm > 0.001f) {  // 增加阈值防止除零
        inv_norm = 1.0f / norm;
        q->q0 *= inv_norm;
        q->q1 *= inv_norm;
        q->q2 *= inv_norm;
        q->q3 *= inv_norm;
    } else {
        // 如果四元数太小，重置为单位四元数
        q->q0 = 1.0f;
        q->q1 = 0.0f;
        q->q2 = 0.0f;
        q->q3 = 0.0f;
    }
}

/**
 * @brief 更新姿态角（使用互补滤波器和四元数）
 */
void imu963ra_update_attitude(void)
{
    // 声明所有局部变量
    float ax, ay, az;
    float gx, gy, gz;
    float acc_norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0_dot, q1_dot, q2_dot, q3_dot;
    float sin_pitch_temp;
    const float RAD_TO_DEG = 180.0f / 3.14159265359f;
    const float DEG_TO_RAD = 3.14159265359f / 180.0f;
    
    if (!init_flag) return;
    
    // 转换为物理单位
    ax = imu963ra_acc_to_g(imu963ra_data_filtered.accX);
    ay = imu963ra_acc_to_g(imu963ra_data_filtered.accY);
    az = imu963ra_acc_to_g(imu963ra_data_filtered.accZ);
    
    gx = imu963ra_gyro_to_dps(imu963ra_data_filtered.gyroX) * DEG_TO_RAD; // 转换为弧度/秒
    gy = imu963ra_gyro_to_dps(imu963ra_data_filtered.gyroY) * DEG_TO_RAD;
    gz = imu963ra_gyro_to_dps(imu963ra_data_filtered.gyroZ) * DEG_TO_RAD;
    
    // 归一化加速度
    acc_norm = sqrt(ax*ax + ay*ay + az*az);
    
    if (acc_norm > 0.1f) {  // 只有在加速度有效时才进行修正
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;
        
        // 从四元数计算预期的重力方向
        vx = 2.0f * (imu963ra_quaternion.q1 * imu963ra_quaternion.q3 - imu963ra_quaternion.q0 * imu963ra_quaternion.q2);
        vy = 2.0f * (imu963ra_quaternion.q0 * imu963ra_quaternion.q1 + imu963ra_quaternion.q2 * imu963ra_quaternion.q3);
        vz = imu963ra_quaternion.q0 * imu963ra_quaternion.q0 - imu963ra_quaternion.q1 * imu963ra_quaternion.q1 - 
             imu963ra_quaternion.q2 * imu963ra_quaternion.q2 + imu963ra_quaternion.q3 * imu963ra_quaternion.q3;
        
        // 计算误差（叉积）
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;
        
        // 积分误差
        integral_error_x += ex * IMU963RA_KI * IMU963RA_SAMPLE_TIME;
        integral_error_y += ey * IMU963RA_KI * IMU963RA_SAMPLE_TIME;
        integral_error_z += ez * IMU963RA_KI * IMU963RA_SAMPLE_TIME;
        
        // 调整陀螺仪测量值
        gx += IMU963RA_KP * ex + integral_error_x;
        gy += IMU963RA_KP * ey + integral_error_y;
        gz += IMU963RA_KP * ez + integral_error_z;
    }
    
    // 四元数微分方程
    q0_dot = 0.5f * (-imu963ra_quaternion.q1 * gx - imu963ra_quaternion.q2 * gy - imu963ra_quaternion.q3 * gz);
    q1_dot = 0.5f * ( imu963ra_quaternion.q0 * gx + imu963ra_quaternion.q2 * gz - imu963ra_quaternion.q3 * gy);
    q2_dot = 0.5f * ( imu963ra_quaternion.q0 * gy - imu963ra_quaternion.q1 * gz + imu963ra_quaternion.q3 * gx);
    q3_dot = 0.5f * ( imu963ra_quaternion.q0 * gz + imu963ra_quaternion.q1 * gy - imu963ra_quaternion.q2 * gx);
    
    // 积分四元数
    imu963ra_quaternion.q0 += q0_dot * IMU963RA_SAMPLE_TIME;
    imu963ra_quaternion.q1 += q1_dot * IMU963RA_SAMPLE_TIME;
    imu963ra_quaternion.q2 += q2_dot * IMU963RA_SAMPLE_TIME;
    imu963ra_quaternion.q3 += q3_dot * IMU963RA_SAMPLE_TIME;
    
    // 归一化四元数
    imu963ra_normalize_quaternion(&imu963ra_quaternion);
    
    // 从四元数计算欧拉角，使用更安全的方法
    // Roll (绕X轴旋转)
    imu963ra_angle.roll = atan2(2.0f * (imu963ra_quaternion.q0 * imu963ra_quaternion.q1 + imu963ra_quaternion.q2 * imu963ra_quaternion.q3),
                                1.0f - 2.0f * (imu963ra_quaternion.q1 * imu963ra_quaternion.q1 + imu963ra_quaternion.q2 * imu963ra_quaternion.q2)) * RAD_TO_DEG;
    
    // Pitch (绕Y轴旋转) - 使用安全的asin
    sin_pitch_temp = 2.0f * (imu963ra_quaternion.q0 * imu963ra_quaternion.q2 - imu963ra_quaternion.q3 * imu963ra_quaternion.q1);
    if (sin_pitch_temp > 1.0f) sin_pitch_temp = 1.0f;
    if (sin_pitch_temp < -1.0f) sin_pitch_temp = -1.0f;
    imu963ra_angle.pitch = asin(sin_pitch_temp) * RAD_TO_DEG;
    
    // Yaw (绕Z轴旋转)
    imu963ra_angle.yaw = atan2(2.0f * (imu963ra_quaternion.q0 * imu963ra_quaternion.q3 + imu963ra_quaternion.q1 * imu963ra_quaternion.q2),
                               1.0f - 2.0f * (imu963ra_quaternion.q2 * imu963ra_quaternion.q2 + imu963ra_quaternion.q3 * imu963ra_quaternion.q3)) * RAD_TO_DEG;
}

/**
 * @brief 获取横滚角
 */
float imu963ra_get_roll(void)
{
    return imu963ra_angle.roll;
}

/**
 * @brief 获取俯仰角
 */
float imu963ra_get_pitch(void)
{
    return imu963ra_angle.pitch;
}

/**
 * @brief 获取偏航角
 */
float imu963ra_get_yaw(void)
{
    return imu963ra_angle.yaw;
}

/**
 * @brief 获取四元数
 */
void imu963ra_get_quaternion(IMU963RA_Quaternion *q)
{
    q->q0 = imu963ra_quaternion.q0;
    q->q1 = imu963ra_quaternion.q1;
    q->q2 = imu963ra_quaternion.q2;
    q->q3 = imu963ra_quaternion.q3;
}

/**
 * @brief 重置姿态解算
 */
void imu963ra_reset_attitude(void)
{
    imu963ra_quaternion.q0 = 1.0f;
    imu963ra_quaternion.q1 = 0.0f;
    imu963ra_quaternion.q2 = 0.0f;
    imu963ra_quaternion.q3 = 0.0f;
    
    imu963ra_angle.roll = 0.0f;
    imu963ra_angle.pitch = 0.0f;
    imu963ra_angle.yaw = 0.0f;
    
    integral_error_x = 0.0f;
    integral_error_y = 0.0f;
    integral_error_z = 0.0f;
}

/**
 * @brief 将加速度原始数据转换为g
 */
float imu963ra_acc_to_g(int16 acc_value)
{
    // 根据量程设置转换（假设±8g量程）
    return (float)acc_value / 4098.0f;
}

/**
 * @brief 将陀螺仪原始数据转换为°/s
 */
float imu963ra_gyro_to_dps(int16 gyro_value)
{
    // 根据量程设置转换（假设±2000dps量程）
    return (float)gyro_value / 14.3f;
}

/**
 * @brief 将磁力计原始数据转换为高斯
 */
float imu963ra_mag_to_gauss(int16 mag_value)
{
    // 根据量程设置转换（假设8G量程）
    return (float)mag_value / 3000.0f;
}

/**
 * @brief IMU数据处理主函数（在定时器中断中调用）
 */
void imu963ra_process(void)
{
    // 读取原始数据
    imu963ra_read_data();
    
    // 应用校准
    imu963ra_apply_calibration();
    
    // 低通滤波
    imu963ra_low_pass_filter();
    
    // // 使用简单的互补滤波器方法（更稳定）
    // imu963ra_simple_attitude_update();
    
    // 如果需要四元数方法，可以调用：
    imu963ra_update_attitude();
}

/**
 * @brief 简单的互补滤波器姿态解算（备用方法）
 */
void imu963ra_simple_attitude_update(void)
{
    float ax, ay, az;
    float gx, gy, gz;
    float acc_roll, acc_pitch;
    static float gyro_roll = 0.0f, gyro_pitch = 0.0f, gyro_yaw = 0.0f;
    const float alpha = 0.98f;  // 互补滤波器系数
    const float RAD_TO_DEG = 180.0f / M_PI;
    const float DEG_TO_RAD = M_PI / 180.0f;
    
    if (!init_flag) return;
    
    // 转换为物理单位
    ax = imu963ra_acc_to_g(imu963ra_data_filtered.accX);
    ay = imu963ra_acc_to_g(imu963ra_data_filtered.accY);
    az = imu963ra_acc_to_g(imu963ra_data_filtered.accZ);
    
    gx = imu963ra_gyro_to_dps(imu963ra_data_filtered.gyroX);
    gy = imu963ra_gyro_to_dps(imu963ra_data_filtered.gyroY);
    gz = imu963ra_gyro_to_dps(imu963ra_data_filtered.gyroZ);
    
    // 从加速度计计算角度
    acc_roll = atan2(ay, az) * RAD_TO_DEG;
    acc_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
    
    // 陀螺仪积分
    gyro_roll += gx * IMU963RA_SAMPLE_TIME;
    gyro_pitch += gy * IMU963RA_SAMPLE_TIME;
    gyro_yaw += gz * IMU963RA_SAMPLE_TIME;
    
    // 互补滤波器
    imu963ra_angle.roll = alpha * gyro_roll + (1.0f - alpha) * acc_roll;
    imu963ra_angle.pitch = alpha * gyro_pitch + (1.0f - alpha) * acc_pitch;
    imu963ra_angle.yaw = gyro_yaw;  // 偏航角只能靠陀螺仪
    
    // 更新陀螺仪积分值
    gyro_roll = imu963ra_angle.roll;
    gyro_pitch = imu963ra_angle.pitch;
}