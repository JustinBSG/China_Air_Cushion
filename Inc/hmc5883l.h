#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include "headfile.h"
#include "main.h"
#include "tick.h"
#include "zf_delay.h"
#include "zf_iic.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HMC5883L_ADDR 0x1E
#define HMC5883L_REG_ADDR_CONFA 0x00
#define HMC5883L_REG_ADDR_CONFB 0x01
#define HMC5883L_REG_ADDR_MODE 0x02
#define HMC5883L_REG_ADDR_X_MSB 0x03
#define HMC5883L_REG_ADDR_X_LSB 0x04
#define HMC5883L_REG_ADDR_Z_MSB 0x05
#define HMC5883L_REG_ADDR_Z_LSB 0x06
#define HMC5883L_REG_ADDR_Y_MSB 0x07
#define HMC5883L_REG_ADDR_Y_LSB 0x08
#define HMC5883L_REG_ADDR_STATUS 0x09
#define HMC5883L_REG_ADDR_IDA 0x0A
#define HMC5883L_REG_ADDR_IDB 0x0B
#define HMC5883L_REG_ADDR_IDC 0x0C

typedef struct {
    int16 x;
    int16 y;
    int16 z;
} HMC5883L_Data;

typedef struct {
    int16 x_min;
    int16 x_max;
    int16 y_min;
    int16 y_max;
    int16 z_min;
    int16 z_max;
    int16 x_offset;
    int16 y_offset;
    int16 z_offset;
    int start_time;
} HMC5883L_Calibration;

void hmc5883l_init(void);
void hmc5883l_write(uint8 reg, uint8* dat);
void hmc5883l_read(uint8 reg, uint8* dat);

bool hmc5883l_is_data_ready(void);
void hmc5883l_read_data(HMC5883L_Data* dat);
float hmc5883l_cal_xy_angle(HMC5883L_Data* dat, HMC5883L_Calibration* cali_data);
void hmc5883l_calibrate(HMC5883L_Calibration* cali_data);

extern HMC5883L_Data hmc5883l_data;
extern HMC5883L_Calibration hmc5883l_cali_data;

#endif  // __HMC5883L_H__