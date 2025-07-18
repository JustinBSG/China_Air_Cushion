#include "hmc5883l.h"

#include <math.h>

HMC5883L_Data hmc5883l_data = {0, 0, 0};
HMC5883L_Calibration hmc5883l_cali_data = {0, 0, 0, 0, 0, 0};

void hmc5883l_init(void) {
  uint8 dat[3] = {0x70, 0xA0, 0x00};
  hmc5883l_write(HMC5883L_REG_ADDR_CONFA, &(dat[0]));
  hmc5883l_write(HMC5883L_REG_ADDR_CONFB, &(dat[1]));
  hmc5883l_write(HMC5883L_REG_ADDR_MODE, &(dat[2]));
}

void hmc5883l_write(uint8 reg, uint8* dat) {
  // HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR << 1, reg, 1, data, 1, HAL_MAX_DELAY);
  iic_write_reg(HMC5883L_ADDR, reg, *dat);
}

void hmc5883l_read(uint8 reg, uint8* dat) {
  // HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR << 1, reg, 1, data, 1, HAL_MAX_DELAY);
  iic_read_reg(HMC5883L_ADDR, reg, dat);
}

bool hmc5883l_is_data_ready(void) {
  uint8 status;
  hmc5883l_read(HMC5883L_REG_ADDR_STATUS, &status);
  return (status & 0x01);
}

void hmc5883l_read_data(HMC5883L_Data* dat) {
  uint8 buffer[6];
  hmc5883l_read(HMC5883L_REG_ADDR_X_MSB, &(buffer[0]));
  hmc5883l_read(HMC5883L_REG_ADDR_X_LSB, &(buffer[1]));
  hmc5883l_read(HMC5883L_REG_ADDR_Y_MSB, &(buffer[2]));
  hmc5883l_read(HMC5883L_REG_ADDR_Y_LSB, &(buffer[3]));
  hmc5883l_read(HMC5883L_REG_ADDR_Z_MSB, &(buffer[4]));
  hmc5883l_read(HMC5883L_REG_ADDR_Z_LSB, &(buffer[5]));
  dat->x = (int16)(buffer[0] << 8 | buffer[1]);
  dat->y = (int16)(buffer[2] << 8 | buffer[3]);
  dat->z = (int16)(buffer[4] << 8 | buffer[5]);
}

float hmc5883l_cal_xy_angle(HMC5883L_Data* dat, HMC5883L_Calibration* cali_data) {
  return atan2(dat->y - cali_data->y_offset, dat->x - cali_data->x_offset) * 180.0 / M_PI + 180.0;
}

void hmc5883l_calibrate(HMC5883L_Calibration* cali_data) {
  cali_data->start_time = HAL_GetTick();
  while (get_tick() - cali_data->start_time <= 10000) {
    HMC5883L_Data dat;
    hmc5883l_read_data(&dat);
    if (dat.x < cali_data->x_min)
      cali_data->x_min = dat.x;
    if (dat.x > cali_data->x_max)
      cali_data->x_max = dat.x;
    if (dat.y < cali_data->y_min)
      cali_data->y_min = dat.y;
    if (dat.y > cali_data->y_max)
      cali_data->y_max = dat.y;
    if (dat.z < cali_data->z_min)
      cali_data->z_min = dat.z;
    if (dat.z > cali_data->z_max)
      cali_data->z_max = dat.z;
    delay_ms(100);
  }
  cali_data->x_offset = (cali_data->x_max + cali_data->x_min) / 2;
  cali_data->y_offset = (cali_data->y_max + cali_data->y_min) / 2;
  cali_data->z_offset = (cali_data->z_max + cali_data->z_min) / 2;
}