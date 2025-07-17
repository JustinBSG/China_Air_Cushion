#include "hmc5883l.h"

#include <math.h>

HMC5883L_Data hmc5883l_data = {0, 0, 0};
HMC5883L_Calibration hmc5883l_cali_data = {0, 0, 0, 0, 0, 0};

void hmc5883l_init(void) {
  uint8 data[3] = {0x70, 0xA0, 0x00};
  hmc5883l_write(HMC5883L_REG_ADDR_CONFA, &(data[0]));
  hmc5883l_write(HMC5883L_REG_ADDR_CONFB, &(data[1]));
  hmc5883l_write(HMC5883L_REG_ADDR_MODE, &(data[2]));
}

void hmc5883l_write(uint8 reg, uint8* data) {
  // HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR << 1, reg, 1, data, 1, HAL_MAX_DELAY);
  iic_write_reg(HMC5883L_ADDR, reg, *data);
}

void hmc5883l_read(uint8 reg, uint8* data) {
  // HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR << 1, reg, 1, data, 1, HAL_MAX_DELAY);
  iic_read_reg(HMC5883L_ADDR, reg, data);
}

bool hmc5883l_is_data_ready(void) {
  uint8 status;
  hmc5883l_read(HMC5883L_REG_ADDR_STATUS, &status);
  return (status & 0x01);
}

void hmc5883l_read_data(HMC5883L_Data* data) {
  uint8 buffer[6];
  hmc5883l_read(HMC5883L_REG_ADDR_X_MSB, &(buffer[0]));
  hmc5883l_read(HMC5883L_REG_ADDR_X_LSB, &(buffer[1]));
  hmc5883l_read(HMC5883L_REG_ADDR_Y_MSB, &(buffer[2]));
  hmc5883l_read(HMC5883L_REG_ADDR_Y_LSB, &(buffer[3]));
  hmc5883l_read(HMC5883L_REG_ADDR_Z_MSB, &(buffer[4]));
  hmc5883l_read(HMC5883L_REG_ADDR_Z_LSB, &(buffer[5]));
  data->x = (int16)(buffer[0] << 8 | buffer[1]);
  data->y = (int16)(buffer[2] << 8 | buffer[3]);
  data->z = (int16)(buffer[4] << 8 | buffer[5]);
}

float hmc5883l_cal_xy_angle(const HMC5883L_Data* const data, const HMC5883L_Calibration* const cali_data) {
  return atan2(data->y - cali_data->y_offset, data->x - cali_data->x_offset) * 180.0 / M_PI + 180.0;
}

void hmc5883l_calibrate(HMC5883L_Calibration* cali_data) {
  cali_data->start_time = HAL_GetTick();
  while (get_tick() - cali_data->start_time <= 10000) {
    HMC5883L_Data data;
    hmc5883l_read_data(&data);
    if (data.x < cali_data->x_min)
      cali_data->x_min = data.x;
    if (data.x > cali_data->x_max)
      cali_data->x_max = data.x;
    if (data.y < cali_data->y_min)
      cali_data->y_min = data.y;
    if (data.y > cali_data->y_max)
      cali_data->y_max = data.y;
    if (data.z < cali_data->z_min)
      cali_data->z_min = data.z;
    if (data.z > cali_data->z_max)
      cali_data->z_max = data.z;
    delay_ms(100);
  }
  cali_data->x_offset = (cali_data->x_max + cali_data->x_min) / 2;
  cali_data->y_offset = (cali_data->y_max + cali_data->y_min) / 2;
  cali_data->z_offset = (cali_data->z_max + cali_data->z_min) / 2;
}