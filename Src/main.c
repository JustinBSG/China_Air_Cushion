#include "main.h"

#include "Libraries/seekfree_peripheral/headfile.h"
#include "fan.h"
#include "imu.h"
#include "pid.h"
#include "task.h"

#define KEY4_PIN P73
#define BUZZER P67

uint8 key4_status = 1;  // set default button status
uint8 key4_previous_status;
char buf[256];
uint8 count = 0;
uint8 stage = 0;
int time_stamp = 0;

void main() {
  tick_init();  // init tick
  board_init();  // init board
  iic_init(IIC_4, IIC4_SCL_P32, IIC4_SDA_P33, 10);
  uart_init(UART_2, UART2_RX_P10, UART2_TX_P11, 115200, TIM_2);

  BUZZER = 0;               // set default output for buzzer
  gpio_mode(P6_7, GPO_PP);  // set pin p6.7 as push pull mode for large current (>20mA), note all pin default to be standard gpio
  
  initial_all_fan();
  // fan_set_speed(&test_fan, FAN_0_SPEED_PWM);

  // while(imu963ra_init_custom()) {
  //   delay_ms(500);
  //   printf("imu963ra init try again.\r\n");
  // }
  // printf("Starting IMU calibration - keep device still...\r\n");
  // delay_ms(2000);
  // imu963ra_calibrate();
  
  // 启动定时器中断（5ms周期，200Hz采样率）
  pit_timer_ms(TIM_4, 5);

  while (1) {
    printf("get_tick(): %d time_stamp: %d\r\n", get_tick(), time_stamp);
    if (get_tick() - time_stamp > 1000) {
      // printf("get_tick(): %d time_stamp: %d\r\n", get_tick(), time_stamp);
      time_stamp = get_tick();
      stage++;
      if (stage == 4)
        stage = 0;
    }

    switch (stage) {
      case 0:
        fan_set_speed(&fans[0], FAN_MID_SPEED_PWM);
        fan_set_speed(&fans[1], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[2], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[3], FAN_0_SPEED_PWM);
        break;
      case 1:
        fan_set_speed(&fans[0], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[1], FAN_MID_SPEED_PWM);
        fan_set_speed(&fans[2], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[3], FAN_0_SPEED_PWM);
        break;
      case 2:
        fan_set_speed(&fans[0], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[1], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[2], FAN_MID_SPEED_PWM);
        fan_set_speed(&fans[3], FAN_0_SPEED_PWM);
        break;
      case 3:
        fan_set_speed(&fans[0], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[1], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[2], FAN_0_SPEED_PWM);
        fan_set_speed(&fans[3], FAN_MID_SPEED_PWM);
        break;
      default:
        break;
    }

    // printf("Starting IMU calibration - keep device still...\r\n");
    // printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\r\n", 
    //        imu963ra_get_roll(), imu963ra_get_pitch(), imu963ra_get_yaw());
    // // 显示原始IMU数据
    // printf("Acc: X=%d, Y=%d, Z=%d\r\n", 
    //        imu963ra_data.accX, imu963ra_data.accY, imu963ra_data.accZ);
    // printf("Gyro: X=%d, Y=%d, Z=%d\r\n", 
    //        imu963ra_data.gyroX, imu963ra_data.gyroY, imu963ra_data.gyroZ);
    // printf("Mag: X=%d, Y=%d, Z=%d\r\n", 
    //        imu963ra_data.magX, imu963ra_data.magY, imu963ra_data.magZ);
    // printf("---\r\n");
  }
}

void pit_callback(void)
{
	// 使用封装的IMU处理函数
	imu963ra_process();
}