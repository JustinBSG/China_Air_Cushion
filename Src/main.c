#include "Libraries/seekfree_peripheral/headfile.h"
#include "main.h"
#include "fan.h"
#include "SEEKFREE_IMU660RA.h"
#include "imu.h"

#define KEY4_PIN P73
#define BUZZER P67

uint8 key4_status = 1; // set default button status
uint8 key4_previous_status;
char buf[256];
uint8 count = 0;

void main() {
  board_init(); // init board

  BUZZER = 0;              // set default output for buzzer
  gpio_mode(P6_7, GPO_PP); // set pin p6.7 as push pull mode for large current (>20mA), note all pin default to be standard gpio

  // pwm_init(TEST_PWM, 50, 0);
  // pwm_duty(TEST_PWM, PWM_DUTY_MAX*0.05*2);
  initial_all_fan();
  // fan_set_speed(&test_fan, FAN_0_SPEED_PWM);

  uart_init(UART_2, UART2_RX_P10, UART2_TX_P11, 115200, TIM_2);
  imu660ra_init();
  imu660ra_cali();
  sprintf(buf, "acc_x_err: %.2f, acc_y_err: %.2f, acc_z_err: %.2f, gyro_x_err: %.2f, gyro_y_err: %.2f, gyro_z_err: %.2f\n",
          acc_x_err, acc_y_err, acc_z_err, gyro_x_err, gyro_y_err, gyro_z_err);
  uart_putbuff(UART_2, buf, strlen(buf));

  while (1) {
    // key4_previous_status = key4_status;
    // key4_status = KEY4_PIN; // read button

    // if (key4_previous_status && !key4_status) // capured falling edge
    // {
    //   delay_ms(10); // debounce
    //   key4_status = KEY4_PIN;
      
    //   if (!key4_status)
    //   {
    //     BUZZER = !BUZZER; // turn on/off the buzzer
    //   }
    // }

    sprintf(buf, "acc_x: %.2f g, acc_y: %.2f g, acc_z: %.2f g, gyro_x: %.2f °/s, gyro_y: %.2f °/s, gyro_z: %.2f °/s\n",
        imu_data.x_acc, imu_data.y_acc, imu_data.z_acc, imu_data.x_gyro, imu_data.y_gyro, imu_data.z_gyro);
    uart_putbuff(UART_2, buf, strlen(buf));
    delay_ms(1000);
  }
}