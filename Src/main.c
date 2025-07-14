#include "Libraries/seekfree_peripheral/headfile.h" // lib provided by Taobao
#include "main.h"
#include "fan.h"

#define KEY4_PIN P73
#define BUZZER P67

uint8 key4_status = 1; // set default button status
uint8 key4_previous_status;

void main()
{
  board_init(); // init board

  BUZZER = 0;              // set default output for buzzer
  gpio_mode(P6_7, GPO_PP); // set pin p6.7 as push pull mode for large current (>20mA), note all pin default to be standard gpio

  // pwm_init(TEST_PWM, 50, 0);
  // pwm_duty(TEST_PWM, PWM_DUTY_MAX*0.05*2);
  initial_all_fan();
  // fan_set_speed(&test_fan, FAN_0_SPEED_PWM);

  while (1)
  {
    key4_previous_status = key4_status;
    key4_status = KEY4_PIN; // read button

    if (key4_previous_status && !key4_status) // capured falling edge
    {
      delay_ms(10); // debounce
      key4_status = KEY4_PIN;
      
      if (!key4_status)
      {
        BUZZER = !BUZZER; // turn on/off the buzzer
      }
    }
  }
}