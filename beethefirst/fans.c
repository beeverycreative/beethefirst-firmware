
#include "fans.h"

uint32_t  currenBWSpeed = 0;


void enableBlower(void)
{
  currenBWSpeed = 100;
#ifndef EXP_Board
  extruder_fan_on();
#endif
#ifdef EXP_Board
  blower_on();
  pwm_set_duty_cycle(BW_PWM_CHANNEL,100);
  pwm_set_enable(BW_PWM_CHANNEL);
#endif
}

void disableBlower(void)
{
  currenBWSpeed = 0;
#ifndef EXP_Board
  extruder_fan_off();
#endif
#ifdef EXP_Board
  blower_off();
  pwm_set_duty_cycle(BW_PWM_CHANNEL,0);
  pwm_set_disable(BW_PWM_CHANNEL);
#endif
}

void setBlowerSpeed(int16_t speed)
{
#ifdef EXP_Board
  blower_on();

  uint16_t s_val = speed;
  uint16_t duty = 0;
  if(s_val >= 255)
    {
      duty = 100;
    } else {
        duty = (uint16_t) s_val*0.4;
    }

  currenBWSpeed = duty;
  pwm_set_duty_cycle(BW_PWM_CHANNEL,duty);
  pwm_set_enable(BW_PWM_CHANNEL);
#endif
}
