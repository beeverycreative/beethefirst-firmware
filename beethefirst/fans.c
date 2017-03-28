
#include "fans.h"

uint32_t  currenBWSpeed = 0;


void enableBlower(void)
{
  currenBWSpeed = 100;
#if !defined(EXP_Board) && !defined(BTF_SMOOTHIE)
  extruder_fan_on();
#endif
#if defined(EXP_Board)
#ifndef BTF_SMOOTHIE
  blower_on();
#endif
  pwm_set_duty_cycle(BW_PWM_CHANNEL,100);
  pwm_set_enable(BW_PWM_CHANNEL);
#endif
}

void disableBlower(void)
{
  currenBWSpeed = 0;
#if !defined(EXP_Board) && !defined(BTF_SMOOTHIE)
  extruder_fan_off();
#endif
#if defined(EXP_Board)
#ifndef BTF_SMOOTHIE
  blower_off();
#endif
  pwm_set_duty_cycle(BW_PWM_CHANNEL,0);
  pwm_set_enable(BW_PWM_CHANNEL);
#endif
}

void setBlowerSpeed(int16_t speed)
{
#if defined(EXP_Board)
#ifndef BTF_SMOOTHIE
  blower_on();
#endif

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
