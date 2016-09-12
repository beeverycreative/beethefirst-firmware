#include "lights.h"

#ifdef EXP_Board
  bool      start_logo_blink = false;      // start logo blink
  bool      stop_logo_blink = true;      // stop logo blink
  bool      logo_state = false;           // logo state
  uint32_t  blink_interval = 2000;

  bool shutDownLogoState = 0;
  int32_t logoDuty = 0;
#endif

void LogoLightControl(void)
{
#ifdef EXP_Board
  //Logo/LEDs Blink
  if(printerShutdown)
    {
      ShutdownLightControl();
    }
  else
    {
      //Turn Logo On
      setLogoPWM(100);
      ilum_on();
    }
#endif
}

#ifdef EXP_Board
void setLogoPWM(int32_t val)
{
  if(val != logoDuty)
    {
      logoDuty = val;
      pwm_set_duty_cycle(LOGO_PWM_CHANNEL,logoDuty);
      pwm_set_enable(LOGO_PWM_CHANNEL);
    }
}


void ShutdownLightControl(void)
{

  if(blink_time > blink_interval)
    {
      if(shutDownLogoState) {
          setLogoPWM(0);
          shutDownLogoState = 0;
          ilum_off();
      } else {
          setLogoPWM(50);
          shutDownLogoState = 1;
          ilum_on();
      }

      blink_time = 0;

    }
}

#endif


