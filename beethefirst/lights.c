#include "lights.h"

#ifdef EXP_Board
  bool      start_logo_blink = false;      // start logo blink
  bool      stop_logo_blink = true;      // stop logo blink
  bool      logo_state = false;           // logo state
  uint32_t  blink_interval = 20000;

  uint32_t shutDownLogoState = 0;
  int32_t logoDuty = 0;
#endif

void LogoLightControl(void)
{
#ifdef EXP_Board
  //Logo Blink
  if(in_power_saving && !printerShutdown)
    {
      PowerSavingLightControl();
    }
  else if(printerShutdown)
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

void setLogoPWM(int32_t val)
{
  if(val != logoDuty)
    {
      logoDuty = val;
      pwm_set_duty_cycle(LOGO_PWM_CHANNEL,logoDuty);
      pwm_set_enable(LOGO_PWM_CHANNEL);
    }
}

#ifdef EXP_Board
void PowerSavingLightControl(void)
{
  ilum_off();
  if(start_logo_blink && (blink_time > blink_interval))
    {
        if(logo_state) {
            setLogoPWM(0);
            logo_state = 0;
        } else {
            setLogoPWM(25);
            logo_state = 1;
        }

        blink_time = 0;

    }

}


void ShutdownLightControl(void)
{
  switch(shutDownLogoState)
  {
  case(0):
    blink_time = 0;
    ilum_on();
    setLogoPWM(50);
    shutDownLogoState ++;
    break;
  case(1):
    if(blink_time > 300)
      {
        setLogoPWM(0);
        shutDownLogoState ++;
      }
  break;
  case(2):
    if(blink_time > 600)
      {
        setLogoPWM(50);
        shutDownLogoState ++;
      }
  break;
  case(3):
    if(blink_time > 900)
      {
        setLogoPWM(0);
        shutDownLogoState ++;
      }
  break;
  case(4):
    if(blink_time > 1200)
      {
        ilum_off();
        shutDownLogoState ++;
      }
  break;
  case(5):
    if(blink_time > 3600)
      {
        shutDownLogoState = 0;
      }
  break;
  default:
    ilum_on();
    setLogoPWM(50);
  }
}

#endif


