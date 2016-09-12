#include <stdbool.h>
#include "gcode_parse.h"
#include "gcode_process.h"
#include "config.h"
#include "pinout.h"
#include "pwm.h"

#ifdef EXP_Board
  extern bool      start_logo_blink;      // start logo blink
  extern bool      stop_logo_blink;      // stop logo blink
  extern bool      logo_state;           // logo state
  extern uint32_t  blink_interval;

  bool shutDownLogoState;
  int32_t logoDuty;
#endif

extern void LogoLightControl(void);
extern void setLogoPWM(int32_t val);

#ifdef EXP_Board
  void ShutdownLightControl(void);
#endif
