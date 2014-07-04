/* Copyright (c) 2011-2013 BEEVC - Electronic Systems        */
/*
 * This file is part of BEESOFT software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version. BEESOFT is
 * distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details. You should have received a copy of the
 * GNU General Public License along with BEESOFT. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_pwm.h"

#include "timer.h"

static tTimer buzzerTimer;

//
// Internal functions
//

static void buzzer_pwm_stop (void)
{
  pwm_set_duty_cycle(3, 0);
  pwm_set_enable(3);
}

static void buzzerTimerCallback (tTimer *pTimer)
{
  (void)pTimer; // not used

  buzzer_pwm_stop();
}

//
// Public functions
//

void buzzer_play (uint16_t duration)
{
  pwm_set_duty_cycle(3, 50);
  pwm_set_enable(3);
  
  StartSlowTimer (&buzzerTimer, duration, buzzerTimerCallback); 
}

void buzzer_wait(void)
{
  while (buzzerTimer.Running){};
  
}

void buzzer_init (void){
  AddSlowTimer (&buzzerTimer);
}


void buzzer_pwm_start (void)
{
  PWM_ResetCounter(LPC_PWM1);
  PWM_CounterCmd(LPC_PWM1, ENABLE);

  /* Start PWM now */
  PWM_Cmd(LPC_PWM1, ENABLE);
}
