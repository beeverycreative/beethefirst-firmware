/* Copyright (c) 2011-2014 BEEVC - Electronic Systems        */
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
#include "pwm.h"

/*set duty cycle*/
int pwm_set_duty_cycle(int channel, int32_t duty){
  PWM_MATCHCFG_Type PWMMatchCfgDat;

  int32_t match_value = 1000;

  if(duty<0 || duty>100)
    return -1;

  /* Set up match value */
  PWM_MatchUpdate(LPC_PWM1, channel, duty*10, PWM_MATCH_UPDATE_NEXT_RST);
  return 0;
}

/* set and enable*/
void pwm_set_enable(int pwm_channel){
  if(pwm_channel<1||pwm_channel>5){
      return;
  }
  /* Reset and Start counter */
  PWM_ResetCounter(LPC_PWM1);
  PWM_CounterCmd(LPC_PWM1, ENABLE);

  /* Start PWM now */
  PWM_Cmd(LPC_PWM1, ENABLE);

  /* Enable PWM Channel Output */
  PWM_ChannelCmd(LPC_PWM1, pwm_channel, ENABLE);
}

/*disable*/
void pwm_set_disable(int pwm_channel){
  PWM_MATCHCFG_Type PWMMatchCfgDat;

  if(pwm_channel<1||pwm_channel>5){
      return;
  }

  /* Enable PWM Channel Output */
  PWM_ChannelCmd(LPC_PWM1, pwm_channel, DISABLE);
}

/* initially all pwm have o duty cycle*/
void init_global_match(int pwm_channel){
  PWM_MATCHCFG_Type PWMMatchCfgDat;

  /* - Single edge
   * - PWM Duty determined by
   * the match on channel 0 to the match of that match channel.*/

  /* Configure PWM channel edge option
   * Note: PWM Channel 1 is in single mode as default state and
   * can not be changed to double edge mode */
  if(pwm_channel!=1)
    PWM_ChannelConfig(LPC_PWM1, pwm_channel, PWM_CHANNEL_SINGLE_EDGE);

  /* Set up match value */
  PWM_MatchUpdate(LPC_PWM1, pwm_channel, 0, PWM_MATCH_UPDATE_NOW);

  /* Configure match option */
  PWMMatchCfgDat.IntOnMatch = DISABLE;
  PWMMatchCfgDat.MatchChannel = pwm_channel;
  PWMMatchCfgDat.ResetOnMatch = DISABLE;
  PWMMatchCfgDat.StopOnMatch = DISABLE;
  PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
}

/* configure match register 0 */
void init_pwm_peripheral(void){
  PWM_TIMERCFG_Type PWMCfgDat;
  PWM_MATCHCFG_Type PWMMatchCfgDat;

  //freq = 1000 Hz
  uint32_t match_value = (((float) 1/1000)) * 1000000;

  /* Initialize PWM peripheral, timer mode
   * PWM prescale value = 1 (absolute value - tick value) */
  PWMCfgDat.PrescaleOption = PWM_TIMER_PRESCALE_USVAL;
  PWMCfgDat.PrescaleValue = 1;
  PWM_Init(LPC_PWM1, PWM_MODE_TIMER, (void *) &PWMCfgDat);

  /* Set match value for PWM match channel 0, update immediately */
  PWM_MatchUpdate(LPC_PWM1, 0, match_value, PWM_MATCH_UPDATE_NOW);

  /* PWM Timer/Counter will be reset when channel 0 matching
   * no interrupt when match
   * no stop when match */
  PWMMatchCfgDat.IntOnMatch = DISABLE;
  PWMMatchCfgDat.MatchChannel = 0;
  PWMMatchCfgDat.ResetOnMatch = ENABLE;
  PWMMatchCfgDat.StopOnMatch = DISABLE;
  PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
}

/* Initialize the pin*/
void pwm_pins_init(int port,int pin){
  PINSEL_CFG_Type PinCfg;

  /*Initialize PWM pin connect*/
  PinCfg.Funcnum = PINSEL_FUNC_1;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_PULLDOWN;
  PinCfg.Portnum = port;
  PinCfg.Pinnum = pin;

  PINSEL_ConfigPin(&PinCfg);
}
