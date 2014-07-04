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

#ifndef _PWM_H
#define _PWM_H

#include "lpc17xx_pwm.h"
#include "lpc17xx.h"
#include "lpc17xx_pinsel.h"

int pwm_set_duty_cycle(int channel, int32_t duty);
void pwm_set_enable(int pwm_channel);
void pwm_set_disable(int pwm_channel);
void init_global_match(int pwm_channel);
void init_pwm_peripheral(void);
void pwm_pins_init(int port,int pin);

#endif  /* _PWM_H */

