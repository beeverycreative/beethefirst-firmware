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

#ifndef _BUZZER_
#define _BUZZER_


void buzzer_init (void);

/* put a PWM signal of 50% duty cycle, with a specified frequency and duration.
 * 20Hz <= frequency <= 20000
 * 10ms <= duration <= (2^16 - 1)ms
 */
void buzzer_play (uint16_t frequency, uint16_t duration);
void buzzer_wait(void);

#endif  /* _BUZZER_ */

