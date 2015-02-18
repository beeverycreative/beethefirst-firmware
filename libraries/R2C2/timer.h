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

#ifndef	_TIMER_H
#define	_TIMER_H

#include <stdbool.h>
#include "pinout.h"

uint32_t sDownADC_raw;
uint32_t sDown_filtered;

// time-related constants
//#define	US	(F_CPU / 1000000)
#define	MS	(F_CPU / 1000)

// #define	DEFAULT_TICK	(100 US)
#define	WAITING_DELAY  (10 * MS)

#define NUM_HARDWARE_TIMERS 4

// Slow timers
typedef struct tTimer tTimer; // incomplete type

typedef void (*tTimerCallback)(tTimer *);

struct tTimer
{
  tTimer            *pNext;
  tTimerCallback    timerCallback;
  uint32_t          Current;
  uint32_t          Reload;
  volatile uint8_t  Running:1;
  volatile uint8_t  Expired:1;
  volatile uint8_t  AutoReload:1;
};

// Hardware timers
typedef struct tHwTimer tHwTimer;

// timer callback gets pointer to tHwTimer struct and copy of timer IR register
typedef void (*tHwTimerCallback)(tHwTimer *, uint32_t);

struct tHwTimer
{
  tHwTimerCallback timerCallback;
} ;

void setupHwTimer (uint16_t timerNum, tHwTimerCallback timerCallback);
void setHwTimerInterval (uint16_t timerNum, uint32_t ticks);
void enableHwTimer (uint16_t timerNum);
void disableHwTimer (uint16_t timerNum);
uint8_t isHwTimerEnabled(uint16_t timerNum);
void setHwTimerMatch (uint16_t timerNum, uint16_t matchReg, uint32_t interval);

void SysTickTimer_Init(void);
void delay(int delay);
void delay_ms(int delay);
void delayMicrosecondsInterruptible(int us);
#define	delay_us(d) delayMicrosecondsInterruptible(d)
long millis(void);

// Slow timer (i.e. +/-1ms resolution)
bool AddSlowTimer (tTimer *pTimer);
void StartSlowTimer (tTimer *pTimer, uint32_t intervalMillis, tTimerCallback timerCallback);
void StopSlowTimer (tTimer *pTimer);
#define IsSlowTimerExpired (pTimer)  ((pTimer)->Expired)

#endif	/* _TIMER_H */

