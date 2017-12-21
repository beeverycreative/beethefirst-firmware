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

#ifndef _ADC_H
#define _ADC_H

#include <stdint.h>
#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"

#include "pinout.h"

#define XTAL_FREQ       10000000
#define MAX_ADC_CLOCK   13000000
#define CLKS_PER_SAMPLE 64

/* Initialize ADC for extruder temperature reading */
void config_adc_pins(void);
void adc_init(int sample_rate, int cclk_div);
uint16_t analog_read(uint8_t adc_channel);

int _adc_clk_freq;

uint32_t _adc_data[8];
void(*_adc_isr[8])(uint32_t value);
void(*_adc_g_isr)(int chan, uint32_t value);
void(*_adc_m_isr)(void);


#endif
