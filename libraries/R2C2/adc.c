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
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"

//#include "pinout.h"
#include "ios.h"
#include "adc.h"


uint16_t analog_read(uint8_t adc_channel)
{
  ADC_ChannelCmd(LPC_ADC, adc_channel, ENABLE);

  // Start conversion
  ADC_StartCmd(LPC_ADC, ADC_START_NOW);

  // Wait conversion complete
  while (!(ADC_ChannelGetStatus(LPC_ADC, adc_channel, ADC_DATA_DONE))){

  }

  ADC_ChannelCmd(LPC_ADC, adc_channel, DISABLE);

  return ADC_ChannelGetData(LPC_ADC, adc_channel);
}
