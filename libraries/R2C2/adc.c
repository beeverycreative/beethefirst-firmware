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

#include "pinout.h"
#include "ios.h"
#include "adc.h"
#include <stdlib.h>

void config_adc_pins(void)
{
        PINSEL_CFG_Type PinCfg;

        //Extruder 0 ADC Config
        PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
        PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
        PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
        PinCfg.Portnum = EXTRUDER_0_SENSOR_ADC_PORT;
        PinCfg.Pinnum = EXTRUDER_0_SENSOR_ADC_PIN;
        PINSEL_ConfigPin(&PinCfg);
        //ADC_IntConfig(LPC_ADC,ADC_ADINTEN7,SET);

#ifdef EXP_Board
        //Extruder Block Temperature ADC Config
        PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
        PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
        PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
        PinCfg.Portnum = HEATED_BED_0_ADC_PORT;
        PinCfg.Pinnum = HEATED_BED_0_ADC_PIN;
        PINSEL_ConfigPin(&PinCfg);

        //R2C2 TEMPERATURE ADC CONFIG
        PinCfg.Funcnum = PINSEL_FUNC_1; /*ADC Function*/
        PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
        PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
        PinCfg.Portnum = R2C2_TEMP_ADC_PORT;
        PinCfg.Pinnum = R2C2_TEMP_ADC_PIN;
        PINSEL_ConfigPin(&PinCfg);

        //Shutdown ADC CONFIG
        PinCfg.Funcnum = PINSEL_FUNC_3; /*ADC Function*/
        PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
        PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
        PinCfg.Portnum = SDOWN_ADC_PORT;
        PinCfg.Pinnum = SDOWN_ADC_PIN;
        PINSEL_ConfigPin(&PinCfg);

#endif

#ifdef USE_BATT
        //Battery ADC CONFIG

        PinCfg.Funcnum = PINSEL_FUNC_1;
        PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
        PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
        PinCfg.Portnum = BATT_ADC_PORT;
        PinCfg.Pinnum = BATT_ADC_PIN;
        PINSEL_ConfigPin(&PinCfg);

#endif
}

/* Initialize ADC for reading sensors */
void adc_init()
{
        uint8_t i;

        config_adc_pins();

        /* Configuration for ADC:
        *  select: ADC channel 2 (if using MCB1700 board)
        *              ADC channel 5 (if using IAR-LPC1768 board)
        *  ADC conversion rate = 200KHz
        */
        ADC_Init(LPC_ADC, 200000);

        ADC_ChannelCmd(LPC_ADC,EXTRUDER_0_SENSOR_ADC_CHANNEL,ENABLE);

#ifdef EXP_Board
        ADC_ChannelCmd(LPC_ADC,HEATED_BED_0_SENSOR_ADC_CHANNEL,ENABLE);

        ADC_ChannelCmd(LPC_ADC,R2C2_TEMP_SENSOR_ADC_CHANNEL,ENABLE);

        ADC_ChannelCmd(LPC_ADC,SDOWN_ADC_SENSOR_ADC_CHANNEL,ENABLE);

#endif

#ifdef USE_BATT

        ADC_ChannelCmd(LPC_ADC,BATT_ADC_SENSOR_ADC_CHANNEL,ENABLE);

#endif

        ADC_IntConfig(LPC_ADC,ADC_ADGINTEN,RESET);
        ADC_BurstCmd(LPC_ADC,ENABLE);


        NVIC_EnableIRQ(ADC_IRQn);


};



void ADC_IRQHandler(void)
{

    uint32_t stat;
    int chan;

    // Read status
    stat = LPC_ADC->ADSTAT;

    //Scan channels for over-run or done and update array
    if (stat & 0x0101) _adc_data[0] = LPC_ADC->ADDR0;
    if (stat & 0x0202) _adc_data[1] = LPC_ADC->ADDR1;
    if (stat & 0x0404) _adc_data[2] = LPC_ADC->ADDR2;
    if (stat & 0x0808) _adc_data[3] = LPC_ADC->ADDR3;
    if (stat & 0x1010) _adc_data[4] = LPC_ADC->ADDR4;
    if (stat & 0x2020) _adc_data[5] = LPC_ADC->ADDR5;
    if (stat & 0x4040) _adc_data[6] = LPC_ADC->ADDR6;
    if (stat & 0x8080) _adc_data[7] = LPC_ADC->ADDR7;

    // Channel that triggered interrupt
    chan = (LPC_ADC->ADGDR >> 24) & 0x07;
    if (_adc_isr[chan] != NULL)
        _adc_isr[chan](_adc_data[chan]);
    if (_adc_g_isr != NULL)
        _adc_g_isr(chan, _adc_data[chan]);
    return;

}

uint16_t analog_read(uint8_t adc_channel)
{
        /*
  ADC_ChannelCmd(LPC_ADC, adc_channel, ENABLE);

  // Start conversion
  ADC_StartCmd(LPC_ADC, ADC_START_NOW);

  // Wait conversion complete
  while (!(ADC_ChannelGetStatus(LPC_ADC, adc_channel, ADC_DATA_DONE))){

  }

  ADC_ChannelCmd(LPC_ADC, adc_channel, DISABLE);
         */

        uint16_t adc_value = ADC_ChannelGetData(LPC_ADC, adc_channel);

        return adc_value;
}
