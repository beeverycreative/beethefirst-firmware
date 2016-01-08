/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011-2013 BEEVC - Electronic Systems   */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
 * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 */

#include "temp.h"
#include "machine.h"
#ifndef BTF_SMOOTHIE
#include "pinout.h"
#endif
#ifdef BTF_SMOOTHIE
#include "pinout_smoothie.h"
#endif
#include "sersendf.h"
#include "stepper.h"
#include "config.h"
#include "pwm.h"

/* {ADC value Extruder0, ADC value HeatedBed0, temperature} */
double temptable[NUMTEMPS][3] = {
    {860, 60, 300},
    {1849, 95, 248},
    {2208, 119, 226},
    {2711, 215, 198},
    {2960, 293, 183},
    {3332, 447, 163},
    {3568, 641, 145},
    {3711, 865, 131},
    {3870, 1408, 105},
    {3960, 1906, 86},
    {4032, 2732, 64},
    {4062, 3352, 42},
    {4070, 3755, 22},
    {4080, 4085, 0}
};

#ifdef EXP_Board
double extruderBlockTemp = 0;
double current_temp_r2c2 = 0;
uint32_t adc_filtered_r2c2 = 4095;
int32_t adc_r2c2_raw;
static double read_R2C2_temp(void);
#endif

#if defined(BTF_SMOOTHIE) && !defined(BTF_SMOOTHIE_V1) && defined(BTF_SMOOTHIE_V2)
static double read_spi_temp(void);
#endif

double current_temp [NUMBER_OF_SENSORS] = {0};
double target_temp  [NUMBER_OF_SENSORS] = {0};
static uint32_t adc_filtered [NUMBER_OF_SENSORS] = {4095, 4095}; // variable must have the higher value of ADC for filter start at the lowest temperature

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

uint16_t thermocoupleErrorCount = 0;
uint16_t thermocoupleErrorTimer = 0;

static double read_temp(uint8_t sensor_number);

void temp_set(double t, uint8_t sensor_number)
{
  if (t)
    {
      steptimeout = 0;
      //?    power_on();
    }

  target_temp[sensor_number] = t;
}

void max_set(int range){
  PID_FUNTIONAL_RANGE =range;
}

double temp_get(uint8_t sensor_number)
{
  current_temp[sensor_number] = read_temp(sensor_number);
  return current_temp[sensor_number];
}

double get_temp(uint8_t sensor_number)
{
  return current_temp[sensor_number];
}
double temp_get_target(uint8_t sensor_number)
{
  return target_temp[sensor_number];
}

uint8_t	temp_achieved(uint8_t sensor_number)
{
  if (current_temp[sensor_number] >= (target_temp[sensor_number] - 2))
    return 255;

  return 0;
}

uint8_t temps_achieved (void)
{
  if ((current_temp[EXTRUDER_0] >= (target_temp[EXTRUDER_0] - 2)) && (current_temp[HEATED_BED_0] >= (target_temp[HEATED_BED_0] - 2)))
    return 255;

  return 0;
}

void temp_print()
{
#if defined(EXP_Board)
  sersendf("T:%g B:%g R:%g\n", current_temp[EXTRUDER_0], current_temp[HEATED_BED_0], current_temp_r2c2);
#elif !defined(BTF_SMOOTHIE)
  sersendf("T:%g B:%g ", current_temp[EXTRUDER_0], current_temp[HEATED_BED_0]);
#elif defined(BTF_SMOOTHIE)
  sersendf("T:%g B:%g C:%g ", current_temp[EXTRUDER_0], current_temp[HEATED_BED_0], current_temp[CHAMBER]);
#endif
}

void temp_tick(void)
{
  double pid_error = 0;
  double pid_error_bed = 0;

  /* Read and average temperatures */
#ifndef BTF_SMOOTHIE
  current_temp[EXTRUDER_0] = read_temp(EXTRUDER_0);
#endif
#if defined(BTF_SMOOTHIE) && defined(BTF_SMOOTHIE_V1) && !defined(BTF_SMOOTHIE_V2)
  current_temp[EXTRUDER_0] = read_temp(EXTRUDER_0);
  current_temp[HEATED_BED_0] = read_temp(HEATED_BED_0);
  current_temp[CHAMBER] = read_temp(CHAMBER);
#endif
#if defined(BTF_SMOOTHIE) && !defined(BTF_SMOOTHIE_V1) && defined(BTF_SMOOTHIE_V2)
  current_temp[EXTRUDER_0] = read_spi_temp();
  //current_temp[EXTRUDER_0] = read_temp(EXTRUDER_0);
  current_temp[HEATED_BED_0] = read_temp(HEATED_BED_0);
  current_temp[CHAMBER] = read_temp(CHAMBER);
#endif
#ifdef EXP_Board
  current_temp[HEATED_BED_0] = read_temp(HEATED_BED_0);

  extruderBlockTemp = current_temp[HEATED_BED_0];
  current_temp_r2c2 = read_R2C2_temp();
#endif

  pid_error = target_temp[EXTRUDER_0] - current_temp[EXTRUDER_0];
  pid_error_bed = target_temp[HEATED_BED_0] - current_temp[HEATED_BED_0];

  //pterm = config.kp * pid_error;
  //iterm += (config.ki*pid_error);
  pterm = kp * pid_error;
  iterm += (ki*pid_error);

  pterm_bed = kp_bed * pid_error_bed;
  iterm_bed += (ki_bed*pid_error_bed);

  if(iterm > PID_FUNTIONAL_RANGE){
      iterm = PID_FUNTIONAL_RANGE;
  }else if(iterm < 0){
      iterm = 0;
  }

  if(iterm_bed > PID_FUNTIONAL_RANGE){
      iterm_bed = PID_FUNTIONAL_RANGE;
  }else if(iterm_bed < 0){
      iterm_bed = 0;
  }

  dterm_temp = pid_error - last_error;
  //dterm = config.kd * dterm_temp;
  dterm = kd * dterm_temp;

  dterm_temp_bed = pid_error_bed - last_error_bed;
  dterm_bed = kd_bed * dterm_temp_bed;

  output = pterm + iterm + dterm;

  output_bed = pterm_bed + iterm_bed + dterm_bed;

  last_error = pid_error;
  last_error_bed = pid_error_bed;

  if(output > 100) {
      output = 100;
  }else if(output<0 ) {
      output = 0;
  }

  if(output_bed > 100) {
      output_bed = 100;
  }else if(output_bed<0 ) {
      output_bed = 0;
  }

  if(target_temp[EXTRUDER_0] == 0){
      output = 0;
      pterm = 0;
      iterm = 0;
      dterm = 0;
      pid_error = 0;
      dterm_temp = 0;
  }

  if(target_temp[HEATED_BED_0] == 0){
      output_bed = 0;
      pterm_bed = 0;
      iterm_bed = 0;
      dterm_bed = 0;
      pid_error_bed = 0;
      dterm_temp_bed = 0;
  }

  if(target_temp[CHAMBER] == 0)
    {
      digital_write(CHAMBER_HEATER_PORT, CHAMBER_HEATER_PIN, 0);
    }

  if(current_temp[CHAMBER] < target_temp[CHAMBER])
    {
      digital_write(CHAMBER_HEATER_PORT, CHAMBER_HEATER_PIN, 1);
    }
#if defined(BTF_SMOOTHIE) && !defined(BTF_SMOOTHIE_V1) && defined(BTF_SMOOTHIE_V2)
  if((current_temp[CHAMBER] > target_temp[CHAMBER] + 10) && target_temp[CHAMBER] != 0)
    {
      setBlowerSpeed(50);
    }
  else
    {
      disableBlower();
    }
#endif

  thermocoupleErrorTimer ++;
  if(thermocoupleErrorTimer > 20)
    {
      if(thermocoupleErrorCount > 10)
        {
          output = 0;
        }
      thermocoupleErrorTimer = 0;
    }


  if(current_temp[EXTRUDER_0] > -50)
    {
      pwm_set_duty_cycle(EXTRUDER_0_PWM_CHANNEL, output);
      pwm_set_enable(EXTRUDER_0_PWM_CHANNEL);
    }

  pwm_set_duty_cycle(HEATED_BED_0_PWM_CHANNEL, output_bed);
  pwm_set_enable(HEATED_BED_0_PWM_CHANNEL);
}


/* Read and average the ADC input signal */
#ifndef BTF_SMOOTHIE
static double read_temp(uint8_t sensor_number)
{
  int32_t raw = 4095; // initialize raw with value equal to lowest temperature.
  double celsius = 0;
  uint8_t i;

  if (sensor_number == EXTRUDER_0){
      raw = analog_read(EXTRUDER_0_SENSOR_ADC_CHANNEL);

  }else if (sensor_number == HEATED_BED_0)
    {
      //raw = analog_read(HEATED_BED_0_SENSOR_ADC_CHANNEL);
      int32_t bed_temp_buf[5];
      for(int32_t i = 0; i < 5; i++)
        {
          bed_temp_buf[i] = analog_read(HEATED_BED_0_SENSOR_ADC_CHANNEL);
        }
      raw = getMedianValue(bed_temp_buf);
    }

  // filter the ADC values with simple IIR
  adc_filtered[sensor_number] = ((adc_filtered[sensor_number] * 15) + raw) / 16;

  raw = adc_filtered[sensor_number];

  /* Go and use the temperature table to math the temperature value... */
  if (raw < temptable[0][sensor_number]) /* Limit the smaller value... */
    {
      celsius = temptable[0][2];
    }
  else if (raw >= temptable[NUMTEMPS-1][sensor_number]) /* Limit the higher value... */
    {
      celsius = temptable[NUMTEMPS-1][2];
    }
  else
    {
      for (i=1; i<NUMTEMPS; i++)
        {
          if (raw < temptable[i][sensor_number])
            {
              celsius = temptable[i-1][2] +
                  (raw - temptable[i-1][sensor_number]) *
                  (temptable[i][2] - temptable[i-1][2]) /
                  (temptable[i][sensor_number] - temptable[i-1][sensor_number]);

              break;
            }
        }
    }

  return celsius;
}
#endif
#ifdef BTF_SMOOTHIE
static double read_spi_temp(void)
{
  uint16_t i = 0;
  SPI_MAX_CS_Low();
  delay(1);
  uint16_t data = SPI_MAX_RecvByte();
  uint16_t data2 = SPI_MAX_RecvByte();
  SPI_MAX_CS_High();

  double temp;
  if(data & 0x0001)
    {
      temp = -9999.0;
      thermocoupleErrorCount += 1;
    }
  else
    {
      data = data >> 2;
      temp = ((double) (data & 0x1FFF)) / (double) 4.0;

      if(data & 0x2000)
        {
          data = ~data;
          temp = ((double) (data & 0x1FFF) + 1) / (double) -4.0;
        }
    }

  return temp;
}

static double read_temp(uint8_t sensor_number)
{
  int32_t raw = 4095; // initialize raw with value equal to lowest temperature.
  double celsius = 0;
  uint8_t i;

  if (sensor_number == EXTRUDER_0){
      raw = analog_read(EXTRUDER_0_SENSOR_ADC_CHANNEL);

  }else if (sensor_number == HEATED_BED_0){

      raw = analog_read(HEATED_BED_0_SENSOR_ADC_CHANNEL);
  }else if (sensor_number == CHAMBER){

      raw = analog_read(CHAMBER_ADC_CHANNEL);
  }

  // filter the ADC values with simple IIR
  adc_filtered[sensor_number] = ((adc_filtered[sensor_number] * 15) + raw) / 16;

  raw = adc_filtered[sensor_number];

  float r = 4700 / (((float)4096 / (float)raw) - (float)1);
  float k = ((float) 1 / (float) 300.15);
  float j = ((float) 1 / (float) 4066);

  celsius = (double) ((float)1 / (k + (j * logf(r / (float)100000)))) - (float) 273.15;

  return celsius;
}
#endif

#ifdef EXP_Board
/* Read and average the R2C2 ADC input signal */
static double read_R2C2_temp(void)
{
  double celsius = 0;

  int32_t adc_r2c2_buf[5];
  for(int32_t i = 0; i < 5; i++)
    {
      adc_r2c2_buf[i] = analog_read(R2C2_TEMP_SENSOR_ADC_CHANNEL);
    }

  adc_r2c2_raw = getMedianValue(adc_r2c2_buf);

  adc_filtered_r2c2 = adc_filtered_r2c2*0.9 + adc_r2c2_raw*0.1;

  double volts = (double) adc_filtered_r2c2*(3.3/4096);

  celsius = (volts - 0.5)*100;

  return celsius;
}
#endif
bool temp_set_table_entry (uint8_t sensor_number, double temp, double adc_val)
{
  if (sensor_number < NUMBER_OF_SENSORS)
    {
      for (int entry=0; entry < NUMTEMPS; entry++)
        {
          if (temptable[entry][2] == temp)
            {
              temptable[entry][sensor_number] = adc_val;
              return true;
            }
        }
      return false;
    }
  else
    return false;
}

double temp_get_table_entry (uint8_t sensor_number, double temp)
{
  double result = 0xffffffff;

  if (sensor_number < NUMBER_OF_SENSORS)
    {
      for (int entry=0; entry < NUMTEMPS; entry++)
        {
          if (temptable[entry][2] == temp)
            {
              result = temptable[entry][sensor_number];
              break;
            }
        }
    }
  return result;
}

void print_pwm(void){
  sersendf("E0 pwm:%g p:%g i:%g d:%g \n", output, pterm,iterm,dterm);
  sersendf("Bed pwm:%g p:%g i:%g d:%g \n", output_bed, pterm_bed,iterm_bed,dterm_bed);
}
