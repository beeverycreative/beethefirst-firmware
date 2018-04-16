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
#include "pinout.h"
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

double current_temp [NUMBER_OF_SENSORS] = {0};
double target_temp  [NUMBER_OF_SENSORS] = {0};
static uint32_t adc_filtered [NUMBER_OF_SENSORS] = {4095, 4095}; // variable must have the higher value of ADC for filter start at the lowest temperature

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

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
  if (current_temp[sensor_number] >= (target_temp[sensor_number]*((100.0-tol)/100)))
    return 255;

  return 0;
}

uint8_t temps_achieved (void)
{
  if ((current_temp[EXTRUDER_0]>=target_temp[EXTRUDER_0]*((100.0-tol)/100)) && (current_temp[HEATED_BED_0] >= target_temp[HEATED_BED_0]*((100.0-tol)/100)))
    return 255;

  return 0;
}

void temp_print()
{
#ifdef EXP_Board
  sersendf("T:%g B:%g R:%g\n", current_temp[EXTRUDER_0], current_temp[HEATED_BED_0], current_temp_r2c2);
#else
  sersendf("T:%g B:%g ", current_temp[EXTRUDER_0], current_temp[HEATED_BED_0]);
#endif
}

void temp_tick(void)
{
  double pid_error = 0;

  /* Read and average temperatures */
  current_temp[EXTRUDER_0] = read_temp(EXTRUDER_0);
#ifdef EXP_Board
  current_temp[HEATED_BED_0] = read_temp(HEATED_BED_0);

  extruderBlockTemp = current_temp[HEATED_BED_0];
  current_temp_r2c2 = read_R2C2_temp();
#endif

  pid_error = target_temp[EXTRUDER_0] - current_temp[EXTRUDER_0];

  pterm = config.kp * pid_error;
  iterm += (config.ki*pid_error);
  //pterm = kp * pid_error;
  //iterm += (ki*pid_error);

  if(iterm > PID_FUNTIONAL_RANGE){
      iterm = PID_FUNTIONAL_RANGE;
  }else if(iterm < 0){
      iterm = 0;
  }

  dterm_temp = pid_error - last_error;
  dterm = config.kd * dterm_temp;
  //dterm = kd * dterm_temp;

  output = pterm + iterm + dterm;
  //output *= 0.95937 + 0.00907*currenBWSpeed + 0.01662*extruderFanSpeed + 0.000009*currenBWSpeed*currenBWSpeed - 0.000035*currenBWSpeed*extruderFanSpeed - 0.000068*extruderFanSpeed*extruderFanSpeed;
  //output += output*config.kVent*extruderFanSpeed;
  //output += output*config.kBlower*currenBWSpeed;

  output = pterm + iterm + dterm;
/*
#ifdef EXP_Board
  double p00 = -0.02242;
  double p10 = -0.001512*extruderFanSpeed;
  double p01 = 0.01811*currenBWSpeed;
  double p20 = 0.0003169*extruderFanSpeed*extruderFanSpeed;
  double p11 = -0.00006381*extruderFanSpeed*currenBWSpeed;
  double p02 = -0.00008276*currenBWSpeed*currenBWSpeed;
  double p30 = -0.000002056*extruderFanSpeed*extruderFanSpeed*extruderFanSpeed;
  double p21 = -0.000000008015*currenBWSpeed*extruderFanSpeed*extruderFanSpeed;
  double p12 = 0.0000002986*extruderFanSpeed*currenBWSpeed*currenBWSpeed;

  double pxy = p00 + p10 + p01 + p20 + p11 + p02 + p30 + p21 + p12;


  output = output*(1 + pxy);
#endif
*/
  last_error = pid_error;

  if(output > 100) {
      output = 100;
  }else if(output<0 ) {
      output = 0;
  }

  if(target_temp[EXTRUDER_0] == 0){
      output = 0;
      pterm = 0;
      iterm = 0;
      dterm = 0;
      pid_error = 0;
      dterm_temp = 0;
  }

  pwm_set_duty_cycle(5, output);
  pwm_set_enable(5);
}


/* Read and average the ADC input signal */
static double read_temp(uint8_t sensor_number)
{
  int32_t raw = 4095; // initialize raw with value equal to lowest temperature.
  double celsius = 0;
  uint8_t i;

  if (sensor_number == EXTRUDER_0){
      raw = analog_read(EXTRUDER_0_SENSOR_ADC_CHANNEL);

  }else if (sensor_number == HEATED_BED_0)
    {
      raw = analog_read(HEATED_BED_0_SENSOR_ADC_CHANNEL);
	  /*
      int32_t bed_temp_buf[5];
      for(int32_t i = 0; i < 5; i++)
        {
          bed_temp_buf[i] = analog_read(HEATED_BED_0_SENSOR_ADC_CHANNEL);
        }
      raw = getMedianValue(bed_temp_buf);
      */
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

#ifdef EXP_Board
/* Read and average the R2C2 ADC input signal */
static double read_R2C2_temp(void)
{
  double celsius = 0;

  int32_t adc_r2c2_buf[9];
  for(int32_t i = 0; i < 9; i++)
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
  sersendf("pwm:%g p:%g i:%g d:%g ", output, pterm,iterm,dterm);
}
