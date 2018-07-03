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

#ifndef	_TEMP_H
#define	_TEMP_H

#define NUMBER_OF_SENSORS               2
#define EXTRUDER_0                      0
#define HEATED_BED_0                    1

#define tol    1.2      //Temperatures tolerance
#define temperatureADC_length 4 // temperatures arrays length
#define sDownADC_length 11 // sDownADC arrays length


#ifdef EXP_Board
  extern uint16_t i_temp_r2c2;
  extern double extruderBlockTemp;
  extern double current_temp_r2c2;
  extern uint16_t adc_r2c2_raw[5];
  extern uint32_t adc_filtered_r2c2;
  extern uint16_t sDown_filtered;
  extern uint16_t sDownADC_raw[sDownADC_length];
#endif
#ifdef USE_BATT
  extern int32_t batt_filtered;
#endif

#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "gcode_parse.h"
#include "ExpBoard.h"

//Init vars
void temp_init();

// set target temperature
void temp_set(double t, uint8_t sensor_number);

// return last read temperature
double temp_get(uint8_t sensor_number);

// return target temperature
double temp_get_target(uint8_t sensor_number);

// true if last read temp is close to target temp, false otherwise
uint8_t temp_achieved(uint8_t sensor_number);

// true if last read temp is close to target temp, false otherwise -- works for extruder and bed temperatures
uint8_t temps_achieved (void);

// send current temperature to host
void temp_print(void);

// periodically read temperature and update heater with PID
void temp_tick(void);

void print_pwm(void);

#define NUMTEMPS 14
extern double target_temp  [NUMBER_OF_SENSORS];
extern double current_temp [NUMBER_OF_SENSORS];
double last_error;

double pterm;
double iterm;
double dterm;
double dterm_temp;
int protection_temperature;
int PID_FUNTIONAL_RANGE;
double output;
extern double temptable[NUMTEMPS][3];
void max_set(int range);
bool      temp_set_table_entry (uint8_t sensor_number, double temp, double adc_val);
double  temp_get_table_entry (uint8_t sensor_number, double temp);
double get_temp(uint8_t sensor_number);

#endif	/* _TIMER_H */
