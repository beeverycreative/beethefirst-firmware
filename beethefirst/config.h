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

#ifndef CONFIG_H
#define CONFIG_H

#include "stdint.h"


#define MM_REPRAP_MENDEL  0
#define MM_RAPMAN         1

struct configuration
{
  int32_t uid;
  
  double steps_per_mm_x;
  double steps_per_mm_y;
  double steps_per_mm_z;
  double steps_per_mm_e0;

  double home_pos_x;
  double home_pos_y;
  double home_pos_z;

  double  acceleration;
  double  junction_deviation;
  
  //PID
  double kp;
  double ki;
  double kd;
  double kBlower;
  double kVent;

  // options

  int32_t status;

  // autonomous
  char filename[50];
  int32_t  sd_pos;
  int32_t estimated_time;
  int32_t time_elapsed;
  int32_t number_of_lines;
  int32_t executed_lines;
  double  startpoint_x;
  double  startpoint_y;
  double  startpoint_z;
  double  startpoint_e;
  double  startpoint_feed_rate;
  double  startpoint_temperature;
  double  startpoint_filament_coeff;
  double  startpoint_feedrate_coeff;
  int32_t blowerSpeed;
  int32_t last_print_time;
  double last_print_filament;
  int32_t battery_ShutDown;

  //Block Fan Control
  double blockControlM;
  double blockControlB;
  double blockTemperatureFanStart;
  double blockTemperatureFanMax;
  double blockFanMinSpeed;
  double blockFanMaxSpeed;

  //Filament and Nozzle configs
  char bcodeStr[100];
  double filament_in_spool;
  uint16_t nozzleSize;

  //Battery Variables
  uint32_t batteryPrintTime;
  uint32_t standByTime;
  uint32_t autoResume;

  //Power Saving
  uint32_t powerSavingWaitTime;

};

extern struct configuration config;

//extern double kp;
//extern double ki;
//extern double kd;

bool read_config (void);
void reset_config (void);
void print_config (void);
void write_config (void);

#endif /* CONFIG_H */
