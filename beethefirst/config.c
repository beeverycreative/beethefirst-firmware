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

#include <stdbool.h>
#include <ctype.h>
#include "stdlib.h"
#include "string.h"
#include <stdio.h>

#include "config.h"
#include "spi.h"
#include "ff.h"
#include "debug.h"
#include "gcode_parse.h"
#include "uart.h"
#include "sbl_config.h"
#include "system_LPC17xx.h"

/* values reflecting the gearing of your machine
 * numbers are integers or double
 */

struct configuration config;

#define TYPE_INT    0
#define TYPE_DOUBLE 1
#define TYPE_STRING 2
#define BAD_DOUBLE  0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
#define BAD_INT     0xFFFFFFFFFFFFFFFF
typedef struct {
  char      *name;
  void      *pValue;
  uint8_t   type;
  union {
    int32_t   val_i;
    double    val_d;
    char*       val_s;
  };
} tConfigItem;

/* calculate the default values appropriate for your machine */
tConfigItem config_lookup [] = 
    {
        { "UID", &config.uid, TYPE_INT, {.val_i=CFG_UID}},
        //{ "machine_model", &config.machine_model, TYPE_INT, {.val_i=0}},

        { "steps_per_mm_x", &config.steps_per_mm_x, TYPE_DOUBLE, {.val_d=STEPS_MM_X}},
        { "steps_per_mm_y", &config.steps_per_mm_y, TYPE_DOUBLE, {.val_d=STEPS_MM_Y}},
        { "steps_per_mm_z", &config.steps_per_mm_z, TYPE_DOUBLE, {.val_d=STEPS_MM_Z}},
        { "steps_per_mm_e0", &config.steps_per_mm_e0, TYPE_DOUBLE, {.val_d=STEPS_MM_E0}},

        { "home_pos_x", &config.home_pos_x, TYPE_DOUBLE, {.val_d=HOME_POS_X}},
        { "home_pos_y", &config.home_pos_y, TYPE_DOUBLE, {.val_d=HOME_POS_Y}},
        { "home_pos_z", &config.home_pos_z, TYPE_DOUBLE, {.val_d=HOME_POS_Z}},

        { "acceleration",       &config.acceleration, TYPE_DOUBLE, {.val_d=500.000}},         /* 100mm / second^2 */
        { "junction_deviation", &config.junction_deviation, TYPE_DOUBLE, {.val_d=0.050}},

        //PID
#ifdef EXP_Board
        { "extruder_kp", &config.kp, TYPE_DOUBLE, {.val_d=10.0}},
#else
        { "extruder_kp", &config.kp, TYPE_DOUBLE, {.val_d=6.0}},
#endif
        { "extruder_ki", &config.ki, TYPE_DOUBLE, {.val_d=0.0013}},
        { "extruder_kd", &config.kd, TYPE_DOUBLE, {.val_d=80.0}},
        { "extruder_kBlower", &config.kBlower, TYPE_DOUBLE, {.val_d=0.0035}},
        { "extruder_kVent", &config.kVent, TYPE_DOUBLE, {.val_d=0.005}},

        { "status", &config.status, TYPE_INT, {.val_i=1}},

        {"filename",config.filename , TYPE_STRING, {.val_s=0}},
        {"sd_pos",&config.sd_pos , TYPE_INT, {.val_i=0}},
        {"estimated_time",&config.estimated_time , TYPE_INT, {.val_i=0}},
        {"time_elapsed",&config.time_elapsed , TYPE_INT, {.val_i=0}},
        {"number_of_lines",&config.number_of_lines , TYPE_INT, {.val_i=0}},
        {"executed_lines",&config.executed_lines , TYPE_INT, {.val_i=0}},
        {"startpoint_x", &config.startpoint_x, TYPE_DOUBLE, {.val_d=0}},
        {"startpoint_y", &config.startpoint_y, TYPE_DOUBLE, {.val_d=0}},
        {"startpoint_z", &config.startpoint_z, TYPE_DOUBLE, {.val_d=0}},
        {"startpoint_e", &config.startpoint_e, TYPE_DOUBLE, {.val_d=0}},
        {"startpoint_feed_rate", &config.startpoint_feed_rate, TYPE_DOUBLE, {.val_d=0}},
        {"startpoint_temperature", &config.startpoint_temperature, TYPE_DOUBLE, {.val_d=0}},
        {"startpoint_filament_coeff", &config.startpoint_filament_coeff, TYPE_DOUBLE, {.val_d=1}},
        {"startpoint_feedrate_coeff", &config.startpoint_feedrate_coeff, TYPE_DOUBLE, {.val_d=1}},
        {"Blower_Speed",&config.blowerSpeed , TYPE_INT, {.val_i=0}},
        {"Last_Print_Time",&config.last_print_time , TYPE_INT, {.val_i=0}},
        {"Last_Print_Filament",&config.last_print_filament , TYPE_DOUBLE, {.val_i=0.0}},
        {"Battery_Shutdown",&config.battery_ShutDown , TYPE_INT, {.val_i=0}},

        //Block Fan Control
        { "block_fan_slope", &config.blockControlM, TYPE_DOUBLE, {.val_d=8.75}},
        { "block_fan_intercept", &config.blockControlB, TYPE_DOUBLE, {.val_d=-276.25}},
        { "block_fan_T_Start", &config.blockTemperatureFanStart, TYPE_DOUBLE, {.val_d=30}},
        { "block_fan_T_Max", &config.blockTemperatureFanMax, TYPE_DOUBLE, {.val_d=40}},
        { "block_fan_Min_Speed", &config.blockFanMinSpeed, TYPE_DOUBLE, {.val_d=30}},
        { "block_fan_Max_Speed", &config.blockFanMaxSpeed, TYPE_DOUBLE, {.val_d=100}},

        ////Filament and Nozzle configs
        {"bcodeStr",config.bcodeStr , TYPE_STRING, {.val_s=0}},
        {"Filament_in_Spool", &config.filament_in_spool, TYPE_DOUBLE, {.val_d=0.0}},
        {"nozzleSize",&config.nozzleSize , TYPE_INT, {.val_i=400}},

        //Battery Variables
        {"battery_Print_Time",&config.batteryPrintTime , TYPE_INT, {.val_i=180}},
        {"standBy_Time",&config.standByTime , TYPE_INT, {.val_i=18000}},
        {"auto_Resume",&config.autoResume , TYPE_INT, {.val_i=1}},

        //Power Saving
        {"Power_Saving_Wait_Time",&config.powerSavingWaitTime , TYPE_INT, {.val_i=600}}

    };

#define NUM_TOKENS (sizeof(config_lookup)/sizeof(tConfigItem))


uint16_t read_u16 (FIL *file, char *line)
{
  f_gets(line, 80, file); /* read one line */
  char *p_pos = strchr(line, '='); /* find the '=' position */

  if (p_pos != NULL)
    return (atoi(p_pos+1));
  else
    return 0;
}

// return true if c matches any character in s
bool char_match (char c, char *s)
{
  while (*s)
    {
      if (*s == c)
        return true;
      s++;
    }
  return false;
}

// a version of strtok(), recognises identifiers, integers, and single-char symbols
// NB: very unsafe; not re-entrant. Use with caution!
char *get_token (char *pLine)
{
  static char *pNext;
  static char saved;
  char *pToken = NULL;

  if (pLine)
    {
      pNext = pLine;
    }
  else if (pNext)
    {
      *pNext = saved;
    }

  if (!pNext)
    return NULL;

  // skip white space
  while (*pNext && char_match (*pNext, " \t\n") )
    {
      pNext ++;
    }

  if (*pNext == 0)
    // reached end of string
    return NULL;
  else
    {
      // find next token
      pToken = pNext;

      if (isalpha (*pNext))
        {
          // identifier is alpha (alpha|digit|"_")*
          while (*pNext && ( isalpha(*pNext) || isdigit(*pNext) || (*pNext == '_' ) ) )
            {
              pNext ++;
            }
        }
      else if (isdigit (*pNext) || char_match (*pNext, "+-"))
        {
          // number is [+|-] (digit)+ [. digit+]
          pNext ++;
          while (*pNext && isdigit (*pNext) )
            {
              pNext ++;
            }
          if (*pNext && *pNext == '.')
            {
              pNext ++;
              while (*pNext && isdigit (*pNext) )
                {
                  pNext ++;
                }
            }
        }
      else
        {
          // anything else is presumed to be single char token, e.g. "="
          pNext ++;
        }

      saved = *pNext;
      *pNext = 0;
      return pToken;
    }
}

double atod (char *s)
{
  double result = 0.0;
  int num_places = 0;
  double frac = 0.0;

  while (*s && *s != '.')
    {
      result *= 10.0;
      result += *s-'0';
      s++;
    }
  if (*s && *s=='.')
    {
      s++;

      while (*s)
        {
          frac *= 10.0;
          frac += *s-'0';
          s++;
          num_places++;
        }
      while (num_places--)
        frac /= 10.0;
      result += frac;
    }
  return result;
}

void print_config (void)
{
  unsigned j;

  for (j=0; (j < NUM_TOKENS); j++)
    {
      switch (config_lookup[j].type)
      {
      case TYPE_INT:
        {
          int32_t *pVal = config_lookup[j].pValue;
          sersendf ("%s = %d\r\n", config_lookup[j].name, *pVal);
          break;
        }
      case TYPE_DOUBLE:
        {
          double *pVal = config_lookup[j].pValue;
          sersendf ("%s = %g\r\n", config_lookup[j].name, *pVal);
          break;
        }
        // ToDo
      case TYPE_STRING:
        {
          char* pVal = config_lookup[j].pValue;
          sersendf ("%s = %s\r\n", config_lookup[j].name, pVal);
          break;
        }
      }
    }
}



bool read_config (void)
{
    unsigned j;
    char *pmem = SECTOR_29_START;
    size_t bytes = (sizeof(config)/sizeof(char));
    char* pConfig = &config;
    char read_err = 0;

    //copy from ram to config
    memcpy(pConfig, pmem, bytes);

    //compares to check if reading is ok
    for (j=0; j < NUM_TOKENS; j++){
        switch (config_lookup[j].type){
            case TYPE_INT:
            {
                int32_t *pVal = config_lookup[j].pValue;
                if(*pVal==BAD_INT && config_lookup[j].val_i != BAD_INT){
                    read_err = 1;
                }/*No need for else*/
                break;
            }
            case TYPE_DOUBLE:
            {
                double *pVal = config_lookup[j].pValue;
                if(*pVal==BAD_DOUBLE && config_lookup[j].val_d != BAD_DOUBLE){
                    read_err = 1;
                }/*No need for else*/
                break;
            }

            // TODO
            case TYPE_STRING:
            {
              char* pVal = config_lookup[j].pValue;
              if(pVal[0]=='\0'){
                  read_err = 1;
              }
              break;
            }
        }
    }

    /*
    if(config.kp < 0.0001
        && config.ki < 0.0001
        && config.kd < 0.0001){

        config.kp = config_lookup[43].val_d;
        config.ki = config_lookup[44].val_d;
        config.kd = config_lookup[45].val_d;

    }*//*No need for else*/

    if(read_err){
        reset_config();
        return false;
    }/*No need for else*/

    /* Initialize using values read from "config.txt" file */
    gcode_parse_init();

    return true;
}
void reset_config (void)
{
    unsigned j;

    // first set defaults
    for (j=0; j < NUM_TOKENS; j++){
        switch (config_lookup[j].type){
            case TYPE_INT:
            {
                int32_t *pVal = config_lookup[j].pValue;
                *pVal = config_lookup[j].val_i;
                break;
            }
            case TYPE_DOUBLE:
            {
                double *pVal = config_lookup[j].pValue;
                *pVal = config_lookup[j].val_d;
                break;
            }
            case TYPE_STRING:
            {
              char* pVal = config_lookup[j].pValue;
              strcpy(pVal, "_no_file");
            }
        }
    }

    char *pmem = SECTOR_29_START;
    uint32_t bytes = (sizeof(config)/sizeof(char));
    char sector[bytes];
    char* pConfig = &config;

    memcpy(&sector, pConfig, bytes);

    prepare_sector(29, 29, SystemCoreClock);
    erase_sector(29, 29, SystemCoreClock);

    prepare_sector(29, 29, SystemCoreClock);
    write_data(   (unsigned)(SystemCoreClock/1000),
                            (unsigned)(SECTOR_29_START),
                            (unsigned)sector,
                            (unsigned)FLASH_BUF_SIZE);

    compare_data((unsigned)(SystemCoreClock/1000),
                            (unsigned)(SECTOR_29_START),
                            (unsigned)sector,
                            (unsigned)FLASH_BUF_SIZE);

    /* Initialize using values read from "config.txt" file */
    gcode_parse_init();

}

void write_config (void)
{
    size_t bytes = sizeof(config)/sizeof(char);
    char sector[bytes];
    char* pConfig = &config;

    memcpy(&sector, pConfig, bytes);

    prepare_sector(29, 29, SystemCoreClock);
    erase_sector(29, 29, SystemCoreClock);

    prepare_sector(29, 29, SystemCoreClock);
    write_data(  (unsigned)(SystemCoreClock/1000),
                           (unsigned)(SECTOR_29_START),
                           (unsigned)sector,
                           (unsigned)FLASH_BUF_SIZE);

    compare_data((unsigned)(SystemCoreClock/1000),
                           (unsigned)(SECTOR_29_START),
                           (unsigned)sector,
                           (unsigned)FLASH_BUF_SIZE);

}
