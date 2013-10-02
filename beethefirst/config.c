/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
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
typedef struct {
  char      *name;
  void      *pValue;
  uint8_t   type;
  union {
    int32_t   val_i;
    double    val_d;
  };
} tConfigItem;

/* calculate the default values appropriate for your machine */
tConfigItem config_lookup [] = 
    {
        { "machine_model", &config.machine_model, TYPE_INT, {.val_i=0}},

        { "steps_per_mm_x", &config.steps_per_mm_x, TYPE_DOUBLE, {.val_d=78.778}},
        { "steps_per_mm_y", &config.steps_per_mm_y, TYPE_DOUBLE, {.val_d=78.778}},
        { "steps_per_mm_z", &config.steps_per_mm_z, TYPE_DOUBLE, {.val_d=112.540}},
        { "steps_per_mm_e", &config.steps_per_mm_e, TYPE_DOUBLE, {.val_d=517.000}},    /* Wades extruder, NEMA 17 geared extruder (1/39 * 6.5mm) */

        /* used for G0 rapid moves and as a cap for all other feedrates */
        { "maximum_feedrate_x", &config.maximum_feedrate_x, TYPE_INT, {.val_i=60000}}, /* 50mm / second */
        { "maximum_feedrate_y", &config.maximum_feedrate_y, TYPE_INT, {.val_i=60000}},
        { "maximum_feedrate_z", &config.maximum_feedrate_z, TYPE_INT, {.val_i=60000}},   /* 1mm / second */
        { "maximum_feedrate_e", &config.maximum_feedrate_e, TYPE_INT, {.val_i=60000}}, /* 50mm / second */

        { "acceleration",       &config.acceleration, TYPE_DOUBLE, {.val_d=1500.000}},         /* 100mm / second^2 */
        { "junction_deviation", &config.junction_deviation, TYPE_DOUBLE, {.val_d=0.050}},

        /* used when searching endstops and similar */
        { "search_feedrate_x", &config.search_feedrate_x, TYPE_INT, {.val_i=1000}},
        { "search_feedrate_y", &config.search_feedrate_y, TYPE_INT, {.val_i=1000}},
        { "search_feedrate_z", &config.search_feedrate_z, TYPE_INT, {.val_i=1000}},
        { "search_feedrate_e", &config.search_feedrate_e, TYPE_INT, {.val_i=1000}},

        { "homing_feedrate_x", &config.homing_feedrate_x, TYPE_INT, {.val_i=2500}},
        { "homing_feedrate_y", &config.homing_feedrate_y, TYPE_INT, {.val_i=2500}},
        { "homing_feedrate_z", &config.homing_feedrate_z, TYPE_INT, {.val_i=2500}},

        // home pos is left front
        { "home_direction_x", &config.home_direction_x, TYPE_INT, {.val_i=-1}},
        { "home_direction_y", &config.home_direction_y, TYPE_INT, {.val_i=-1}},
        { "home_direction_z", &config.home_direction_z, TYPE_INT, {.val_i=1}},

        { "home_pos_x", &config.home_pos_x, TYPE_DOUBLE, {.val_d=0.0}},
        { "home_pos_y", &config.home_pos_y, TYPE_DOUBLE, {.val_d=0.0}},
        { "home_pos_z", &config.home_pos_z, TYPE_DOUBLE, {.val_d=123.495}},

        { "printing_vol_x", &config.printing_vol_x , TYPE_INT, {.val_i=120}},
        { "printing_vol_y", &config.printing_vol_y , TYPE_INT, {.val_i=80}},
        { "printing_vol_z", &config.printing_vol_z , TYPE_INT, {.val_i=85}},

        // dump pos
        { "have_dump_pos", &config.have_dump_pos , TYPE_INT, {.val_i=0}},
        { "dump_pos_x", &config.dump_pos_x , TYPE_INT, {.val_i=120}},
        { "dump_pos_y", &config.dump_pos_x , TYPE_INT, {.val_i=120}},

        // rest pos
        { "have_rest_pos", &config.have_rest_pos , TYPE_INT, {.val_i=1}},
        { "rest_pos_x", &config.rest_pos_x , TYPE_INT, {.val_i=120}},
        { "rest_pos_y", &config.rest_pos_y , TYPE_INT, {.val_i=120}},

        // wipe pos
        { "have_wipe_pos",   &config.have_wipe_pos , TYPE_INT, {.val_i=0}},
        { "wipe_entry_pos_x", &config.wipe_entry_pos_x , TYPE_INT, {.val_i=0}},
        { "wipe_entry_pos_y", &config.wipe_entry_pos_y , TYPE_INT, {.val_i=0}},
        { "wipe_pos_x", &config.wipe_entry_pos_x , TYPE_INT, {.val_i=0}},     // DEPRECATED
        { "wipe_pos_y", &config.wipe_entry_pos_y , TYPE_INT, {.val_i=0}},     // DEPRECATED
        { "wipe_exit_pos_x", &config.wipe_exit_pos_x , TYPE_INT, {.val_i=0}},
        { "wipe_exit_pos_y", &config.wipe_exit_pos_y , TYPE_INT, {.val_i=0}},

        { "steps_per_revolution_e", &config.steps_per_revolution_e, TYPE_INT, {.val_i=3200}},  // 200 * 16

        { "wait_on_temp", &config.wait_on_temp, TYPE_INT, {.val_i=0}},

        { "enable_extruder_1", &config.enable_extruder_1, TYPE_INT, {.val_i=1}},
        { "status", &config.status, TYPE_INT, {.val_i=1}},
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
      }
    }
}



void read_config (void)
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
        }
    }


    char sector[FLASH_BUF_SIZE];
    char *pmem = SECTOR_29_START;
    size_t bytes = (sizeof(config)/sizeof(char));
    char* pConfig = &config;

    if(*pmem != 0xFF){
        memcpy(pConfig, pmem, bytes);
    }else{
        memcpy(&sector, pConfig, bytes);

        while (bytes < FLASH_BUF_SIZE) {
            sector[bytes] = 255;
            bytes++;
        }

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
    }

    /* Initialize using values read from "config.txt" file */
    gcode_parse_init();

}


void write_config (void)
{
    char sector[FLASH_BUF_SIZE];
    size_t bytes = sizeof(config)/sizeof(char);
    char* pConfig = &config;

    memcpy(&sector, pConfig, bytes);

    while (bytes  < FLASH_BUF_SIZE) {
       sector[bytes] = 255;
       bytes ++;
    }

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
