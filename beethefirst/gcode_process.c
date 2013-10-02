/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
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

#include <string.h>
#include "gcode_process.h"
#include "gcode_parse.h"
#include "serial.h"
#include "sermsg.h"
#include "sersendf.h"
#include "temp.h"
#include "timer.h"
#include "pinout.h"
#include "config.h"
#include "ff.h"
#include "buzzer.h"
//#include "debug.h"
#include "planner.h"
#include "stepper.h"
#include "geometry.h"
#include "bootloader.h"
#include "sbl_config.h"
#include "system_LPC17xx.h"

FIL       file;
uint32_t  filesize = 0;
uint32_t  sd_pos = 0;
bool      sd_printing = false;      // printing from SD file
bool      sd_active = false;        // SD card active
bool      sd_writing_file = false;  // writing to SD file

#define EXTRUDER_NUM_1  1
#define EXTRUDER_NUM_2  2
#define EXTRUDER_NUM_3  4

uint8_t   extruders_on;
double    extruder_1_speed;         // in RPM

uint32_t  auto_prime_steps = 0;
uint32_t  auto_reverse_steps = 0;
const double auto_prime_feed_rate = 18000;
const double auto_reverse_feed_rate = 18000;
double auto_prime_factor = 640;
double auto_reverse_factor = 640;

static void enqueue_moved (tTarget *pTarget)
{
  // grbl
  tActionRequest request;

  if (pTarget->x != startpoint.x || pTarget->y != startpoint.y ||
      pTarget->z != startpoint.z || pTarget->e != startpoint.e
  )
  {
    request.ActionType = AT_MOVE;
    request.target= *pTarget;
    request.target.invert_feed_rate =  false;

    if (config.enable_extruder_1 == 0)
      request.target.e = startpoint.e;

    plan_buffer_action (&request);
  }
  else
  {
    // no move, just set feed rate
    plan_set_feed_rate (pTarget);
  }
}

static void enqueue_wait_temp (void)
{
  tActionRequest request;

  request.ActionType = AT_WAIT_TEMPS;
  plan_buffer_action (&request);
}

static void enqueue_wait (void)
{
  tActionRequest request;

  request.ActionType = AT_WAIT;
  plan_buffer_action (&request);
}

// wait for move queue to be empty
static void synch_queue (void)
{
  st_synchronize();
}

static void SpecialMoveXY(double x, double y, double f) 
{
  tActionRequest request;

  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = x;
  request.target.y = y;
  request.target.z = startpoint.z;
  request.target.e = startpoint.e;
  request.target.feed_rate = f; 
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

static void SpecialMoveZ(double z, double f) 
{
  tActionRequest request;

  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = startpoint.x;
  request.target.y = startpoint.y;
  request.target.z = z;
  request.target.e = startpoint.e;
  request.target.feed_rate = f; 
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

static void SpecialMoveE (double e, double feed_rate) 
{
  tTarget next_targetd;

  if (config.enable_extruder_1)
  {
    next_targetd = startpoint;
    next_targetd.e = startpoint.e + e;
    next_targetd.feed_rate = feed_rate;
    enqueue_moved(&next_targetd);
  }
}

static void zero_x(void)
{
  int dir;
  int max_travel;

  if (config.home_direction_x < 0)
  {
    dir = -1;
  }
  else
  {
    dir = 1;
  }
  max_travel = max (300, config.printing_vol_x);

  // move to endstop
  SpecialMoveXY(startpoint.x + dir * max_travel, startpoint.y, config.homing_feedrate_x);
  synch_queue();

  // move forward a bit
  SpecialMoveXY(startpoint.x - dir * 3, startpoint.y, config.search_feedrate_x);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x + dir * 6, startpoint.y, config.search_feedrate_x);

  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;

  //R2C2: XICO B3.1 HOT FIX
  //new_pos.x = config.home_pos_x;
  new_pos.x = -96.0;
  plan_set_current_position (&new_pos);
}

static void zero_y(void)
{
  int dir;
  int max_travel;

  if (config.home_direction_y < 0)
  {
    dir = -1;
  }
  else
  {
    dir = 1;
  }
  max_travel = max (300, config.printing_vol_y);

  // move to endstop
  SpecialMoveXY(startpoint.x, startpoint.y + dir * max_travel, config.homing_feedrate_y);
  synch_queue();

  // move forward a bit
  SpecialMoveXY(startpoint.x, startpoint.y - dir * 3, config.search_feedrate_y);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x, startpoint.y + dir * 6, config.search_feedrate_y);

  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;

  //R2C2: XICO B3.1 HOT FIX
  //new_pos.y = config.home_pos_y;
  new_pos.y = -74.5;
  plan_set_current_position (&new_pos);
}

static void zero_z(void)
{
  int dir;
  int max_travel;

  if (config.home_direction_z < 0)
  {
    dir = -1;
  }
  else
  {
    dir = 1;
  }
  max_travel = max (300, config.printing_vol_z);

  // move to endstop
  SpecialMoveZ(startpoint.z + dir * max_travel, config.homing_feedrate_z);  
  synch_queue();


  /**
   * Normal move Z
   **/

    tTarget next_targetd = startpoint;
      next_targetd.x = startpoint.x;
      next_targetd.y = startpoint.y;
      next_targetd.z = startpoint.z - dir * 10;
      next_targetd.e = startpoint.e;
      next_targetd.feed_rate =  config.homing_feedrate_z;
      enqueue_moved(&next_targetd);
      synch_queue();
  /*
   * end
   */
  // move forward a bit
  //SpecialMoveZ(startpoint.z - dir * 1, config.search_feedrate_z);
  //synch_queue();

  // move back in to endstop slowly
  SpecialMoveZ(startpoint.z + dir *15 , config.search_feedrate_z);
  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;
  new_pos.z = config.home_pos_z;

  plan_set_current_position (&new_pos);
}

static void zero_e(void)
{
  // extruder only runs one way and we have no "endstop", just set this point as home
  //startpoint.E = current_position.E = 0;
  tTarget new_pos = startpoint;
  new_pos.e = 0;
  plan_set_current_position (&new_pos);
}

void sd_initialise(void)
{
  sd_active = true;
}

FRESULT sd_list_dir_sub (char *path)
{
  FRESULT res;
  FILINFO fno;
  DIR dir;
  int i;
  char *fn;
#if _USE_LFN
  static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
  fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
#endif

  res = f_opendir(&dir, path);
  if (res == FR_OK)
  {
    i = strlen(path);
    for (;;)
    {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0) break;
      if (fno.fname[0] == '.') continue;
#if _USE_LFN
      fn = *fno.lfname ? fno.lfname : fno.fname;
#else
      fn = fno.fname;
#endif
      if (fno.fattrib & AM_DIR)
      {
        sersendf("%s/%s/\r\n", path, fn);

        strcat (path, "/");
        strcat (path, fn);
        // sprintf(&path[i], "/%s", fn);
        res = sd_list_dir_sub(path);
        if (res != FR_OK)
        {
          break;
        }
        path[i] = 0;
      }
      else
      {
        sersendf("%s/%s\r\n", path, fn);
      }
    }
  }

  return res;
}

void sd_list_dir (void)
{
  char path[120];

  strcpy (path, "");

  sd_list_dir_sub(path);
}

unsigned sd_open(FIL *pFile, char *path, uint8_t flags)
{
  FRESULT res;

  res = f_open (pFile, path, flags);

  if (res == FR_OK)
  {
    return 1;
  }
  else
  {
    //debug
    sersendf ("sd_open:%d", res);
    return 0;
  }
}

void sd_close(FIL *pFile)
{
  f_close (pFile);
}

bool sd_read_file(tLineBuffer *pLine)
{
  char *ptr;

  ptr = f_gets(pLine->data, MAX_LINE, &file);

  if (ptr != NULL)
  {
    pLine->len = strlen(ptr);
    sd_pos += pLine->len;
    return true;
  }
  else
  {
    return false;
  }
}

bool sd_write_to_file(char *pStr, unsigned bytes_to_write)
{
  UINT bytes_written;
  FRESULT result;

  result = f_write (&file, pStr, bytes_to_write, &bytes_written);

  return result == FR_OK;
}

unsigned sd_filesize (FIL *pFile)
{
  return f_size(pFile);
}

void sd_seek(FIL *pFile, unsigned pos)
{
  f_lseek (pFile, pos);
}

/****************************************************************************
 *                                                                           *
 * Command Received - process it                                             *
 *                                                                           *
 ****************************************************************************/

eParseResult process_gcode_command()
{
    double backup_f;
    uint8_t axisSelected = 0;
    eParseResult result = PR_OK;
    bool reply_sent = false;

    tTarget next_targetd = startpoint;

    // convert relative to absolute
    if (next_target.option_relative)
    {
        next_targetd.x = startpoint.x + next_target.target.x;
        next_targetd.y = startpoint.y + next_target.target.y;
        next_targetd.z = startpoint.z + next_target.target.z;
        next_targetd.e = startpoint.e + next_target.target.e;
        if (next_target.seen_F)
            next_targetd.feed_rate = next_target.target.feed_rate;
    }
    else
    {
        // absolute
        if (next_target.seen_X)
            next_targetd.x = next_target.target.x;
        if (next_target.seen_Y)
            next_targetd.y = next_target.target.y;
        if (next_target.seen_Z)
            next_targetd.z = next_target.target.z;
        if (next_target.seen_E)
            next_targetd.e = next_target.target.e;
        if (next_target.seen_F)
            next_targetd.feed_rate = next_target.target.feed_rate;
    }

    //  sersendf(" X:%ld Y:%ld Z:%ld E:%ld F:%ld\r\n", (int32_t)next_target.target.X, (int32_t)next_target.target.Y, (int32_t)next_target.target.Z, (int32_t)next_target.target.E, (uint32_t)next_target.target.F);
    //  sersendf(" X:%g Y:%g Z:%g E:%g F:%g\r\n", next_targetd.x, next_targetd.y, next_targetd.z, next_targetd.e, next_targetd.feed_rate);

    // E ALWAYS absolute
    // host should periodically reset E with "G92 E0", otherwise we overflow our registers after only a few layers

  if (next_target.seen_G)
  {
    switch (next_target.G)
    {
    // G0 - rapid, unsynchronised motion
    // since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
      case 0:
      {
#if 0
                backup_f = next_targetd.feed_rate;
                next_targetd.feed_rate = config.maximum_feedrate_x * 2;
                enqueue_moved (&next_targetd);
                next_targetd.feed_rate = backup_f;
#endif
      }
      break;

      // G1 - synchronised motion
      case 1:
      {
          config.status = 4;
#if 0
          if((extruders_on == EXTRUDER_NUM_1) && !next_target.seen_E){
              // approximate translation for 3D code. distance to extrude is move distance times extruder speed factor
              //TODO: extrude distance for Z moves
              double d = calc_distance (ABS(next_targetd.x - startpoint.x), ABS(next_targetd.y - startpoint.y));

              next_targetd.e = startpoint.e + d * extruder_1_speed / next_targetd.feed_rate * 24.0;
          }
#endif
          enqueue_moved(&next_targetd);
      }
      break;

      //	G4 - Dwell
      case 4:
      {
        config.status = 0;

#if 0
        // wait for all moves to complete
        //synch_queue();

        // delay
        delay_ms(next_target.P);
#endif
      }
      break;

      //	G20 - inches as units
      case 20:
      {
        config.status = 0;

#if 0
        next_target.option_inches = 1;
#endif
      }
      break;

      //	G21 - mm as units
      case 21:
      {
        config.status = 0;

#if 0
        next_target.option_inches = 1;
#endif
      }
      break;

      //	G30 - go home via point
      case 30:
      {
          config.status = 0;

#if 0
          enqueue_moved(&next_targetd);
          if (next_target.seen_X)
          {
               zero_x();
               axisSelected = 1;
          }

          if (next_target.seen_Y)
          {
               zero_y();
               axisSelected = 1;
          }

          if (next_target.seen_Z)
          {
               zero_z();
               axisSelected = 1;
          }

          if (next_target.seen_E)
          {
               zero_e();
               axisSelected = 1;
          }

          if(!axisSelected)
          {
               if (config.machine_model == MM_RAPMAN)
               {
                    // move stage down to clear Z endstop
                    // Rapman only?
                    next_targetd = startpoint;
                    next_targetd.z += 3;
                    next_targetd.feed_rate = config.homing_feedrate_z;
                    enqueue_moved(&next_targetd);
                }

                zero_x();
                zero_y();
                zero_z();
                zero_e();
          }
#endif
      }
      break;

      //	G28 - go home
      case 28:
      {
          config.status = 4;

          if (next_target.seen_X)
          {
              zero_x();
              axisSelected = 1;
          }

          if (next_target.seen_Y)
          {
              zero_y();
              axisSelected = 1;
          }

          if (next_target.seen_Z)
          {
              zero_z();
              axisSelected = 1;
          }

          if (next_target.seen_E)
          {
              zero_e();
              axisSelected = 1;
          }

          if(!axisSelected)
          {
              if (config.machine_model == MM_RAPMAN)
              {
                  // move stage down to clear Z endstop
                  // Rapman only?
                  next_targetd = startpoint;
                  next_targetd.z += 3;
                  next_targetd.feed_rate = config.homing_feedrate_z;
                  enqueue_moved(&next_targetd);
              }

              zero_x();
              zero_y();
              zero_z();
              zero_e();
          }
      }
      break;

      // G90 - absolute positioning
      case 90:
      {
          config.status = 0;
#if 0
          next_target.option_relative = 0;
#endif
      }
      break;

      // G91 - relative positioning
      case 91:
      {
          config.status = 0;

#if 0
          next_target.option_relative = 1;
#endif
      }
      break;

      //	G92 - set current position
      case 92:
      {
          config.status = 4;
          tTarget new_pos;

          // must have no moves pending if changing position
          synch_queue();

          new_pos = startpoint;

          if (next_target.seen_X)
          {
            new_pos.x = next_target.target.x;
            axisSelected = 1;
          }

          if (next_target.seen_Y)
          {
            new_pos.y = next_target.target.y;
            axisSelected = 1;
          }

          if (next_target.seen_Z)
          {
            new_pos.z = next_target.target.z;
            axisSelected = 1;
          }

          if (next_target.seen_E)
          {
            new_pos.e = 0;
            axisSelected = 1;
          }

          if(!axisSelected)
          {
            new_pos.x = 0;
            new_pos.y = 0;
            new_pos.z = 0;
            new_pos.e = 0;
          }

          plan_set_current_position (&new_pos);
      }
      break;

      // unknown gcode: spit an error
      default:
        config.status = 0;
        serial_writestr("ok - Error: Bad G-code ");
        serwrite_uint8(next_target.G);
        serial_writestr("\r\n");
    }
  }
  else if (next_target.seen_M)
  {
    switch (next_target.M)
    {
      // SD File functions
      case 20: // M20 - list SD Card files
      {

          config.status = 0;

#if 0
          serial_writestr("Begin file list\r\n");
          // list files in root folder
          sd_list_dir();
          serial_writestr("End file list\r\n");
#endif
      }
      break;

      case 21: // M21 - init SD card
      {
          config.status = 0;

#if 0
          sd_printing = false;
          sd_initialise();
          // NB : assume that the disk has been mounted in config.c
          // TODO: mount volume here and change config.c
#endif
      }
      break;


      case 22: // M22 - release SD card
      {
        config.status = 0;

#if 0
        sd_printing = false;
        sd_active = false;
        // TODO: should unmount volume
#endif
      }
      break;

      case 23: // M23 <filename> - Select file
      {

        config.status = 0;

#if 0
          if (!sd_active)
          {
              sd_initialise();
          }
          if(sd_active)
          {
              sd_printing = false;
              sd_close(&file);
              if (sd_open(&file, next_target.filename, FA_READ))
              {
                  filesize = sd_filesize(&file);
                  sersendf("File opened: %s Size: %d\r\n", next_target.filename, filesize);
                  sd_pos = 0;
                  sersendf("File selected\r\n");
              }
              else
              {
                  sersendf("file.open failed\r\n");
              }
          }
#endif
      }
      break;


      case 24: //M24 - Start SD print
      {
          if(sd_active)
          {
              config.status = 5;
              sd_printing = true;
          }
      }
      break;

      case 25: //M25 - Pause SD print
      {
          if(sd_printing)
          {
              sd_printing = false;
              config.status = 3;

          }
      }
      break;

      case 26: //M26 - Set SD file pos
      {

          config.status = 0;

#if 0
          if(sd_active && next_target.seen_S)
          {
            sd_pos = next_target.S;  // 16 bit
            sd_seek(&file, sd_pos);
          }
#endif
      }
      break;

      case 27: //M27 - Get SD status
      {
          config.status = 0;

#if 0
          if(sd_active)
          {
              sersendf("SD printing byte %d/%d\r\n", sd_pos, filesize);
          }
          else
          {
              serial_writestr("Not SD printing\r\n");
          }
#endif
      }
      break;

      case 28: //M28 <filename> - Start SD write
      {
          config.status = 0;

#if 0
          if (!sd_active)
          {
              sd_initialise();
          }
          if(sd_active)
          {
              sd_close(&file);
              sd_printing = false;

              if (!sd_open(&file, next_target.filename, FA_CREATE_ALWAYS | FA_WRITE))
              {
                  sersendf("open failed, File: %s.\r\n", next_target.filename);
              }
              else
              {
                  sd_writing_file = true;
                  sersendf("Writing to file: %s\r\n", next_target.filename);
              }
          }
#endif
      }
      break;

      case 29: //M29 - Stop SD write
      {
          config.status = 0;
      }
      break;

      case 82: // M82 - use absolute distance for extrusion
      {
          config.status = 0;
      }
      break;

      // M101- extruder on
      case 101:
      {
          config.status = 0;

#if 0
          extruders_on = EXTRUDER_NUM_1;
          if (auto_prime_steps != 0)
          {
              SpecialMoveE ((double)auto_prime_steps / auto_prime_factor, auto_prime_feed_rate);
          }
#endif
      }
      break;

      // M103- extruder off
      case 103:
      {
          config.status = 0;

#if 0
          extruders_on = 0;
          if (auto_reverse_steps != 0)
          {
            SpecialMoveE (-(double)auto_reverse_steps / auto_reverse_factor, auto_reverse_feed_rate);
          }
#endif
      }
      break;

      // M104- set temperature
      case 104:
      {
          if(sd_printing)
              break;

          if (config.enable_extruder_1)
          {
              temp_set(next_target.S, EXTRUDER_0);

              if (config.wait_on_temp)
              {
                  enqueue_wait_temp();
              }
          }
      }
      break;

      // M105- get temperature
      case 105:
      {
          temp_print();
      }
      break;

      // M106- fan on
      case 106:
      {
          extruder_fan_on();
      }
      break;

      // M107- fan off
      case 107:
      {
          extruder_fan_off();
      }
      break;

      // M108 - set extruder speed
      // S = RPM * 10
      case 108:
      {
          config.status = 0;

#if 0
          // S = RPM * 10
          if (next_target.seen_S)
          {
             extruder_1_speed = (double)next_target.S / 10.0;
          }
#endif
      }
      break;

      // M109- set temp and wait
      case 109:
      {
          if(sd_printing)
              break;

          config.status = 4;

          if (config.enable_extruder_1)
          {
              temp_set(next_target.S, EXTRUDER_0);
              enqueue_wait_temp();
          }
      }

      break;

      // M110- set line number
      case 110:
      {
          config.status = 0;

#if 0
          next_target.N_expected = next_target.S - 1;
#endif
      }
      break;

      // M111- set debug level
      case 111:
      {
          config.status = 0;
      }
      break;

      // M112- immediate stop
      case 112:
      {
          config.status = 0;

          st_go_idle();

          // disable extruder and bed heaters
          temp_set(0, EXTRUDER_0);
          temp_set(0, HEATED_BED_0);

          queue_flush();
      }
      break;

      // M113- extruder PWM
      case 113:
      {
          config.status = 0;
          queue_flush();
      }
      break;
      // M115 - report firmware version
      case 115:
      {
          serial_writestr(" 2.1.1");
          serial_writestr(" ");
      }
      break;

      // M116 - Wait for all temperatures and other slowly-changing variables to arrive at their set values.
      case 116:
      {
          config.status = 0;

#if 0
          if (config.enable_extruder_1)
          {
              enqueue_wait();
          }
#endif
      }
      break;

      case 117:
      {
          //try to read the unique ID - not working in this LCP Revision
          /*
           serial_writestr(" ");
           read_device_serial_number();
           serial_writestr("\r\n");
           */

          char serialnumber[10] = {0};
          char *pmem117;

          pmem117 = SECTOR_14_START;

          for (int i = 0; i < 10; i++) {
              serialnumber[9 - i] = *pmem117;
              pmem117++;
          }

          serial_writeblock(serialnumber, 10);
          serial_writestr(" ");
      }
      break;

      /* M114- report XYZE to host */
       case 118:
       {
           if (next_target.option_inches)
          {
               config.status = 0;

          }
          else
          {

             sersendf(" C: X:%g Y:%g Z:%g E:%g", startpoint.x, startpoint.y, startpoint.z, startpoint.e);
          }
       }
       break;

      case 119:
      {
          config.status = 0;

/*
          #if (X_MIN_PIN > -1)
                serial_writestr ("x_min:");
                serial_writestr ( x_min() ? "H ":"L ");
          #endif
          #if (Y_MIN_PIN > -1)
                serial_writestr ("y_min:");
                serial_writestr ( y_min() ? "H ":"L ");
          #endif
          #if (Z_MIN_PIN > -1)
                serial_writestr ("z_min:");
                serial_writestr ( z_min() ? "H ":"L ");
          #endif
                serial_writestr ("\r\n");
                break;

                // M130- heater P factor
                case 130:
                  serial_writestr("ok N:");
                  serwrite_uint32(next_target.N);
                  serial_writestr(" M");
                  serwrite_uint32(next_target.M);
                  serial_writestr("-invalid comand\r\n");
                //if (next_target.seen_S)
                //p_factor = next_target.S;
*/
      }
      break;

      // M130- heater P factor
      case 130:
      {
          config.status = 0;
      }
      break;

      // M131- heater I factor
      case 131:
      {
          config.status = 0;

#if 0
          if (next_target.seen_S)
              i_factor = next_target.S;
#endif
      }
      break;

      // M132- heater D factor
      case 132:
      {
          config.status = 0;

#if 0
          if (next_target.seen_S)
              d_factor = next_target.S;
#endif
      }
      break;

      // M133- heater I limit
      case 133:
      {
          config.status = 0;

#if 0
          if (next_target.seen_S)
              i_limit = next_target.S;
#endif
      }
      break;

      // M134- save PID settings to eeprom
      case 134:
      {

          config.status = 0;

#if 0
          heater_save_settings();
#endif
      }
      break;

      /* M140 - Bed Temperature (Fast) */
      case 140:
      {
          config.status = 0;

#if 0
          temp_set(next_target.S, HEATED_BED_0);
#endif
      }
      break;

      /* M141 - Chamber Temperature (Fast) */
      case 141:
      {

          config.status = 0;
      }
      break;

      /* M142 - Bed Holding Pressure */
      case 142:
      {
        config.status = 0;
      }
      break;

      // M190- power on
      case 190:
      {
          config.status = 0;

#if 0
          power_on();
          x_enable();
          y_enable();
          z_enable();
          e_enable();
          steptimeout = 0;
#endif
      }
      break;

      // M191- power off
      case 191:
      {
          config.status = 0;

#if 0
          x_disable();
          y_disable();
          z_disable();
          e_disable();
          power_off();
#endif
      }
      break;

      // M200 - set steps per mm
      case 200:
      {
          if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0){
              sersendf ("X%g Y%g Z%g E%g",
                        config.steps_per_mm_x,
                        config.steps_per_mm_y,
                        config.steps_per_mm_z,
                        config.steps_per_mm_e);
          }else{
              if (next_target.seen_X){
                  config.steps_per_mm_x = next_target.target.x;
              }
              if (next_target.seen_Y){
                  config.steps_per_mm_y = next_target.target.y;
              }
              if (next_target.seen_Z){
                  config.steps_per_mm_z = next_target.target.z;
              }
              if (next_target.seen_E){
                  config.steps_per_mm_e = next_target.target.e;
              }

              gcode_parse_init();
          }
      }
      break;

      // M202 - set max speed in mm/min
      case 202:
      {
          config.status = 0;

  #if 0
          if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0)
          {
              sersendf ("ok X%d Y%d Z%d E%d\r\n",
                  config.maximum_feedrate_x,
                  config.maximum_feedrate_y,
                  config.maximum_feedrate_z,
                  config.maximum_feedrate_e);
          }else{
              if (next_target.seen_X){
                  config.maximum_feedrate_x = next_target.target.x;
              }
              if (next_target.seen_Y){
                  config.maximum_feedrate_y = next_target.target.y;
              }
              if (next_target.seen_Z){
                  config.maximum_feedrate_z = next_target.target.z;
              }
              if (next_target.seen_E){
                  config.maximum_feedrate_e = next_target.target.e;
              }
          }
  #endif
      }
      break;

      // M206 - set accel in mm/sec^2
      case 206:
      {
          if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0){

              sersendf (" X%g",
                        config.acceleration);
          }else{
              if (next_target.seen_X){
                  config.acceleration = next_target.target.x;
              }
          }
      }
      break;

      // M227 - Enable Auto-prime/reverse (steps)
      // P: prime on start (steps)
      // S: reverse on stop (steps)
      case 227:
      {
          config.status = 0;

#if 0
          if (next_target.seen_S && next_target.seen_P)
          {
              auto_prime_steps = next_target.S;
              auto_reverse_steps = next_target.P;
          }
#endif
      }
      break;

      // M228 - Disable Auto-prime/reverse
      case 228:
      {
          config.status = 0;

#if 0
          auto_prime_steps = 0;
          auto_reverse_steps = 0;
#endif
      }
      break;

      // M229 - Enable Auto-prime/reverse
      // P: prime on start (rotations)
      // S: reverse on stop (rotations)
      case 229:
      if (next_target.seen_S && next_target.seen_P)
      {
          auto_prime_steps = next_target.S * config.steps_per_revolution_e;
          auto_reverse_steps = next_target.P * config.steps_per_revolution_e;
      }
      break;

      // M300 - beep
      // S: frequency
      // P: duration
      case 300:
      {
          uint16_t frequency = 1000;  // 1kHz
          uint16_t duration = 1000; // 1 second

          if (next_target.seen_S){
              frequency = next_target.S;
          }
          if (next_target.seen_P){
              duration = next_target.P;
          }

          buzzer_wait ();
          buzzer_play (frequency, duration);
      }
      break;

      // M300 - beep
      // S: frequency
      // P: duration
      case 301:
      {
          uint16_t frequency = 1000;  // 1kHz
          uint16_t duration = 1000; // 1 second

          if (next_target.seen_S)
          {
              frequency = next_target.S;
          }
          if (next_target.seen_P)
          {
              duration = next_target.P;
          }

          buzzer_wait ();
          buzzer_play (frequency, duration);
      }
      break;

      // M500 - set/get adc value for temperature
      // S: temperature (degrees C, 0-300)
      // P: ADC val
      case 500:
      {
          if (next_target.seen_S && next_target.seen_P){
              temp_set_table_entry (EXTRUDER_0, next_target.S, next_target.P);
          }else if (next_target.seen_S){
              sersendf (" [%d] = %d", next_target.S, temp_get_table_entry (EXTRUDER_0, next_target.S));
          }else{
            serial_writestr ("E: bad param ");
          }
      }
      break;

      // M501 - set/get adc value for temperature
      // S: temperature (degrees C, 0-300)
      // P: ADC val
      case 501:
      {

          if (next_target.seen_S && next_target.seen_P){
              temp_set_table_entry (HEATED_BED_0, next_target.S, next_target.P);
          }else if (next_target.seen_S){
              sersendf (" [%d] = %d", next_target.S, temp_get_table_entry (HEATED_BED_0, next_target.S));
          }else{
              serial_writestr ("E: bad param ");
          }
      }
      break;

      // M542 - nozzle wipe/move to rest location
      case 542:
      {
          config.status = 0;

#if 0
          // TODO: this depends on current origin being same as home position
          if (config.have_rest_pos || config.have_wipe_pos)
          {
              // move above bed if ncessary
              if (startpoint.z < 2)
              {
                  next_targetd = startpoint;
                  next_targetd.z = 2;
                  next_targetd.feed_rate = config.maximum_feedrate_z;
                  enqueue_moved(&next_targetd);
              }

              if (config.have_wipe_pos)
              {
                  // move to start of wipe area
                  next_targetd.x = config.wipe_entry_pos_x;
                  next_targetd.y = config.wipe_entry_pos_y;
                  next_targetd.z = startpoint.z;
                  next_targetd.feed_rate = config.maximum_feedrate_x;
              }
              else
              {
                  // move to rest position
                  next_targetd.x = config.rest_pos_x;
                  next_targetd.y = config.rest_pos_y;
                  next_targetd.z = startpoint.z;
                  next_targetd.feed_rate = config.maximum_feedrate_x;
              }

              enqueue_moved(&next_targetd);
          }
#endif
      }
      break;

      // M543 - exit nozzle wipe/no op
      case 543:
      {
          config.status = 0;

#if 0
          if (config.have_wipe_pos)
          {
              // move out of wipe area
              next_targetd.x = config.wipe_exit_pos_x;
              next_targetd.y = config.wipe_exit_pos_y;
              next_targetd.z = startpoint.z;
              next_targetd.feed_rate = config.maximum_feedrate_x;
              enqueue_moved(&next_targetd);

              next_targetd.x = config.wipe_entry_pos_x;
              next_targetd.y = config.wipe_entry_pos_y;
              next_targetd.z = startpoint.z;
              next_targetd.feed_rate = config.maximum_feedrate_x;
              enqueue_moved(&next_targetd);

          }
#endif
      }
      break;

      // M551 - Prime extruder 1
      // P : number of steps
      // S : RPM * 10
      case 551:
      {
          config.status = 0;
#if 0
          if (next_target.seen_S && next_target.seen_P)
          {
              // calc E distance, use approximate conversion to get distance, not critical
              // TODO: how to derive magic number
              // S is RPM*10, but happens to give about the right speed in mm/min
              SpecialMoveE ((double)next_target.P / 256.0, next_target.S);
          }
#endif
      }
      break;

      // M600 print the values read from the config file
      case 600:
      {
          print_config();
      }
      break;

      case 601:
      {
         write_config();
      }
      break;

      //set home position absolute
      case 604:
      {
          if (next_target.seen_X)
          {
              config.home_pos_x = next_target.target.x;
              axisSelected = 1;
          }//no need for else

          if (next_target.seen_Y)
          {
              config.home_pos_y = next_target.target.y;
              axisSelected = 1;
          }//no need for else

          if (next_target.seen_Z)
          {
              config.home_pos_z = next_target.target.z;
              axisSelected = 1;
          }//no need for else

          if(!axisSelected)
          {
              config.home_pos_x = 0.0;
              config.home_pos_y = 0.0;
              config.home_pos_z = 0.0;
          }//no need for else
      }
      break;

      //set home position
      case 605:
      {
          if (next_target.seen_X){
              config.home_pos_x -= next_target.target.x;
              axisSelected = 1;
          }//no need for else

          if (next_target.seen_Y){
            config.home_pos_y -= next_target.target.y;
            axisSelected = 1;
          }//no need for else

          if (next_target.seen_Z){
              config.home_pos_z -= next_target.target.z;
              axisSelected = 1;
          }//no need for else

          if(!axisSelected){
              config.home_pos_x = 0.0;
              config.home_pos_y = 0.0;
              config.home_pos_z = 0.0;
          }//no need for else
      }
      break;

      // M606 - wait for empty movement queue
      case 606:
      {
          config.status = 0;

#if 0
          enqueue_wait();
#endif
      }
      break;

      case 607:
      {
          prepare_sector(29, 29, SystemCoreClock);
          erase_sector(29, 29, SystemCoreClock);

          read_config();
      }
      break;

      case 609:
      {
          go_to_reset(); // reinicia o sistema
      }
      break;

      // M610 - switch to USB bootloader
      case 610:
      {
          go_to_bootloader ();
      }
      break;

      // M620 - devolve a versão/nome da impressora
      case 620:
      {
          serial_writestr(" BEETHEFIRST ");
      }
      break;

      case 625:
      {
          serial_writestr(" S:");
          serwrite_uint32(config.status);

          if(config.status == 0)
             config.status = 3;

        }
        break;

        // unknown mcode: spit an error
        default:
        {
          config.status = 0;
          serial_writestr("ok - E: Bad M-code ");
          serwrite_uint8(next_target.M);
          serial_writestr("\r\n");
        }
    }
  }else{
      serial_writestr("E: Bad code ");

  }

  if (!reply_sent)
  {
    serial_writestr(" ok\r\n");
    //sersendf("ok Q:%d\r\n", plan_queue_size());
  }

  return result;
}
