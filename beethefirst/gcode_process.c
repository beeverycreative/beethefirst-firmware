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

#include <string.h>
#include "gcode_parse.h"
#include "gcode_process.h"
#include "serial.h"
#include "sermsg.h"
#include "sersendf.h"
#include "temp.h"
#include "timer.h"
#include "pinout.h"
#include "config.h"
#include "ff.h"
#include "sd.h"
#include "buzzer.h"
//#include "debug.h"
#include "planner.h"
#include "stepper.h"
#include "geometry.h"
#include "bootloader.h"
#include "sbl_config.h"
#include "system_LPC17xx.h"
#include "lpc17xx_wdt.h"

#ifndef FW_V
  #define FW_V "0.0.0"
#endif


DWORD acc_size;                         /* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;
#if _USE_LFN
char Lfname[_MAX_LFN+1];
#endif

char Line[128];                         /* Console input buffer */
FIL       file;
FATFS Fatfs[_VOLUMES];          /* File system object for each logical drive */
uint32_t  filesize = 0;
uint32_t  sd_pos = 0;

bool      sd_printing = false;      // printing from SD file
bool      print2USB = false;      // printing from SD file to USB
bool      sd_pause = false;             // printing paused
bool      sd_resume = false;             // resume from sd pause
bool      in_power_saving = false;      //
bool      enter_power_saving = false;      // printing from SD file
bool      leave_power_saving = false;      // printing from SD file
bool      sd_active = false;        // SD card active
bool      sd_writing_file = false;  // writing to SD file

#ifdef EXP_Board
  bool      start_logo_blink = false;      // start logo blink
  bool      stop_logo_blink = true;      // stop logo blink
  bool      logo_state = false;           // logo state
  uint32_t  blink_interval = 10000;

  bool      manualBlockFanControl = false;        //manual control of fan using M126 and M127 M-Codes
  int32_t   extruderFanSpeed = 0;
#endif

double kp = 6.0;
double ki = 0.0013;
double kd = 80.0;

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

void enqueue_moved (tTarget *pTarget)
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

  next_targetd = startpoint;
  next_targetd.e = startpoint.e + e;
  next_targetd.feed_rate = feed_rate;
  enqueue_moved(&next_targetd);
}

void zero_x(void)
{
  int dir;
  int max_travel;

  if (HOME_DIR_X < 0)
    {
      dir = -1;
    }
  else
    {
      dir = 1;
    }
  max_travel = max (300, PRINT_VOL_X);

  // move to endstop
  SpecialMoveXY(startpoint.x + dir * max_travel, startpoint.y, HOME_FEED_X);
  synch_queue();

  // move forward a bit
  SpecialMoveXY(startpoint.x - dir * 3, startpoint.y, SEARCH_FEED_X);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x + dir * 6, startpoint.y, SEARCH_FEED_X);

  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;

  //R2C2: XICO B3.1 HOT FIX
  new_pos.x = HOME_POS_X;
  plan_set_current_position (&new_pos);
}

void zero_y(void)
{
  int dir;
  int max_travel;

  if (HOME_DIR_Y < 0)
    {
      dir = -1;
    }
  else
    {
      dir = 1;
    }
  max_travel = max (300, PRINT_VOL_Y);

  // move to endstop
  SpecialMoveXY(startpoint.x, startpoint.y + dir * max_travel, HOME_FEED_Y);
  synch_queue();

  // move forward a bit
  SpecialMoveXY(startpoint.x, startpoint.y - dir * 3, SEARCH_FEED_Y);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x, startpoint.y + dir * 6, SEARCH_FEED_Y);

  synch_queue();

  tTarget new_pos = startpoint;

  //R2C2: XICO B3.1 HOT FIX
  //new_pos.y = HOME_POS_Y;
  new_pos.y = -74.5;
  plan_set_current_position (&new_pos);
}

void zero_z(void)
{
  int dir;
  int max_travel;

  if (HOME_DIR_Z < 0)
    {
      dir = -1;
    }
  else
    {
      dir = 1;
    }
  max_travel = max (300, PRINT_VOL_Z);

  // move to endstop
  SpecialMoveZ(startpoint.z + dir * max_travel, HOME_FEED_Z);
  synch_queue();


  /**
   * Normal move Z
   **/

  tTarget next_targetd = startpoint;
  next_targetd.x = startpoint.x;
  next_targetd.y = startpoint.y;
  next_targetd.z = startpoint.z - dir * 10;
  next_targetd.e = startpoint.e;
  next_targetd.feed_rate =  HOME_FEED_Z;
  enqueue_moved(&next_targetd);
  synch_queue();
  /*
   * end
   */
   // move forward a bit
  //SpecialMoveZ(startpoint.z - dir * 1, config.search_feedrate_z);
  //synch_queue();

  // move back in to endstop slowly
  //SpecialMoveZ(startpoint.z + dir *15 , config.search_feedrate_z);
  SpecialMoveZ(startpoint.z + dir *15 , SEARCH_FEED_Z);
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

void sd_list_dir (void)
{
  char path[120];

  strcpy (path, "");

  sd_list_dir_sub(path);
}

unsigned sd_open(FIL *pFile, char *path, uint8_t flags){
  FRESULT res;

  res = f_open (pFile, path, flags);

  if (res == FR_OK){
      return 1;
  }else{
      if(!next_target.seen_B) {
          //debug
          sersendf (" %d - ", res);
          return 0;
      }/*no need for else*/
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
  //int res = 0;
  result = f_write (&file, pStr, bytes_to_write, &bytes_written);
  /*
  res = memcmp(&pStr,&bytes_written,bytes_to_write);

  if(res)
    serial_writestr("error comparing\n");
   */
  return result;
}

unsigned sd_filesize (FIL *pFile)
{
  return f_size(pFile);
}

void sd_seek(FIL *pFile, unsigned pos)
{
  f_lseek (pFile, pos);
}

FRESULT scan_files (char* path)
{
        DIR dirs;
        FRESULT res;
        BYTE i;
        char *fn;

        sersendf("Reading file list");

        if ((res = f_opendir(&dirs, path)) == FR_OK) {
                i = strlen(path);
                while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
                        if (_FS_RPATH && Finfo.fname[0] == '.') continue;
#if _USE_LFN
                        fn = *Finfo.lfname ? Finfo.lfname : Finfo.fname;
#else
                        fn = Finfo.fname;
#endif
                        if (Finfo.fattrib & AM_DIR) {
                                acc_dirs++;
                                *(path+i) = '/'; strcpy(path+i+1, fn);
                                res = scan_files(path);
                                *(path+i) = '\0';
                                if (res != FR_OK) break;
                        } else {
                                sersendf("%s/%s\n", path, fn);
                                acc_files++;
                                acc_size += Finfo.fsize;
                        }
                        //sersendf("%s/%s\n", path, fn);
                }
        }

        return res;
}

void sd_init()
{
  DSTATUS ds;

  ds = disk_initialize(0);
  if(ds != RES_OK) {
      sersendf("Error initializing disk - %d\n", ds);
      return;
  }
  sersendf("Card init OK.\n");
  sersendf("Card type: ");
  switch (CardType)
  {
  case CARDTYPE_MMC:
    sersendf("MMC\n");
    break;
  case CARDTYPE_SDV1:
    sersendf("Version 1.x Standard Capacity SD card.\n");
    break;
  case CARDTYPE_SDV2_SC:
    sersendf("Version 2.0 or later Standard Capacity SD card.\n");
    break;
  case CARDTYPE_SDV2_HC:
    sersendf("Version 2.0 or later High/eXtended Capacity SD card.\n");
    break;
  default:
    break;
  }

  sersendf("Sector size: %d bytes\n", CardConfig.sectorsize);
  sersendf("Sector count: %d\n", CardConfig.sectorcnt);
  sersendf("Block size: %d sectors\n", CardConfig.blocksize);
  sersendf("Card capacity: %d MByte\n\n", (((CardConfig.sectorcnt >> 10) * CardConfig.sectorsize)) >> 10);

  FRESULT fsRes;

  fsRes = f_mount(&Fatfs[0],"",1);
  //sersendf("FsRes: %d",fsRes);
  if(fsRes == FR_OK) {
      sersendf("Disk mounted\n");
  } else {
      sersendf("Error Mounting FatFs - %d\n", ds);
      return;
  }

}

void print_infi(void) {
  /*
     * AUTOMATIC START PRINTING FILE NAMED INFI
     */

    sd_printing = false;
    sd_init();

    executed_lines = 0;
    //closes file
    sd_close(&file);

    char path [120] = "INFI";

    //strcpy(next_target.filename, path);

    //opens a file
    if (sd_open(&file, "INFI", FA_READ)) {
        if(!next_target.seen_B) {
            sersendf("File opened\n");
        }/*No need for else*/
        sd_pos = 0;
        //save current filename to config
        strcpy(config.filename, next_target.filename);
    }else{
        if(!next_target.seen_B){
            sersendf("error opening file\n");
            serial_writestr(next_target.filename);
            serial_writestr("\n");
        }/*No need for else*/
    }

    FRESULT res;
    res = f_lseek(&file, sd_pos);

    if(res != FR_OK){
        if(!next_target.seen_B){
            serial_writestr("error seeking position on file\n");
        }/*No need for else*/
        //break;
    }/*No need for else*/

    config.status = 5;
    sd_printing = true;
}

void reinit_system(){
  leave_power_saving = 0;
  in_power_saving = FALSE;

#ifdef EXP_Board
  r2c2_fan_on();

  extruder_fan_on();
  manualBlockFanControl = false;

  blink_time = 0;
  start_logo_blink = 0;
  stop_logo_blink = 1;
  logo_state = 1;
  pwm_set_duty_cycle(LOGO_PWM_CHANNEL,100);
  pwm_set_enable(LOGO_PWM_CHANNEL);
  ilum_on();

#endif

  x_enable();
  y_enable();
  z_enable();
  e_enable();

  zero_x();
  zero_y();
  zero_z();

  while(!(plan_queue_empty())){
      continue;
  }
}

/****************************************************************************
 *                                                                          *
 * Command Received - process it                                             *
 *                                                                           *
 ****************************************************************************/

eParseResult process_gcode_command(){
  double backup_f;
  uint8_t axisSelected = 0;
  eParseResult result = PR_OK;
  bool reply_sent = false;

  tTarget next_targetd = startpoint;

  // convert relative to absolute
  if (next_target.option_relative){
      next_targetd.x = startpoint.x + next_target.target.x;
      next_targetd.y = startpoint.y + next_target.target.y;
      next_targetd.z = startpoint.z + next_target.target.z;
      next_targetd.e = startpoint.e + next_target.target.e;
      if (next_target.seen_F){
          next_targetd.feed_rate = next_target.target.feed_rate;
      }/*No need for else*/
  }else{
      // absolute
      if (next_target.seen_X){
          next_targetd.x = next_target.target.x;
      }/*No need for else*/

      if (next_target.seen_Y){
          next_targetd.y = next_target.target.y;
      }/*No need for else*/

      if (next_target.seen_Z){
          next_targetd.z = next_target.target.z;
      }/*No need for else*/

      if (next_target.seen_E){
          if(get_temp(EXTRUDER_0) < protection_temperature){
              if(!next_target.seen_B && !sd_printing){
                  serial_writestr("temperature too low ");
              }/* No need for else */
          }else{
              if(sd_printing && (filament_coeff != 1)){
                  // in the case of a filament change
                  next_targetd.e = startpoint.e + filament_coeff*(next_target.target.e - last_target_e);

              }else{
                  next_targetd.e = next_target.target.e;
              }

              //save this value
              last_target_e = next_target.target.e;
          }
      }/*No need for else*/

      if (next_target.seen_F){
          next_targetd.feed_rate = next_target.target.feed_rate;
      }/*No need for else*/
  }

  if(leave_power_saving){
      if(next_target.seen_M){
          if((next_target.M == 625 || next_target.M == 637)){
                //do nothing
          }else{
              reinit_system();
          }
      }else{
          reinit_system();
      }
  }/*No need for else*/

  if (next_target.seen_G){

      switch (next_target.G)
      {
      // G1 - synchronised motion
      case 0:
      case 1:
        {
          if(!position_ok && !next_target.seen_B && !sd_printing){
              serial_writestr("position not ok ");
              break;
          }
          if(!sd_printing){
              config.status = 4;
          }else{
              config.status = 5;
          }

          enqueue_moved(&next_targetd);

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //G28 - go home
      case 28:
        {
          next_targetd.feed_rate = HOME_FEED_Z;
          double aux = config.acceleration;
          config.acceleration = 1000;
          if(!sd_printing){
              config.status = 4;
          }else{
              config.status = 5;
          }
          if (next_target.seen_X){
              zero_x();
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_Y){
              zero_y();
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_Z){
              zero_z();
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_E){
              zero_e();
              axisSelected = 1;
          }/*No need for else*/

          if(!axisSelected){
              /*
              if (config.machine_model == MM_RAPMAN){
                  // move stage down to clear Z endstop
                  // Rapman only?
                  next_targetd = startpoint;
                  next_targetd.z += 3;
                  enqueue_moved(&next_targetd);
              }*/

              zero_z();
              zero_x();
              zero_y();
              zero_e();
          }/*No need for else*/

          config.acceleration = aux ;
          position_ok = 1;
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //G92 - set current position
      case 92:
        {

          tTarget new_pos;

          // must have no moves pending if changing position
          synch_queue();

          new_pos = startpoint;

          if (next_target.seen_X){
              new_pos.x = next_target.target.x;
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_Y){
              new_pos.y = next_target.target.y;
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_Z){
              new_pos.z = next_target.target.z;
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_E){
              new_pos.e = next_target.target.e;
              axisSelected = 1;
          }/*No need for else*/

          if(!axisSelected){
              new_pos.x = 0;
              new_pos.y = 0;
              new_pos.z = 0;
              new_pos.e = 0;
          }/*No need for else*/

          plan_set_current_position (&new_pos);

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // unknown gcode: spit an error
      default:
        {
          config.status = 0;
          if(!next_target.seen_B && !sd_printing){
              serial_writestr("ok - Error: Bad G-code ");
              serwrite_uint8(next_target.G);
              if(next_target.seen_N){
                  serial_writestr(" N:");
                  serwrite_uint32(next_target.N);
              }/*No need for else*/
              serial_writestr("\r\n");
          }/*No need for else*/
          reply_sent = 1;
        }
      }
  }else if (next_target.seen_M){

      switch (next_target.M)
      {
#ifdef EXP_Board
      // M5 Turn THE LIGHTS ON
      case 5:
        {
          ilum_on();
        }
        break;
        // M6 Turn THE LIGHTS OFF
      case 6:
        {
          ilum_off();
        }
        break;
#endif
      // SD File functions
      case 20: // M20 - list SD Card files
        {
          sersendf("Begin file list\n");

          // list files in root folder
          //sd_list_dir();
          scan_files("");
          sersendf("End file list\r\n");
          /*
          if(!next_target.seen_B){
              serial_writestr("Begin file list\n");

              // list files in root folder
              //sd_list_dir();
              scan_files("");
              serial_writestr("End file list\r\n");
          }*/
          /*No need for else*/
        }
        break;

      case 21: // M21 - init SD card
        {
          sd_printing = false;
          sd_init();

          //TODO - Config is corrupted after initiating the SD Card File System, read_config to reload.
          read_config();

        }
        break;

        //M23 Make a new file with name filename
      case 23:
        {
          executed_lines = 0;
          //closes file
          sd_close(&file);

          //opens a file
          if (sd_open(&file, next_target.filename, FA_READ)) {
              if(!next_target.seen_B) {
                  sersendf("File opened: %s\n",next_target.filename);
              }/*No need for else*/
              sd_pos = 0;
              //save current filename to config
              strcpy(config.filename, next_target.filename);
              /*
              while (p1) {
                  if ((UINT)p1 >= blen) {
                      cnt = blen; p1 -= blen;
                  } else {
                      cnt = p1; p1 = 0;
                  }
                  res = f_read(&File1, Buff, cnt, &s2);
                  if (res != FR_OK) { put_rc(res); break; }
                  p2 += s2;
                  if (cnt != s2) break;
              }
              */
          }else{
              if(!next_target.seen_B){
                  sersendf("error opening file: %s\n",next_target.filename);
                  serial_writestr(next_target.filename);
                  serial_writestr("\n");
              }/*No need for else*/
          }
        }
        break;

      case 25: //M25 - Pause SD print
        {
          if(sd_printing)
            {
              sd_printing = false;
              config.status = 3;
            }/*No need for else*/
        }
        break;

      case 26: //M26 - Set SD file pos
        {
          if(next_target.seen_S){
              sd_pos = next_target.S;  // 16 bit
              sd_seek(&file, sd_pos);
          }/*No need for else*/
        }
        break;

        //M28 - transfer size and begin if valid
      case 28:
        {
          if(!next_target.seen_A){
              if(!next_target.seen_B){
                  serial_writestr("error - not seen A\n");
                  reply_sent = 1;
              }/*No need for else*/
              break;
          }/*No need for else*/

          if(!next_target.seen_D){
              if(!next_target.seen_B){
                  serial_writestr("error - not seen D\n");
                  reply_sent = 1;
              }/*No need for else*/
              break;
          }/*No need for else*/

          bytes_to_transfer = next_target.D - next_target.A + 1;

          if ((bytes_to_transfer < 1) || (next_target.A > next_target.D)){
              if(!next_target.seen_B){
                  serial_writestr("error - invalid number of bytes to transfer\n");
                  reply_sent = 1;
              }/*No need for else*/
              break;
          }/*No need for else*/

          if (bytes_to_transfer > 0){
              if(!next_target.seen_B){
                  serial_writestr("will write ");
                  serwrite_uint32((bytes_to_transfer));
                  serial_writestr(" bytes ");
              }/*No need for else*/
          }/*No need for else*/

          FRESULT res;
          res = f_lseek(&file, next_target.A);

          if(res != FR_OK){
              if(!next_target.seen_B){
                  serial_writestr("error seeking position on file\n");
                  reply_sent = 1;
              }/*No need for else*/
              break;
          }/*No need for else*/

          number_of_bytes = 0;
          transfer_mode = 1;

          //status = transfering
          config.status = 6;
        }
        break;

        //M29 Delete SD File
      case 29:
        {
          FRESULT res;
          //f_open(&file,next_target.filename,FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
          res = f_unlink(next_target.filename);
          sersendf("Deleted File: %s - Res: %d\n",next_target.filename,res);

        }
        break;

        //M30 <filename>
      case 30:
        {
          //closes file
          sd_close(&file);

          //opens as empty file
          if (sd_open(&file, next_target.filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ)) {
              if(!next_target.seen_B) {
                  sersendf("File created\n");
              }/*No need for else*/
              sd_pos = 0;
              //save current filename to config
              strcpy(config.filename, next_target.filename);
          }else{
              if(!next_target.seen_B){
                  sersendf("error creating file\n");
                  serial_writestr(next_target.filename);
                  serial_writestr("\n");
              }/*No need for else*/
          }
          executed_lines = 0;
        }
        break;

      case 31: //M31 -variables from software
        {
          if(next_target.seen_A){
              estimated_time = next_target.A;
          }else{
              if(!next_target.seen_B){
                  serial_writestr("error - not seen A\n");
                  reply_sent = 1;
              }/*No need for else*/
              break;
          }

          if(next_target.seen_L){
              number_of_lines = next_target.L;
          }else{
              if(!next_target.seen_B){
                  serial_writestr("error - not seen L\n");
                  reply_sent = 1;
              }/*No need for else*/
              break;
          }
        }
        break;

      case 32: //M32 - variables to software
        {
          if(next_target.seen_A){
              __disable_irq();
              time_elapsed = next_target.A;
              __enable_irq();

          }else{
              if(!next_target.seen_B){

                  serial_writestr("A");
                  serwrite_uint32(estimated_time);
                  serial_writestr(" B");

                  __disable_irq();
                  serwrite_uint32(time_elapsed);
                  __enable_irq();

                  serial_writestr(" C");
                  serwrite_uint32(number_of_lines);
                  serial_writestr(" D");
                  serwrite_uint32(executed_lines);
                  serial_writestr(" ");

                  if(number_of_lines == executed_lines){
                      serial_writestr("Done printing file\n");
                  }/*No need for else*/
              }/*No need for else*/
          }
        }
        break;

      case 33: //M33 - Start SD print
        {

          FRESULT res;
          res = f_lseek(&file, sd_pos);
          sersendf("Starting print\n");
          if(res != FR_OK){
              if(!next_target.seen_B){
                  serial_writestr("error seeking position on file\n");
              }/*No need for else*/
              break;
          }/*No need for else*/

          config.status = 5;
          sd_printing = true;
          sd_pause = false;
          sd_resume = false;
        }
        break;

      case 34: //M34 - Print file to USB
        {

          sd_printing = true;
          print2USB = true;

        }
        break;

        //resume print from shutdown, get vars from config
      case 35:
        {
          //get all the vars from config
          strcpy(next_target.filename, config.filename);
          sd_pos              = config.sd_pos;
          estimated_time      = config.estimated_time;
          time_elapsed        = config.time_elapsed;
          number_of_lines     = config.number_of_lines;
          executed_lines      = config.executed_lines;

          sd_close(&file);
          //opens a file
          if (sd_open(&file, next_target.filename, FA_READ)) {
              if(!next_target.seen_B) {
                  sersendf("File opened\n");
              }/*No need for else*/
          }else{
              if(!next_target.seen_B){
                  sersendf("error opening file\n");
                  serial_writestr(next_target.filename);
                  serial_writestr("\n");
              }/*No need for else*/
          }
        } break;

        //Prepare to shutdown save vars to config
        //Only use during prints, after M640 - which prepares the config
      case 36: //M36
        {
          //set status to shutdown
          config.status = 9;

          write_config();
        }break;

        // M104- set temperature
      case 104:
        {
          double maxTemp = 250;
#ifdef EXP_Board
          maxTemp = 300;
#endif
          if(next_target.S > maxTemp){
              temp_set(maxTemp, EXTRUDER_0);
          }else{
              temp_set(next_target.S, EXTRUDER_0);
          }

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // M105- get temperature
      case 105:
        {
          if(!next_target.seen_B && !sd_printing){
              temp_print();
          }/*No need for else*/

          if(sd_printing){
              temp_print();
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // M106- fan on
      case 106:
        {
#ifndef EXP_Board
          extruder_fan_on();
#endif
#ifdef EXP_Board
          if(next_target.seen_S){
              blower_on();

              uint16_t s_val = next_target.S;
              uint16_t duty = 0;
              if(s_val >= 255)
                {
                  duty = 100;
                } else {
                    duty = (uint16_t) s_val*0.4;
                }

              pwm_set_duty_cycle(BW_PWM_CHANNEL,duty);
              pwm_set_enable(BW_PWM_CHANNEL);
          } else {
              blower_on();
              pwm_set_duty_cycle(BW_PWM_CHANNEL,100);
              pwm_set_enable(BW_PWM_CHANNEL);
          }
#endif
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // M107- fan off
      case 107:
        {
#ifndef EXP_Board
          extruder_fan_off();
#endif
#ifdef EXP_Board
          blower_off();
          pwm_set_duty_cycle(BW_PWM_CHANNEL,0);
          pwm_set_disable(BW_PWM_CHANNEL);
#endif
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // M109- set temp and wait
      case 109:
        {
          if(!sd_printing){
              config.status = 4;
          }else{
              config.status = 5;
          }

          temp_set(next_target.S, EXTRUDER_0);
          enqueue_wait_temp();

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // M112- immediate stop
      case 112:
        {
          config.status = 0;

          queue_flush();
          reset_current_block();

          sd_printing = false;
          sd_pause = false;
          sd_resume = false;
        }
        break;

        // M115 - report firmware version
      case 115:
        {
          if(!next_target.seen_B && !sd_printing){
              //serial_writestr(" 0.0.0");
              serial_writestr(FW_V);
              serial_writestr(" ");
          }
        }
        break;

      case 117:
        {
          //try to read the unique ID - not working in this LPC Revision
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

          if(!next_target.seen_B){
              serial_writeblock(serialnumber, 10);
              serial_writestr(" ");
          }/*No need for else*/
        }
        break;

        /*M121- report XYZE to host */
      case 121:
        {
          if (next_target.option_inches){
              config.status = 0;
          }else{
              if(!next_target.seen_B && !sd_printing){
                  sersendf(" C: X:%g Y:%g Z:%g E:%g ", startpoint.x, startpoint.y, startpoint.z, startpoint.e);
              }/*No need for else*/
          }
        }
        break;

        /*M122- report XYZE from config to host  */
      case 122:
        {
          if (next_target.option_inches){
              config.status = 0;
          }else{
              if(!next_target.seen_B && !sd_printing){
                  sersendf(" C: X:%g Y:%g Z:%g E:%g ", config.startpoint_x, config.startpoint_y, config.startpoint_z, config.startpoint_e);
              }/*No need for else*/
          }
        }
        break;

#ifdef EXP_Board
        // M126- Control Extruder fan on
      case 126:
        {
          //Control extruder fan
          if(next_target.seen_S){
              manualBlockFanControl = true;
              extruder_block_fan_on();

              uint16_t s_val = next_target.S;
              if(s_val >= 255)
                {
                  extruderFanSpeed = 100;
                } else {
                    extruderFanSpeed = (uint16_t) s_val*0.4;
                }

              pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,extruderFanSpeed);
              pwm_set_enable(FAN_EXT_PWM_CHANNEL);
          } else {
              manualBlockFanControl = false;
              extruder_block_fan_on();
              pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,100);
              pwm_set_enable(FAN_EXT_PWM_CHANNEL);
          }/*No need for else*/

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // M127- Extruder block fan off
      case 127:
        {
          manualBlockFanControl = true;
          pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,0);
          pwm_set_disable(FAN_EXT_PWM_CHANNEL);
          extruder_block_fan_off();

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

       //M128 - Excruder block regression adjust
      case 128:
        {

          if(next_target.seen_A) {
              config.blockTemperatureFanStart = next_target.A;
          }
          if(next_target.seen_D) {
              config.blockTemperatureFanMax = next_target.D;
          }
          if(next_target.seen_L) {
              config.blockFanMinSpeed = next_target.L;
          }
          if(next_target.seen_P) {
              config.blockFanMaxSpeed = next_target.P;
          }

          if(next_target.seen_A || next_target.seen_D || next_target.seen_L || next_target.seen_P)
            {
              //Extruder block fan regression
              config.blockControlM = (config.blockFanMaxSpeed - config.blockFanMinSpeed)/(config.blockTemperatureFanMax - config.blockTemperatureFanStart);
              config.blockControlB = config.blockFanMaxSpeed - config.blockControlM*config.blockTemperatureFanMax;
            }

          sersendf(" S: %u M: %g B: %g state: %u ", extruderFanSpeed, config.blockControlM, config.blockControlB, manualBlockFanControl);
        }
        break;

#endif
        // M130 temperature PID
      case 130:
        {
          if(!next_target.seen_B ){
              if ((next_target.seen_T | next_target.seen_U | next_target.seen_V) == 0){
                  serial_writestr("kp: ");
                  //serwrite_double(config.kp);
                  serwrite_double(kp);
                  serial_writestr(" ki:");
                  //serwrite_double(config.ki);
                  serwrite_double(ki);
                  serial_writestr(" kd:");
                  //serwrite_double(config.kd);
                  serwrite_double(kd);
                  serial_writestr(" ");
              }/*No need for else*/
          }/*No need for else*/
        }
        break;

      case 131:
        {
          print_pwm();
        }
        break;

#ifdef EXP_Board
        // M134 Turn R2C2 fAN ON
      case 134:
        {
          r2c2_fan_on();
        }
        break;
        // M135 Turn R2C2 fAN ON
      case 135:
        {
          r2c2_fan_off();
        }
        break;
        // M136 - start logo blink
      case 136:
        {
          blink_time = 0;
          start_logo_blink = 1;
          stop_logo_blink = 0;
          logo_state = 0;

          if(next_target.seen_S) {
              blink_interval = next_target.S;
          } else {
              blink_interval = 5000;
          }

          /* PWM Control*/
          /*if(next_target.seen_S ){
                              //logo_on();
                              pwm_set_duty_cycle(LOGO_PWM_CHANNEL,next_target.S);
                              pwm_set_enable(LOGO_PWM_CHANNEL);
                          } else {
                              //logo_on();
                              pwm_set_duty_cycle(LOGO_PWM_CHANNEL,100);
                              pwm_set_enable(LOGO_PWM_CHANNEL);
                          }
           */
        }
        break;

        // M137 - turn logo off
      case 137:
        {
          blink_time = 0;
          start_logo_blink = 0;
          stop_logo_blink = 1;
          logo_state = 1;
          pwm_set_duty_cycle(LOGO_PWM_CHANNEL,100);
          pwm_set_enable(LOGO_PWM_CHANNEL);
          ilum_on();
          /*
                          pwm_set_duty_cycle(LOGO_PWM_CHANNEL,0);
                          pwm_set_enable(LOGO_PWM_CHANNEL);
                          //pwm_set_disable(LOGO_PWM_CHANNEL);
           */

        }
        break;
#endif

        // M200 - set steps per mm
      case 200:
        {
          if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E)){
              /*
              if(next_target.seen_X) {
                  config.steps_per_mm_x = next_targetd.x;
              } else if(next_target.seen_Y) {
                  config.steps_per_mm_y = next_targetd.y;
              } else if(next_target.seen_Z) {
                  config.steps_per_mm_z = next_targetd.z;
              } else if(next_target.seen_E) {
                  config.steps_per_mm_e = next_targetd.e;
              }

              write_config();
              */
              sersendf("Option Disabled");
          } else {
              if(!next_target.seen_B && !sd_printing){
                  sersendf ("X%g Y%g Z%g E%g ",
                      STEPS_MM_X,
                      STEPS_MM_Y,
                      STEPS_MM_Z,
                      STEPS_MM_E0);
              }/*No need for else*/
          }

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // M206 - set accel in mm/sec^2
      case 206:
        {
          if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0){
              if(!next_target.seen_B && !sd_printing){
                  sersendf (" X%g ",
                      config.acceleration);
              }/*No need for else*/
          }else{
              if (next_target.seen_X){
                  if(next_target.target.x > 2000){
                      next_target.target.x = 2000;
                  }
                  config.acceleration = next_target.target.x;

              }/*No need for else*/
          }

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // M300 - beep
        // S: frequency
        // P: duration
      case 300:
        {
          uint16_t duration = 1000; // 1 second

          if (next_target.seen_P && next_target.P<11000){
              duration = next_target.P;
          }/*No need for else*/

          buzzer_wait ();
          buzzer_play (duration);

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

      // M302 set safe temp
      case 302:
      {
        if (next_target.seen_S){
            protection_temperature=next_target.S;
        }else{
            protection_temperature=0;
        }
      }
      break;

        // M400 bcode
      case 400:
        {
          if(next_target.seen_A){
              config.bcode= next_target.A;
          }else{
              serial_writestr(" bcode:A");
              serwrite_int32(config.bcode);
              serial_writestr(" ");
          }
        }
        break;

#ifdef EXP_Board
        //M504 - Report Power input voltage
      case 504:
        {
          double voltsInput;

          voltsInput = (double) sDown_filtered/112.5;

          sersendf("Raw input: %u Volts: %g\n", sDown_filtered,voltsInput);
        }
        break;

        //M505 - Clear Shutdown flag
      case 505:
        {
          config.status = 3;
          write_config();
        }
        break;
#endif
        //M506 - Load config
      case 506:
        {
          read_config();
        }
        break;

        // M600 print the values read from the config file
      case 600:
        {
          if(!next_target.seen_B){
              print_config();
          }/*No need for else*/
        }
        break;

      case 601:
        {
          write_config();
        }
        break;

        //calibrate
      case 603:
        {
          tTarget new_pos;

          //recalculate distante from home
          config.home_pos_z -= startpoint.z;

          //my position is zero
          new_pos = startpoint;
          new_pos.z = 0;
          plan_set_current_position (&new_pos);

        }
        break;

        //set home position absolute
      case 604:
        {
          /*
          if (next_target.seen_X){
              HOME_POS_X = next_target.target.x;
              axisSelected = 1;
          }//no need for else

          if (next_target.seen_Y){
              HOME_POS_Y = next_target.target.y;
              axisSelected = 1;
          }//no need for else
          */
          if (next_target.seen_Z){
              config.home_pos_z = next_target.target.z;
              axisSelected = 1;
          }//no need for else
          /*
          if(!axisSelected) {
              HOME_POS_X = 0.0;
              HOME_POS_Y = 0.0;
              HOME_POS_Z = 0.0;
          }//no need for else
          */
          if(sd_printing){
              reply_sent = 1;
          }
        }
        break;

        //set home position
      case 605:
        {
          /*
          if (next_target.seen_X){
              HOME_POS_X -= next_target.target.x;
              axisSelected = 1;
          }//no need for else

          if (next_target.seen_Y){
              HOME_POS_Y -= next_target.target.y;
              axisSelected = 1;
          }//no need for else
          */
          if (next_target.seen_Z){
              config.home_pos_z -= next_target.target.z;
              axisSelected = 1;
          }//no need for else
          /*
          if(!axisSelected){
              HOME_POS_X = 0.0;
              HOME_POS_Y = 0.0;
              HOME_POS_Z = 0.0;
          }//no need for else
          */
          sersendf("Option Disabled");

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        /* RESET CONFIG*/
      case 607:
        {
          reset_config();
        }
        break;

        /* M609 - RESTART IN BOOTLOADER MODE*/
      case 609:
        {
          if(!sd_printing){
              delay_ms(1000);
              USBHwConnect(FALSE);
              go_to_reset(); // reinicia o sistema
          }else{
              sersendf("Can't Reset Printer While Printing");
          }
        }
        break;

      case 625:
        {
          if(!next_target.seen_B){
              serial_writestr(" S:");
              serwrite_int32(config.status);
              serial_writestr(" ");
              if(config.status == 0){
                  if(!sd_printing){
                      config.status = 3;
                  }else{
                      config.status = 5;
                  }
              }/*No need for else*/
          }/*No need for else*/
        }
        break;

      case 636:
        {
          if(bip_switch == 0){
              bip_switch = 1;
          }else{
              bip_switch = 0;
          }
        }
        break;

      case 637:
        {
          reply_sent = 1;
        }
        break;

      case 638:
        {
          if(!next_target.seen_B){
              serial_writestr("last N:");
              serwrite_uint32(next_target.N);
              serial_writestr(" sdpos:");
              serwrite_uint32(executed_lines);
              serial_writestr(" ");
          }/*No need for else*/
        }
        break;

      case 639:
        {
          if(!next_target.seen_B ){
              for(int i=0;i<120;i++){
                  if(next_target.filename[i]){
                      serial_writechar(next_target.filename[i]);
                  }else{
                      break;
                  }
              }
              serial_writestr(" ");
          }/*No need for else*/
        }
        break;


        //stop sd_printing and copy state to config
      case 640:
        {
          if(sd_printing){

              //save vars
              config.sd_pos          = sd_pos;
              config.estimated_time  = estimated_time;
              config.time_elapsed    = time_elapsed;
              config.number_of_lines = number_of_lines;
              config.executed_lines  = executed_lines;
              config.startpoint_x    = startpoint.x;
              config.startpoint_y    = startpoint.y;
              config.startpoint_z    = startpoint.z;
              config.startpoint_e    = startpoint.e;
              config.startpoint_feed_rate = startpoint.feed_rate;
              config.startpoint_temperature = target_temp[EXTRUDER_0];
              config.startpoint_filament_coeff = filament_coeff;

              write_config();

              //config.status = 7;
              sd_printing = false;
              sd_pause = true;
              sd_resume = false;
          }/* No need for else */
        }
        break;

      case 641:
        {
          if(next_target.seen_A){

              if (next_target.A == 1){
                  enter_power_saving = 1;
                  rest_time = 0;
              }
              if (next_target.A == 0){
                  enter_power_saving = 0;
              }
          } else {
              enter_power_saving = 0;
          }

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

      case 642:
        {
          if(next_target.seen_W){
              filament_coeff = next_target.W;
          }else{
              serial_writestr("filament coefficient:");
              serwrite_double(filament_coeff);
              serial_writestr(" ");

          }
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;
        //Resume SD Print from pause
      case 643:
        {
          if(next_target.seen_W) {
              filament_coeff = next_target.W;
          }
          if(next_target.seen_S) {
              temp_set(next_target.S, EXTRUDER_0);
          } else {
              temp_set(config.startpoint_temperature, EXTRUDER_0);
          }
          enqueue_wait_temp();

          sd_resume = true;
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        // unknown mcode: spit an error
      default:
        {
          config.status = 0;
          if(!next_target.seen_B && !sd_printing){
              serial_writestr("ok - E: Bad M-code ");
              serwrite_uint8(next_target.M);
              if(next_target.seen_N){
                  serial_writestr(" N:");
                  serwrite_uint32(next_target.N);
                  //next_target.N = 0;

              }/*No need for else*/
              serial_writestr("\r\n");
          }
          reply_sent = 1;
        }
      }
  }else if(next_target.seen_semi_comment || next_target.seen_parens_comment){
      reply_sent = 1;
  }else{
      if(!next_target.seen_B && !sd_printing){
          serial_writestr("E: Bad code ");
      }/*No need for else*/
      if(sd_printing){
          reply_sent = 1;
      }/*No need for else*/
  }

  if (!reply_sent){
      if(!next_target.seen_B){
          serial_writestr("ok Q:");
          serwrite_uint32(plan_queue_size());
          if(next_target.seen_N){
              serial_writestr(" N:");
              serwrite_uint32(next_target.N);
              //next_target.N = 0;
          }/*No need for else*/
          serial_writestr("\r\n");
      }/*No need for else*/
  }/*No need for else*/

  if(next_target.seen_M && enter_power_saving){
      if(!(next_target.M == 625 || next_target.M == 637)){
          rest_time = 0;

      }/*No need for else*/
  }/*No need for else*/

  return result;
}
