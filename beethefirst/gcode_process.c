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

//Relative Coordinates Option
bool relativeCoordinates = false;

//For Pause Functions
double currentE;
double currentF;

char Line[128];                         /* Console input buffer */
FIL       file;
FATFS Fatfs[_VOLUMES];          /* File system object for each logical drive */
uint32_t  filesize = 0;
uint32_t  sd_pos = 0;

bool      sd_printing = false;      // printing from SD file
bool      print2USB = false;      // printing from SD file to USB
bool      sd_pause = false;             // printing paused
bool      sd_resume = false;             // resume from sd pause
bool      printerShutdown = false;             // printer in shutdown
bool      printerPause = false;             // printer in pause
bool      sDownAfterPause = false;              // wait for printer to pause and enter shutdown
bool      sd_restartPrint = false;
bool      disableSerialReply = false;
bool      in_power_saving = false;      //
bool      enter_power_saving = false;      // printing from SD file
bool      leave_power_saving = false;      // printing from SD file
bool      sd_active = false;        // SD card active
bool      sd_writing_file = false;  // writing to SD file

#ifdef USE_BATT
bool ps_ext_state = false;
bool batteryMode = false;
bool charging = false;
#endif
//Status Variables
char statusStr[32];
bool is_calibrating = false;
bool is_heating = false;

//Calibrate Position
int32_t calibratePos = 0;

#ifdef EXP_Board
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

void enqueue_wait_temp (void)
{
  tActionRequest request;

  request.ActionType = AT_WAIT_TEMPS;
  plan_buffer_action (&request);
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

static TCHAR lfname[_MAX_LFN];

FRESULT scan_files (char* path)
{
  DIR dirs;
  FRESULT res;
  BYTE i;
  char *fn;

  if ((res = f_opendir(&dirs, path)) == FR_OK) {

      for (;; ) {
          Finfo.lfname = lfname;
          Finfo.lfsize = _MAX_LFN - 1;
          /* Read a directory item */
          res = f_readdir(&dirs, &Finfo);
          if (res || !Finfo.fname[0]) {
              break;                                  /* Error or end of dir */
          }
          if (Finfo.fattrib & AM_DIR) {

          }
          else {
              sersendf("/%s\r\n", Finfo.lfname[0] ? Finfo.lfname : Finfo.fname);
          }

      }
  }

  return res;
}

FRESULT sd_init()
{
  DSTATUS ds;

  ds = disk_initialize(0);
  if(ds != RES_OK) {
      sersendf("Error initializing disk - %d\n", ds);
      return;
  }
  FRESULT fsRes;

  fsRes = f_mount(&Fatfs[0],"",1);
  //sersendf("FsRes: %d",fsRes);
  if(fsRes == FR_OK) {
      //sersendf("Disk mounted\n");
  } else {
      sersendf("Error Mounting FatFs - %d\n", ds);
  }

  return fsRes;
}

bool print_file()
{
  FRESULT res;

  //Init SD Card
  res = sd_init();
  if(res != FR_OK)
    {
      sersendf("error mouting file system");
      return false;
    }

  //opens a file
  if (sd_open(&file, config.filename, FA_READ)) {
      if(!next_target.seen_B) {
          //sersendf("File opened: %s\n",next_target.filename);
      }/*No need for else*/
      sd_pos = 0;
      //save current filename to config
      //strcpy(config.filename, fName);
  }else{
      if(!next_target.seen_B){
          sersendf("error opening file: %s \n",config.filename);
      }/*No need for else*/
  }

  res = f_lseek(&file, sd_pos);
  //sersendf("Starting print\n");
  if(res != FR_OK){
      if(!next_target.seen_B){
          serial_writestr("error seeking position on file\n");
      }/*No need for else*/
      return false;
  }/*No need for else*/

  time_elapsed = 0;
  estimated_time = 0;
  number_of_lines = 0;
  executed_lines = 0;

  config.last_print_time = 0;
  write_config();

  config.status = 5;
  sd_printing = true;
  sd_pause = false;
  sd_resume = false;
  printerPause = false;
  printerShutdown = false;

  return true;
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

  home();

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
  //if (next_target.option_relative){
  if (relativeCoordinates){
      next_targetd.x = startpoint.x + next_target.target.x;
      next_targetd.y = startpoint.y + next_target.target.y;
      next_targetd.z = startpoint.z + next_target.target.z;
      next_targetd.e = startpoint.e + next_target.target.e;
      if (next_target.seen_F){
          next_targetd.feed_rate = next_target.target.feed_rate * feedrate_coeff;
          currentF = next_target.target.feed_rate * feedrate_coeff;
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
                  currentE = next_target.target.e;
                  // in the case of a filament change
                  next_targetd.e = startpoint.e + filament_coeff*(next_target.target.e - last_target_e);
              }else{
                  currentE = next_target.target.e;
                  next_targetd.e = next_target.target.e;
              }

              //save this value
              last_target_e = next_target.target.e;
          }
      }/*No need for else*/

      if (next_target.seen_F){
          next_targetd.feed_rate = next_target.target.feed_rate * feedrate_coeff;
          currentF = next_target.target.feed_rate * feedrate_coeff;
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
          relativeCoordinates = false;

          if(!sd_printing){
              config.status = 4;
          }else{
              config.status = 5;
          }
          if (next_target.seen_X){
              home_x();
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_Y){
              home_y();
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_Z){
              home_z();
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_E){
              home_e();
              axisSelected = 1;
          }/*No need for else*/

          if(!axisSelected){
              home();
          }/*No need for else*/


          //Cancel Existing Procedures & Clear Status String
          if(is_calibrating)
            {
              calibratePos = 0;
              is_calibrating = false;
            }
          if(is_heating && !sd_printing)
            {
              is_heating = false;
              temp_set(0, EXTRUDER_0);
            }
          memset(statusStr, '\0', sizeof(statusStr));

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //G90 - Set to Absolute Positioning
      case 90:
        {
          relativeCoordinates = false;
        }
        break;

        //G91 - Set to Relative Positioning
      case 91:
        {
          relativeCoordinates = true;
        }
        break;

        //G92 - set current position
      case 92:
        {
          if (next_target.seen_X){
              SetXPos(next_target.target.x);
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_Y){
              SetYPos(next_target.target.y);
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_Z){
              SetZPos(next_target.target.z);
              axisSelected = 1;
          }/*No need for else*/

          if (next_target.seen_E){
              SetEPos(next_target.target.e);
              axisSelected = 1;
          }/*No need for else*/

          if(!axisSelected){
              SetXPos(0);
              SetYPos(0);
              SetZPos(0);
              SetEPos(0);
          }/*No need for else*/
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //Start Calibration Procedure
      case 131:
        {
          char str[80];
          //memset(statusStr, '\0', sizeof(statusStr));
          if(sd_printing)
            {
              break;
            }

          double zCal = 2.0;
          //Verify if a specific Z position is aked before sending any command to the planner queue
          if(next_target.seen_Z)
            {
              zCal = next_targetd.z;
            }
          home();
          config.acceleration = 400;
          GoTo5D(0,67,zCal,startpoint.e,15000);
          config.acceleration = 1000;
          calibratePos = 1;
          strcpy(statusStr, "Calibration");
          is_calibrating = true;
        }
        break;
        //Prceed to next Calibration Procedure step
      case 132:
        {
          if(calibratePos == 1)
            {
              strcpy(statusStr, "Calibration");
              //recalculate distante from home
              config.home_pos_z -= startpoint.z;
              SetZPos(0);
              write_config();

              config.acceleration = 400;
              GoTo5D(startpoint.x,startpoint.y,10,startpoint.e,15000);
              GoTo5D(-31,-65,10,startpoint.e,15000);
              GoTo5D(startpoint.x,startpoint.y,0,startpoint.e,1000);

              calibratePos = 2;
            }
          else if(calibratePos == 2)
            {
              strcpy(statusStr, "Calibration");

              config.acceleration = 400;
              GoTo5D(startpoint.x,startpoint.y,10,startpoint.e,15000);
              GoTo5D(31,-65,10,startpoint.e,15000);
              GoTo5D(startpoint.x,startpoint.y,0,startpoint.e,1000);

              calibratePos = 3;
            }
          else if(calibratePos == 3)
            {
              disableSerialReply = true;
              memset(statusStr, '\0', sizeof(statusStr));
              home();
              disableSerialReply = false;
              calibratePos = 0;
            }
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

        }
        break;

        //M23 Make a new file with name filename
      case 23:
        {
          executed_lines = 0;
          //closes file
          sd_close(&file);
          char fileName[120];

          //opens a file
          if (sd_open(&file, next_target.filename, FA_READ)) {
              if(!next_target.seen_B) {
                  //sersendf("File opened: %s\n",next_target.filename);
              }/*No need for else*/
              sd_pos = 0;
              //save current filename to config
              strcpy(config.filename, next_target.filename);
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
          char fName[120];
          //closes file
          sd_close(&file);
          sd_init();

          memset(fName, '\0', sizeof(fName));

          if(strlen(next_target.filename) > 0)
            {
              strcpy(fName, next_target.filename);
            }
          else
            {
              strcpy(fName, "ABCDE");
            }

          //opens as empty file
          if (sd_open(&file, fName, FA_CREATE_ALWAYS | FA_WRITE | FA_READ)) {
              if(!next_target.seen_B) {
                  sersendf("File created: %s\n",fName);
              }/*No need for else*/
              sd_pos = 0;
              //save current filename to config
              strcpy(config.filename, next_target.filename);
              strcpy(statusStr, "Waiting4File");
          }else{
              if(!next_target.seen_B){
                  sersendf("error creating file: %s\n",fName);
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

                  if(estimated_time == 0 && number_of_lines == 0)
                    {
                      serial_writestr("A");
                      serwrite_uint32(estimated_time);
                      serial_writestr(" B");
                      __disable_irq();
                      serwrite_uint32(time_elapsed);
                      __enable_irq();

                      serial_writestr(" C");
                      serwrite_uint32(sd_pos);
                      serial_writestr(" D");
                      serwrite_uint32(file.fsize);
                      serial_writestr(" ");

                    } else {

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
                    }
              }/*No need for else*/
          }
        }
        break;

      case 33: //M33 - Start SD print
        {
          char fName[120];
          FRESULT res;

          memset(fName, '\0', sizeof(fName));

          if(strlen(next_target.filename) > 0)
            {
              strcpy(fName, next_target.filename);
            }
          else
            {
              strcpy(fName, "ABCDE");
            }

          strcpy(config.filename, fName);

          if(print_file() == false)
            {
              break;
            }

        }
        break;

      case 34: //M34 - Print file to USB
        {

          sd_printing = true;
          print2USB = true;

        }
        break;

        //Prepare to shutdown save vars to config
        //Only use during prints, after M640 - which prepares the config
      case 36: //M36
        {
          enterShutDown();
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
          enableBlower();
#endif
#ifdef EXP_Board
          if(next_target.seen_S){
              setBlowerSpeed((int16_t) next_target.S);
          } else {
              enableBlower();
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
          disableBlower();
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
          printerPause = false;
          printerShutdown = false;

          write_config();

          home();
          temp_set(0,EXTRUDER_0);
          disableBlower();
          enter_power_saving = 0;

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
              //if(!next_target.seen_B && !sd_printing){
              if(!next_target.seen_B){
                  sersendf(" C: X:%g Y:%g Z:%g E:%g F:%g ", startpoint.x, startpoint.y, startpoint.z, startpoint.e, startpoint.feed_rate);
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
          } else {
              if(!next_target.seen_B && !sd_printing){
                  sersendf ("X%g Y%g Z%g E%g ",
                      config.steps_per_mm_x,
                      config.steps_per_mm_y,
                      config.steps_per_mm_z,
                      config.steps_per_mm_e);
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

        // M220 - Set speed factor override percentage
      case 220:
        {
          if(next_target.seen_S)
            {
              double currentFeedrate = feedrate_coeff;
              feedrate_coeff = (double) next_target.S / 100;
              startpoint.feed_rate = startpoint.feed_rate * feedrate_coeff/currentFeedrate;
            }
          else {
              serial_writestr("feedrate coefficient:");
              serwrite_double((double)feedrate_coeff*100);
              serial_writestr(" ");
          }
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

#ifdef EXP_Board
        //M504 - Report Power input voltage
      case 504:
        {
          double voltsInput;

          voltsInput = (double) sDown_filtered/108.3;

          sersendf("Raw input: %u Volts: %g\n", sDown_filtered,voltsInput);
        }
        break;

#endif

        //M505 - Clear Shutdown flag
      case 505:
        {
          config.status = 3;
          printerShutdown = false;
          write_config();
        }
        break;

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
              if(printerPause)
                {
                  serial_writestr("Pause ");
                }
              if(printerShutdown)
                {
                  serial_writestr("Shutdown ");
                }
              if(in_power_saving)
                {
                  serial_writestr("Power_Saving ");
                }

              if(strlen(statusStr) > 0)
                {
                  serial_writestr("W:");
                  serial_writestr(statusStr);
                  serial_writestr(" ");
                }


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

        //Alive Command
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

        //echo
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
              memset(next_target.filename, '\0', sizeof(next_target.filename));
          }/*No need for else*/
        }
        break;


        //stop sd_printing and copy state to config
      case 640:
        {
          if(sd_printing)
            {
              initPause();
              write_config();

              sd_printing = false;
              sd_pause = true;
              sd_resume = false;
          }/* No need for else */
        }
        break;

      case 641:
        {
          if(next_target.seen_S)
            {
              powerSavingDelay = (uint32_t) next_target.S;
            }
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
              config.startpoint_filament_coeff= next_target.W;
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

        //Load Filament
      case 701:
        {
          if(!sd_printing)
            {
              buzzer_wait ();
              buzzer_play (3000);
              Extrude(100,300);
              SetEPos(0);
            }
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //Unload Filament
      case 702:
        {
          if(!sd_printing)
            {
              buzzer_wait ();
              buzzer_play (3000);
              Extrude(50,300);
              Extrude(-23,1000);
              Extrude(25,800);
              Extrude(-30,2000);
              Extrude(-50,200);
              SetEPos(0);
            }
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //Start Heating and got to heat position
      case 703:
        {
          if(!sd_printing)
            {
              if(next_target.seen_S && !is_heating)
                {
                  temp_set(next_target.S,EXTRUDER_0);
                  home();
                  config.acceleration = 400;
                  GoTo5D(-30,0,10,startpoint.e,15000);
                  config.acceleration = 1000;
                  strcpy(statusStr, "Heating");
                  is_heating = true;
                }
              else if(is_heating && current_temp[EXTRUDER_0] >= target_temp[EXTRUDER_0] - 5)
                {
                  GoTo5D(startpoint.x,startpoint.y,startpoint.z,startpoint.e,15000);
                  config.acceleration = 400;
                  GoTo5D(-50,0,110,startpoint.e,startpoint.feed_rate);
                  config.acceleration = 1000;
                  is_heating = false;
                  strcpy(statusStr, "Load/Unload");
                }
            }
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //Cancel Heating
      case 704:
        {
          if(!sd_printing)
            {
              memset(statusStr, '\0', sizeof(statusStr));
              home();
              temp_set(0,EXTRUDER_0);
              disableBlower();
              is_heating = false;
            }
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //Set Filament String
      case 1000:
        {

          if(strlen(next_target.filename) > 0)
            {
              strcpy(config.bcodeStr,next_target.filename);
              memset(next_target.filename, '\0', sizeof(next_target.filename));
              write_config();
            } else {
                sersendf("Error, Please Specify Filament String");
            }

        }
        break;

        //Read Filament String
      case 1001:
        {
          sersendf("'%s'\n",config.bcodeStr);
        }
        break;

        //Read Last Print Time
      case 1002:
        {
          sersendf("Last Print Time: %u\n",config.last_print_time);
        }
        break;
        //Print Last file
      case 1003:
        {
          synch_queue();
          print_file();
        }
        break;

#ifdef USE_BATT
        //Read PS Ext Input
      case 1100:
        {
          //ps_ext_state = digital_read(PS_EXT_READ_PORT,PS_EXT_READ_PIN);

          sersendf("Power Supply State: %u\n", ps_ext_state);

        }
        break;

        //Enable Battery Charge
      case 1101:
        {
          STEP_uC_enable();
        }
        break;

        //Disable Battery Charge
      case 1102:
        {
          STEP_uC_disable();
        }
        break;

        //Enable Battery
      case 1103:
        {
          BATT_uC_enable();
        }
        break;

        //Disable Battery
      case 1104:
        {
          BATT_uC_disable();
        }
        break;

        //M1105 - Report Battery input voltage
      case 1105:
        {
          double voltsInput;

          voltsInput = (double) batt_filtered/112.5;

          sersendf("Raw input: %u Volts: %g\n", batt_filtered,voltsInput);
        }
        break;
        //M1106 - Change Battery Print Time
      case 1106:
        {
          if(next_target.seen_S){
              config.batteryPrintTime = (uint32_t) next_target.S;
              write_config();
          }
        }
        break;
        //M1107 - Change Battery Stand By Time
      case 1107:
        {
          if(next_target.seen_S){
              config.standByTime = (uint32_t) next_target.S;
              write_config();
          }
        }
        break;
        //M1108 - Change Auto Resume Mode
      case 1108:
        {
          if(next_target.seen_S){
              config.autoResume = (uint32_t) next_target.S;
              write_config();
          }
        }
        break;

#endif
#ifdef EXP_Board
        //M1109 - Report r2c2 raw adc temperature
      case 1109:
        {
          sersendf("Raw input: %u\n", adc_filtered_r2c2);
        }
        break;

#endif
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
      if(!next_target.seen_B && !disableSerialReply){
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
