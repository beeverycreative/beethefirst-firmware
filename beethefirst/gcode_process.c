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
bool      shutdown_pause = false;             // printing paused
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

double printed_filament = 0;

extern bool debugMode = false;              //Enable debug functions

//Pause at Z Vars
bool pauseAtZ = false;
double pauseAtZVal;

#ifdef USE_BATT
bool ps_ext_state = false;
bool batteryMode = false;
bool charging = false;
#endif
//Status Variables
char statusStr[32];
bool is_calibrating = false;
bool is_heating_Process = false;
bool is_heating_MCode = false;

//Calibrate Position
int32_t calibratePos = 0;

#ifdef EXP_Board
bool      manualBlockFanControl = false;        //manual control of fan using M126 and M127 M-Codes
int32_t   extruderFanSpeed = 0;
#endif

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

/****************************************************************************
 *                                                                          *
 * enqueue_wait_temp - Prepare Wait for temperature                         *
 *                                                                          *
 ****************************************************************************/
void enqueue_wait_temp (void)
{
  tActionRequest request;

  request.ActionType = AT_WAIT_TEMPS;
  plan_buffer_action (&request);
}

/****************************************************************************
 *                                                                          *
 * sd_initialise - Initialize SD Card                                       *
 *                                                                          *
 ****************************************************************************/

void sd_initialise(void)
{
  sd_active = true;
}

/****************************************************************************
 *                                                                          *
 * sd_list_dir - List SD Directories                                        *
 *                                                                          *
 ****************************************************************************/

void sd_list_dir (void)
{
  char path[line_length];

  strcpy (path, "");

  sd_list_dir_sub(path);
}

/****************************************************************************
 *                                                                          *
 * sd_open - Open SD File                                                   *
 *                                                                          *
 ****************************************************************************/

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

/****************************************************************************
 *                                                                          *
 * sd_close - Close SD File                                                 *
 *                                                                          *
 ****************************************************************************/

void sd_close(FIL *pFile)
{
  f_close (pFile);
}

/****************************************************************************
 *                                                                          *
 * sd_read_file - Read SD Fie Line                                          *
 *                                                                          *
 ****************************************************************************/

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

/****************************************************************************
 *                                                                          *
 * sd_write_to_file - Write to SD File                                      *
 *                                                                          *
 ****************************************************************************/

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

/****************************************************************************
 *                                                                          *
 * sd_filesize - get SD File Size                                           *
 *                                                                          *
 ****************************************************************************/

unsigned sd_filesize (FIL *pFile)
{
  return f_size(pFile);
}

/****************************************************************************
 *                                                                          *
 * SD Seek - Seek SD File Position                                          *
 *                                                                          *
 ****************************************************************************/

void sd_seek(FIL *pFile, unsigned pos)
{
  f_lseek (pFile, pos);
}

/****************************************************************************
 *                                                                          *
 * scan_files - Scan and list SD Card Files                                 *
 *                                                                          *
 ****************************************************************************/

static TCHAR lfname[_MAX_LFN];

FRESULT scan_files (char* path)
{

  DIR dirs;
  FRESULT res;
  BYTE i;
  char *fn;

  //sersendf("Reading file list");

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

/****************************************************************************
 *                                                                          *
 * sd_init - Initialize SD Card                                             *
 *                                                                          *
 ****************************************************************************/

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

/****************************************************************************
 *                                                                          *
 * print_file - Print Selected File                                         *
 *                                                                          *
 ****************************************************************************/

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
  printed_filament = 0.0;

  config.last_print_time = 0;
  write_config();

  if(is_heating_Process) is_heating_Process = false;

  config.status = 5;
#ifdef EXP_Board
  extruderFanSpeed = 0;
  manualBlockFanControl = false;
#endif
  debugMode = false;
  sd_printing = true;
  sd_pause = false;
  sd_resume = false;
  printerPause = false;
  printerShutdown = false;

  return true;
}

/****************************************************************************
 *                                                                          *
 * reinit_system - Re-Init System                                           *
 *                                                                          *
 ****************************************************************************/

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
 * write_config_override - Write Config Override                            *
 *                                                                          *
 ****************************************************************************/

bool write_config_override()
{

  __disable_irq();

  char fName[line_length];
  char currfName[line_length];
  char line[100];

  memset(currfName, '\0', sizeof(currfName));
  strcpy(currfName, config.filename);

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
      strcpy(fName, "OVER");
    }
  //opens as empty file
  if (sd_open(&file, fName, FA_CREATE_ALWAYS | FA_WRITE | FA_READ)) {
      if(!next_target.seen_B) {
          //sersendf("File created: %s\n",fName);
      }
      sd_pos = 0;
      //save current filename to config
      strcpy(config.filename, next_target.filename);
  }else{
      if(!next_target.seen_B){
          sersendf("error creating file: %s\n",fName);
      }
  }

  memset(line, '\0', sizeof(line));
  strcpy(line,"M1110 S1\n");
  sd_write_to_file(line,strlen(line));

  //Backup Z offset
  memset(line, '\0', sizeof(line));
  double num = config.home_pos_z;
  unsigned int intpart = (unsigned int)num;
  double dec = (num - intpart)*10000.0;
  unsigned int decpart = (unsigned int)dec;
  if(decpart < 1000.0F && decpart >= 100.0F) {
      sprintf(&line[0],"M604 Z%u.0%u\n",intpart,decpart);
  } else if(decpart < 100.0F && decpart >= 10.0F) {
      sprintf(&line[0],"M604 Z%u.00%u\n",intpart,decpart);
  } else if(decpart < 10.0F) {
      sprintf(&line[0],"M604 Z%u.000%u\n",intpart,decpart);
  } else {
      sprintf(&line[0],"M604 Z%u.%u\n",intpart,decpart);
  }

  sd_write_to_file(line,strlen(line));

  //Backup Nozzle Size
  memset(line, '\0', sizeof(line));
  sprintf(&line[0],"M1027 S%u\n",config.nozzleSize);
  sd_write_to_file(line,strlen(line));

  //Backup Filament Name
  if(strcmp(config.bcodeStr, "_no_file") != 0)
    {
      char bcodeStrBck[line_length];

      memset(line, '\0', sizeof(line));
      memset(bcodeStrBck, '\0', sizeof(bcodeStrBck));
      strcpy(bcodeStrBck,config.bcodeStr);
      sprintf(&line[0],"M1000 %s\n",bcodeStrBck);
      sd_write_to_file(line,strlen(line));
    }

  //Save to config
  memset(line, '\0', sizeof(line));
  sprintf(&line[0],"M601\n");
  sd_write_to_file(line,strlen(line));

  sd_close(&file);

  if(printerPause || sd_printing || sd_resume || sd_pause)
    {
      strcpy(config.filename,currfName);
      write_config();
    }

  __enable_irq();

  return true;

}

/****************************************************************************
 *                                                                          *
 * read_config_override - Read Config Override                            *
 *                                                                          *
 ****************************************************************************/

bool read_config_override()
{
  sersendf("Loading Config Override\n");
  sd_init();

  executed_lines = 0;
  //closes file
  sd_close(&file);
  char fileName[line_length];

  //opens a file
  if (sd_open(&file, "OVER", FA_READ)) {
      if(!next_target.seen_B) {
          //sersendf("File opened: %s\n",next_target.filename);
      }/*No need for else*/
      sd_pos = 0;
      //save current filename to config
      strcpy(config.filename, "OVER");
  }else{
      if(!next_target.seen_B){
          sersendf("error opening file: %s\n",next_target.filename);
          serial_writestr(next_target.filename);
          serial_writestr("\n");
      }/*No need for else*/
  }

  print_file();

  return true;
}
/****************************************************************************
 *                                                                          *
 * Command Received - process it                                            *
 *                                                                          *
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

  //Reset last comand timer and Leave Power Saving if needed
  if(next_target.seen_G
      || next_target.M == 28 // start transfer file
      || next_target.M == 33 || next_target.M == 34 || next_target.M == 104
      || next_target.M == 109 || next_target.M == 640 || next_target.M == 643 || next_target.M == 701
      || next_target.M == 702 || next_target.M == 703 || next_target.M == 704)
    {
      if(leave_power_saving)
        {
          reinit_system();
        }

      lastCmd_time = 0;
    }

  if (next_target.seen_G){

      switch (next_target.G)
      {
      // G1 - synchronised motion
      case 0:
      case 1:
        {
          if(!position_ok && !next_target.seen_B && !sd_printing && !debugMode){
              serial_writestr("position not ok ");
              break;
          }
          if(!sd_printing){
              config.status = 4;
          }else{
              config.status = 5;
          }

          double e_move_mm;
          if(relativeCoordinates) {
              e_move_mm = next_target.target.e;
          } else {
              e_move_mm = next_target.target.e - startpoint.e;
          }

          if(e_move_mm != 0)
            {
              if(sd_printing)
                {
                  printed_filament += e_move_mm;
                }
              //If not printing increase filament based on GCode
              if(!sd_printing)
                {
                  config.filament_in_spool -= e_move_mm;
                  write_config();
                }
            }

          enqueue_moved(&next_targetd);

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/

          if(pauseAtZ)
            {
              if(startpoint.z >= pauseAtZVal)
                {
                  initPause();
                  write_config();

                  sd_printing = false;
                  sd_pause = true;
                  sd_resume = false;
                }
            }
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
          if(is_heating_Process && !sd_printing)
            {
              is_heating_Process = false;
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
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //G91 - Set to Relative Positioning
      case 91:
        {
          relativeCoordinates = true;
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
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
          if(!sd_printing){
              config.status = 4;
          }else{
              config.status = 5;
          }
          home();
          config.acceleration = 400;
          GoTo5D(0,67,zCal,startpoint.e,15000);
          config.acceleration = 1000;
          calibratePos = 1;
          strcpy(statusStr, "Calibration");
          is_calibrating = true;
          config.status = 3;
          synch_queue();
          config.acceleration = 500;
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
              write_config_override();

              if(!sd_printing){
                  config.status = 4;
              }else{
                  config.status = 5;
              }

              config.acceleration = 400;
              GoTo5D(startpoint.x,startpoint.y,10,startpoint.e,15000);
              GoTo5D(-31,-65,10,startpoint.e,15000);
              GoTo5D(startpoint.x,startpoint.y,0,startpoint.e,1000);

              calibratePos = 2;
              config.status = 3;
              synch_queue();
              config.acceleration = 500;
            }
          else if(calibratePos == 2)
            {
              strcpy(statusStr, "Calibration");

              if(!sd_printing){
                  config.status = 4;
              }else{
                  config.status = 5;
              }

              config.acceleration = 400;
              GoTo5D(startpoint.x,startpoint.y,10,startpoint.e,15000);
              GoTo5D(31,-65,10,startpoint.e,15000);
              GoTo5D(startpoint.x,startpoint.y,0,startpoint.e,1000);

              calibratePos = 3;
              config.status = 3;
              synch_queue();
              config.acceleration = 500;
            }
          else if(calibratePos == 3)
            {

              if(!sd_printing){
                  config.status = 4;
              }else{
                  config.status = 5;
              }

              memset(statusStr, '\0', sizeof(statusStr));
              home();
              calibratePos = 0;
              config.status = 3;
              synch_queue();
              config.acceleration = 500;
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
          char fileName[line_length];

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
          char fName[line_length];
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
                  if(!sd_printing)
                    {
                      serial_writestr("error - not seen A\n");
                    }
                  reply_sent = 1;
              }/*No need for else*/
              break;
          }

          if(next_target.seen_L){
              number_of_lines = next_target.L;
          }else{
              if(!next_target.seen_B){
                  if(!sd_printing)
                    {
                      serial_writestr("error - not seen L\n");
                    }
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
              if(!next_target.seen_B)
                {
                  serial_writestr("A");
                  serwrite_uint32(estimated_time);
                  serial_writestr(" B");
                  __disable_irq();
                  serwrite_uint32(time_elapsed);
                  __enable_irq();

                  if(number_of_lines == 0)
                    {
                      serial_writestr(" C");
                      serwrite_uint32(file.fsize);
                      serial_writestr(" D");
                      serwrite_uint32(sd_pos);
                      serial_writestr(" ");

                      if(file.fsize == sd_pos){
                          serial_writestr("Done printing file\n");
                      }/*No need for else*/
                    }
                  else
                    {
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
          char fName[line_length];

          memset(fName, '\0', sizeof(fName));
          memset(statusStr, '\0', sizeof(statusStr));

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
          temp_set(0, EXTRUDER_0);
        }break;

      case 100:
	{/*
          if(next_target.seen_S){
              SDown_Threshold = next_target.S;
          }else{*/
              sersendf("Threshold:%u\n", SDown_Threshold);

	}
      break;

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
          temp_set(next_target.S, EXTRUDER_0);
          is_heating_MCode = true;
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
          config.status = 3;

          config.filament_in_spool -= printed_filament;
          write_config();

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

        //M128 - Extruder block regression adjust
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
              bool pidChange = false;

              if (next_target.seen_T){
               //   sersendf("kp=%g\n", next_target.T);
                  config.kp = next_target.T;
                  pidChange = true;
              }

              if (next_target.seen_U){
              //    sersendf("ki=%g\n", next_target.U*1000.0);
                  config.ki = next_target.U;

                  pidChange = true;
              }

              if (next_target.seen_V){
            //      sersendf("kd=%g\n", next_target.V);
                  config.kd = next_target.V;
                  pidChange = true;
              }

              if (pidChange)
                {
                  write_config();
                }

              if(!sd_printing)
                {
                  serial_writestr("kp: ");
                  serwrite_double(config.kp);

                  serial_writestr(" ki:");
                  serwrite_double(config.ki*1000.0);

                  serial_writestr(" kd:");
                  serwrite_double(config.kd);

                  serial_writestr(" ");
                }

              if(sd_printing){
                  reply_sent = 1;
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
                  config.steps_per_mm_e0 = next_targetd.e;
              }

              write_config();
          } else {
              if(!next_target.seen_B && !sd_printing){
                  sersendf ("X%g Y%g Z%g E%g ",
                      config.steps_per_mm_x,
                      config.steps_per_mm_y,
                      config.steps_per_mm_z,
                      config.steps_per_mm_e0);
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
                  if(next_target.target.x > 10000){
                      next_target.target.x = 10000;
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

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/

        }
        break;
        // M221 - Set flowrate factor override percentage
      case 221:
        {
          if(next_target.seen_S)
            {
        //    double currentFlowrate = filament_coeff;
              filament_coeff = (double) next_target.S / 100;
              //startpoint.f = startpoint.feed_rate * feedrate_coeff/currentFeedrate;
            }
          else {
              sersendf("flowrate coefficient: %g\n",(double)filament_coeff*100);
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
          if(in_power_saving)
            {
              break;
            }
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
              if(is_heating_Process || is_calibrating)
                {
                  home_z();
                }
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
              for(int i=0;i<line_length;i++){
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

          else if(next_target.seen_A){

              //enter power saving
              if (next_target.A == 1){
                  enter_power_saving = 1;
                  rest_time = 0;
              }
              //leave power saving
              if (next_target.A == 0){
                  enter_power_saving = 0;
                  reinit_system();
              }
          } else {
              sersendf("Invalid arguments");
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
#ifdef USE_BATT
          if(!ps_ext_state)
            {
              sersendf("Can't resume print without power supply\n");
            }
#endif

          if(pauseAtZ)
            {
              pauseAtZ = false;
            }

          queue_flush();
          reset_current_block();

          if(next_target.seen_W) {
              config.startpoint_filament_coeff= next_target.W;
          }
          if(next_target.seen_S) {
              temp_set(next_target.S, EXTRUDER_0);
          } else {
              temp_set(config.startpoint_temperature, EXTRUDER_0);
          }
          enqueue_wait_temp();

          is_heating_Process = false;
          memset(statusStr, '\0', sizeof(statusStr));

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
              if(!sd_printing){
                  config.status = 4;
              }else{
                  config.status = 5;
              }
              float loadFeedrate = 300;
              if(next_target.seen_S)
                {
                  loadFeedrate = next_target.S;
                }
              buzzer_wait ();
              buzzer_play (3000);
              Extrude(65,loadFeedrate);
              SetEPos(0);
              config.status = 3;
              config.filament_in_spool -= 65;
              write_config();
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
              if(!sd_printing){
                  config.status = 4;
              }else{
                  config.status = 5;
              }
              float loadFeedrate = 300;
              if(next_target.seen_S)
                {
                  loadFeedrate = next_target.S;
                }
              buzzer_wait ();
              buzzer_play (3000);
              Extrude(15,loadFeedrate);
              Extrude(-23,1000);
              Extrude(25,2*loadFeedrate);
              Extrude(-30,2000);
              Extrude(-50,loadFeedrate);
              SetEPos(0);
              config.status = 3;
              config.filament_in_spool -= 17;
              write_config();
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
              // Wait for all movements to end
              synch_queue();

              // Check if heating process is active
              if(is_heating_Process)
                {
                  // Check if the setpoint temperature was reached
                  if (current_temp[EXTRUDER_0] >= target_temp[EXTRUDER_0] - 5)
                    {
                      // Move bed to change filemant coordinates
                      double zDest = config.home_pos_z;
                      if(printerPause || printerShutdown) zDest = startpoint.z; // If there is a print do not move the z axis

                      // Backup current acceleration value
                      double acclerationStart = config.acceleration;

                      // Ensure low acceleration in this move
                      config.acceleration = 400;
                      GoTo5D(0,0,zDest,startpoint.e,15000);
                      if(!printerPause && !printerShutdown) config.acceleration = 1000;

                      // Wait for movement to stop
                      synch_queue();

                      // Write status load/unload string
                      strcpy(statusStr, "Load/Unload");

                      // Restore acceleration
                      config.acceleration = acclerationStart;

                    }
                  else break;
                }
              else
                {
                  // If no temperature is define, break
                  if(!next_target.seen_S) break;

                  // Start Heating process
                  is_heating_Process = true;
                  temp_set(next_target.S,EXTRUDER_0);
                  config.startpoint_temperature = next_target.S;
                  home();

                  // Move bed to heating coordinates
                  double zDest = 10.0;
                  if(printerPause || printerShutdown) zDest = startpoint.z; // If there is a print do not move the z axis

                  // Backup current acceleration value
                  double acclerationStart = config.acceleration;

                  // Ensure low acceleration in this move
                  config.acceleration = 400;
                  GoTo5D(0,0,zDest,startpoint.e,15000);
                  if(!printerPause && !printerShutdown) config.acceleration = 1000;

                  // Wait for movement to stop
                  synch_queue();

                  // Write heating status string
                  strcpy(statusStr, "Heating");

                  // Restore previous acceleration
                  config.acceleration = acclerationStart;

                }

              config.status = 3;

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
              config.status = 4;
              home();
              temp_set(0,EXTRUDER_0);
              disableBlower();
              is_heating_Process = false;
              config.status = 3;
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
              memset(config.bcodeStr, '\0', sizeof(config.bcodeStr));
              strcpy(config.bcodeStr,next_target.filename);
              memset(next_target.filename, '\0', sizeof(next_target.filename));
              write_config();
              write_config_override();
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
          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/
        }
        break;

        //Set ammount of filament in spool (mm)
      case 1024:
        {
          if(next_target.seen_X)
            {
              config.filament_in_spool = next_target.target.x;
              write_config();
            }
        }
        break;

        //Get ammount of filament in spool (mm)
      case 1025:
        {
          sersendf("Filament in Spool: %g\n",config.filament_in_spool);
        }
        break;

        //Send last print filament consumption (mm)
      case 1026:
        {
          sersendf("Last print filament: %g\n",config.last_print_filament);
        }
        break;

        //Set Nozzle Size (in Microns default:400)
      case 1027:
        {
          if(sd_printing)
            {
              reply_sent = 1;
            }
          else
            {
              if(next_target.seen_S)
                {
                  config.nozzleSize = next_target.S;
                  write_config();
                  write_config_override();
                }
            }
        }
        break;

        //Report Nozzle Size
      case 1028:
        {

          sersendf("Nozzle Size:%u\n",config.nozzleSize);

        }
        break;

        //Send Extruder log
      case 1029:
        {
#ifdef EXP_Board
          sersendf("T:%g/%g(%g%c) kp:%g ki:%g kd:%g pTerm:%g iTerm:%g dTerm:%g B:%g Vent:%u Bw:%u Z:%g ",\
              current_temp[0],target_temp[0],output,'%',\
              config.kp,config.ki*1000.0,config.kd,pterm,iterm,dterm,\
              current_temp[1],extruderFanSpeed,currenBWSpeed, startpoint.z);
#else
          sersendf("T:%g/%g(%g%c) kp:%g ki:%g kd:%g pTerm:%g iTerm:%g dTerm:%g B:%g Bw:%u Z:%g ",\
              current_temp[0],target_temp[0],output,'%',\
              config.kp,config.ki*1000.0,config.kd,pterm,iterm,dterm,\
              current_temp[1],currenBWSpeed, startpoint.z);
#endif
        }
        break;

        //Write Config-Override
      case 1030:
        {
          write_config_override();
        }
        break;

        //Read Config-Override
      case 1031:
        {
          read_config_override();
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

          voltsInput = (double) batt_filtered * (3.3 / (double) 4096) * 11;

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
        //M1110 - Toggle Debug Mode
      case 1110:
        {
          if(next_target.seen_S)
            {
              if(next_target.S == 1)
                {
                  debugMode = true;
                  if(!sd_printing) {
                      sersendf("Debug Mode Enabled\n");
                  }
                }
              else if(next_target.S == 0)
                {
                  debugMode = false;
                  if(!sd_printing) {
                      sersendf("Debug Mode Disabled\n");
                  }
                }
            } else {
                if(debugMode == false)
                  {
                    if(!sd_printing) {
                        sersendf("Debug Mode Disabled\n");
                    }
                  }
                else
                  {
                    if(!sd_printing) {
                        sersendf("Debug Mode Enabled\n");
                    }
                  }
            }

          if(sd_printing){
              reply_sent = 1;
          }/*No need for else*/


        }
        break;
        //M1111 - Define seconds to enter shutdown
      case 1111:
        {
          if(next_target.seen_S)
            {
              config.powerSavingWaitTime = next_target.S;
              write_config();
            }
          else
            {
              sersendf("Power saving time: %u seconds\n",config.powerSavingWaitTime);
              sersendf("Time since last useful command: %u milliseconds\n",lastCmd_time);
            }

        }
        break;
        //TODO M1200 - Replacement for M640 Command - Pause
        //TODO M1201 - Replacement for M643 Command - Resume
        //M1202- Pause ate Z
      case 1202:
        {
          if(next_target.seen_Z)
            {
              if(pauseAtZ == true)
                {
                  sersendf("Can't set pause at Z, waiting for pending pause\n");
                }
              else {
                  if(next_targetd.z <= startpoint.z)
                    {
                      pauseAtZ = false;
                      pauseAtZVal = 0;
                    }
                  else
                    {
                      pauseAtZ = true;
                      pauseAtZVal = next_targetd.z;
                    }
              }
            }
          else
            {
              if(pauseAtZ == true)
                {
                  sersendf("Pausing at Z: %g\n",pauseAtZVal);
                }
              else {
                  sersendf("No pause pending\n");
              }
            }

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
