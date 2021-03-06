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

#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

#include <stdbool.h>
#include "gcode_parse.h"
#include "ff.h"
#include "lights.h"
#include "fans.h"
#include "MovementController.h"
#include <stdio.h>

//Relative Coordinates Option
bool relativeCoordinates;

//For Pause Function
extern double currentE;
extern double currentF;

// for SD functions
extern FIL       file;
extern uint32_t  filesize;
extern uint32_t  sd_pos;
extern bool      restore_config_override;
extern bool      sd_printing;     // printing from SD file
extern bool      print2USB;      // printing from SD file to USB
extern bool      sd_pause;             // printing paused
extern bool      shutdown_pause;       //pause print from shutdown
extern bool      sd_resume;             // resume from sd pause
extern bool      printerShutdown;             // printer in shutdown
extern bool      printerPause;             // printer in pause
extern bool      sDownAfterPause;              // wait for printer to pause and enter shutdown
extern bool      sd_restartPrint;
extern bool      disableSerialReply;
extern bool      in_power_saving;      //
extern bool      enter_power_saving;      // printing from SD file
extern bool      leave_power_saving;
extern bool      sd_active;       // SD card active
extern bool      sd_writing_file; // writing to SD file
extern bool      is_heating_Process;
extern bool      is_calibrating;
extern bool      is_heating_MCode;

extern bool	     override_config;

#ifdef EXP_Board
  extern bool      manualBlockFanControl;        //manual control of fan using M132 and M133 M-Codes
  extern int32_t   extruderFanSpeed;
#endif

extern bool     debugMode;              //Enable debug functions

double printed_filament;

void enqueue_wait_temp (void);

FRESULT scan_files (char* path);
extern void sd_close (FIL *pFile);
extern FRESULT sd_init(void);
extern bool sd_write_to_file(char *pStr, unsigned bytes_to_write);

extern bool print_file();
extern bool write_config_override();
extern bool read_config_override();

extern bool sd_read_file (tLineBuffer *pLine);

// when we have a whole line, feed it to this
eParseResult process_gcode_command(void);

void reinit_system(void);

#endif	/* _GCODE_PROCESS_H */
