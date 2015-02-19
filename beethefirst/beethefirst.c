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

#include <stdint.h>
#include <stdlib.h>

#include "lpc17xx_timer.h"
#include "lpc17xx_wdt.h"
#include "lpc17xx_adc.h"
#include "r2c2.h"

#include "machine.h"
#include "gcode_parse.h"
#include "gcode_process.h"
#include "pinout.h"
#include "debug.h"
#include "config.h"
#include "temp.h"

#include "planner.h"
#include "stepper.h"
#include "sbl_config.h"
#include "pwm.h"

tTimer temperatureTimer;

tLineBuffer serial_line_buf;
tLineBuffer sd_line_buf;

/* initialize PWM */
void pwm_init(void){

  pwm_pins_init(2,2);
  pwm_pins_init(2,4);

  init_pwm_peripheral();

  init_global_match(3);
  init_global_match(5);

}

/* Initialize ADC for reading sensors */
void adc_init(void)
{
  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = EXTRUDER_0_SENSOR_ADC_PORT;
  PinCfg.Pinnum = EXTRUDER_0_SENSOR_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = HEATED_BED_0_ADC_PORT;
  PinCfg.Pinnum = HEATED_BED_0_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  ADC_Init(LPC_ADC, 200000); /* ADC conversion rate = 200Khz */
}

void io_init(void)
{
  /* setup I/O pins */
  pin_mode(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, OUTPUT);
  digital_write(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, 1); /* Disable reset for all stepper motors */

  pin_mode(X_STEP_PORT, X_STEP_PIN, OUTPUT);
  pin_mode(X_DIR_PORT, X_DIR_PIN, OUTPUT);
  pin_mode(X_ENABLE_PORT, X_ENABLE_PIN, OUTPUT);
  x_enable();
  pin_mode(X_MIN_PORT, X_MIN_PIN, INPUT);

  pin_mode(Y_STEP_PORT, Y_STEP_PIN, OUTPUT);
  pin_mode(Y_DIR_PORT, Y_DIR_PIN, OUTPUT);
  pin_mode(Y_ENABLE_PORT, Y_ENABLE_PIN, OUTPUT);
  y_enable();
  pin_mode(Y_MIN_PORT, Y_MIN_PIN, INPUT);

  pin_mode(Z_STEP_PORT, Z_STEP_PIN, OUTPUT);
  pin_mode(Z_DIR_PORT, Z_DIR_PIN, OUTPUT);
  pin_mode(Z_ENABLE_PORT, Z_ENABLE_PIN, OUTPUT);
  z_enable();
  pin_mode(Z_MIN_PORT, Z_MIN_PIN, INPUT);

  pin_mode(E_STEP_PORT, E_STEP_PIN, OUTPUT);
  pin_mode(E_DIR_PORT, E_DIR_PIN, OUTPUT);
  pin_mode(E_ENABLE_PORT, E_ENABLE_PIN, OUTPUT);
  e_enable();

  pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, OUTPUT);
  extruder_fan_off();
}

void temperatureTimerCallback (tTimer *pTimer)
{
  /* Manage the temperatures */
  temp_tick();
}


void init(void)
{
  // set up inputs and outputs
  io_init();
  //temperature read
  adc_init();
  //pwm
  pwm_init();

  /* Initialize Gcode parse variables */
  gcode_parse_init();

  //temperature interruption
  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 10, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;

}


void WDT_IRQHandler(void){
}

int app_main (void){
  long timer1 = 0;
  eParseResult parse_result;
  unsigned int bip;
  int temperature = 0;

  /*variables used to write the gcode in the sd card*/
  unsigned int counter = 0;
  unsigned char sector[SD_BUF_SIZE] = {0};
  unsigned int BytesWritten;
  FRESULT res;
  // set up pid default variables
  last_error = 0;
  dterm_temp = 0;
  pterm = 0;
  dterm = 0;
  iterm = 0;
  output = 0;
  PID_FUNTIONAL_RANGE = 80;
  estimated_time = 0;
  __disable_irq();
  time_elapsed = 0;
  __enable_irq();

  number_of_lines = 0;
  rest_time = 0;
  last_target_e = 0;
  filament_coeff = 1;

  // Set initial protection_temperature
  protection_temperature = 180;

  //debug bip
  bip = 2;
  bip_switch = 0;
  position_ok = 0;

  init();
  read_config();

  // grbl init
  plan_init();
  st_init();

  WDT_Init (WDT_CLKSRC_PCLK, WDT_MODE_RESET );
  WDT_Start (30000000);

  buzzer_init();
  buzzer_play(1000); /* low beep */
  buzzer_wait();

  // main loop
  for (;;){
      WDT_Feed();

      //bip a cada +-20s
      if(bip == 1){
          if(bip_switch){
              buzzer_play(200);
          }
      }else if(bip == 2000000){
          bip=0;
      }

      bip++;

      //Power saving check
      if(enter_power_saving && (rest_time > 30000) && !sd_printing){

          zero_z();

          while(!(plan_queue_empty())){
              continue;
          }
          //pin_mode(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, OUTPUT);
          //digital_write(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, 0); /* Disable reset for all stepper motors */
          x_disable();
          y_disable();
          z_disable();
          e_disable();

          temp_set(0, EXTRUDER_0);

          extruder_fan_off();

          leave_power_saving = 1;
          enter_power_saving = 0;
      }/* No need for else */


      //if not executing movements
      //nor in a error state
      //nor recovering from shutdown
      //nor printing form sd card
      //then is ready
      if((plan_queue_empty())
          && (config.status != 0)
          && (config.status != 9)
          && (!sd_printing)){

          config.status = 3;
      }/*no need for else*/


      if((plan_queue_empty())
          && (config.status != 0)
          && (sd_printing)){

          config.status = 5;
      }/*no need for else*/

      // process characters from the usb port
      while (!serial_line_buf.seen_lf
          && (serial_rxchars() != 0)){

          unsigned char c = serial_popchar();

          if (serial_line_buf.len < MAX_LINE){
              serial_line_buf.data [serial_line_buf.len] = c;
              serial_line_buf.len++;
          }/*no need for else*/

          if (((c==10) || (c==13))
              && (transfer_mode == 0)){

              if (serial_line_buf.len > 1)
                serial_line_buf.seen_lf = 1;
              else
                serial_line_buf.len = 0;

          }/*no need for else*/

          if (transfer_mode == 1){
              number_of_bytes = number_of_bytes + 1;
              if ((number_of_bytes == bytes_to_transfer)
                  ||((counter + serial_line_buf.len) == SD_BUF_SIZE)){

                  serial_line_buf.seen_lf = 1;
                  break;
              }/*no need for else*/
          }/*no need for else*/
      }

      // process SD file if no serial command pending
      if (!sd_line_buf.seen_lf
          && sd_printing
          && (plan_queue_size() < 10)){

          if (sd_read_file (&sd_line_buf)){
              sd_line_buf.seen_lf = 1;
              executed_lines++;
          }else{
              if(send_sd_footer)
                {
                  //END OF SD PRINT
                  temp_set(0,EXTRUDER_0);
                  zero_z();
                  zero_x();

                  tActionRequest request;
                  request.target = startpoint;
                  request.target.y = 65;
                  request.target.feed_rate = 1500;
                  request.ActionType = AT_MOVE;
                  request.target.invert_feed_rate =  false;
                  plan_buffer_action(&request);

                  // must have no moves pending if changing position
                  st_synchronize();
                  tTarget newTarget;
                  newTarget = startpoint;
                  newTarget.e = 0;
                  plan_set_current_position(&newTarget);

                  send_sd_footer = false;
                }

              sd_printing = false;
              filament_coeff = 1;
          }

      }/*no need for else*/

      // if queue is full, we wait
      if (!plan_queue_full()
          && !transfer_mode
          && (serial_line_buf.seen_lf
              || sd_line_buf.seen_lf) ){

          /* At end of each line, put the "GCode" on movebuffer.
           * If there are movement to do, Timer will start and execute code which
           * will take data from movebuffer and generate the required step pulses
           * for stepper motors.
           */

          // give priority to user commands
          if (serial_line_buf.seen_lf){

              parse_result = gcode_parse_line (&serial_line_buf);
              serial_line_buf.len = 0;
              serial_line_buf.seen_lf = 0;

          }else if (sd_line_buf.seen_lf){
              parse_result = gcode_parse_line (&sd_line_buf);
              sd_line_buf.len = 0;
              sd_line_buf.seen_lf = 0;
          }
      }/*no need for else*/


      if (transfer_mode
          && (serial_line_buf.len != 0)){

          /*used in the debug loop back*/
          //serial_writeblock(serial_line_buf.data,serial_line_buf.len);

          /*This should never occur!*/
          if (!((counter + serial_line_buf.len) <= SD_BUF_SIZE)){
              serial_writestr("error : sector overflow ");
              serwrite_uint32(counter + serial_line_buf.len);
              serial_writestr("\n");
          }/*no need for else*/

          /*the USB message is transfered to the array that is going to be stored*/
          for (int i = 0; i < serial_line_buf.len; i++){
              sector[counter+i] = serial_line_buf.data[i];
          }

          counter = counter + serial_line_buf.len;
          serial_line_buf.len = 0;
          serial_line_buf.seen_lf = 0;

          /*if the array to be written is full, it is write*/
          if (counter == SD_BUF_SIZE){
              /* writes to the file*/
              res = sd_write_to_file(sector, SD_BUF_SIZE);
              if(res != FR_OK) {
                  serwrite_uint32(res);
                  serial_writestr(" - error writing file\n");
              }else{
                  serial_writestr("tog\n");
              }
              counter = 0;

          }/*no need for else*/

          if (number_of_bytes == bytes_to_transfer){

              /*if the array to be written is full, it is write*/
              if (counter != 0){

                  /* writes to the file*/
                  res = sd_write_to_file(sector, counter);
                  if(res != FR_OK) {
                      serwrite_uint32(res);
                      serial_writestr(" - error writing file\n");
                  }else{
                      serial_writestr("tog\n");
                  }
              }/*no need for else*/


              f_sync(&file);

              bytes_to_transfer = 0;
              number_of_bytes = 0;
              transfer_mode = 0;
              counter = 0;
              config.status = 3;

          }/*no need for else*/

      }/*no need for else*/

  }
}
