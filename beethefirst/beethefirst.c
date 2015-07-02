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
#ifdef EXP_Board
  int32_t i_sDownADC_raw;
  int32_t sDownADC_raw[8];
  tTimer sDownTimer;
  int32_t sDown_filtered = 4095;

  tTimer blockFanTimer;
#endif

tLineBuffer serial_line_buf;
tLineBuffer sd_line_buf;

/* initialize PWM */
void pwm_init(void){

  pwm_pins_init(BUZZER_PORT,BUZZER_PIN_Number);           //Buzzer pwm
  pwm_pins_init(EXTRUDER_0_HEATER_PORT,EXTRUDER_0_HEATER_PIN_Number);
#ifdef EXP_Board
  pwm_pins_init(FAN_EXT_V1_PORT,FAN_EXT_V1_PIN);
  pwm_pins_init(BW_V1_PORT,BW_V1_PIN);
  pwm_pins_init(LOGO_ON_PORT,LOGO_ON_PIN);
#endif

  init_pwm_peripheral();

  init_global_match(BUZZER_PWM_CHANNEL);         //Buzzer
  init_global_match(EXTRUDER_0_PWM_CHANNEL);         //Heater
#ifdef EXP_Board
  init_global_match(FAN_EXT_PWM_CHANNEL);               //Extruder Block Fan
  init_global_match(LOGO_PWM_CHANNEL);                  //Logo
  init_global_match(BW_PWM_CHANNEL);                    //Blower
  init_global_match(HEATED_BED_0_PWM_CHANNEL);         //Bed

  //Turn Logo On
  pwm_set_duty_cycle(LOGO_PWM_CHANNEL,100);
  pwm_set_enable(LOGO_PWM_CHANNEL);

  //Turn Extruder Block Fan On at 100%
  extruder_block_fan_on();
  //pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,100);
  //pwm_set_enable(FAN_EXT_PWM_CHANNEL);
#endif

}

/* Initialize ADC for reading sensors */
void adc_init(void)
{
  PINSEL_CFG_Type PinCfg;

  //Extruder 0 ADC Config
  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = EXTRUDER_0_SENSOR_ADC_PORT;
  PinCfg.Pinnum = EXTRUDER_0_SENSOR_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

#ifdef EXP_Board
  //Extruder Block Temperature ADC Config
  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = HEATED_BED_0_ADC_PORT;
  PinCfg.Pinnum = HEATED_BED_0_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  //R2C2 TEMPERATURE ADC CONFIG
  PinCfg.Funcnum = PINSEL_FUNC_1; /*ADC Function*/
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = R2C2_TEMP_ADC_PORT;
  PinCfg.Pinnum = R2C2_TEMP_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  //Shutdown ADC CONFIG
  PinCfg.Funcnum = PINSEL_FUNC_3; /*ADC Function*/
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = SDOWN_ADC_PORT;
  PinCfg.Pinnum = SDOWN_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

#endif

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

  /* Heated Bed 0 Heater pin */
  pin_mode(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, OUTPUT);
  heated_bed_off();

#ifndef EXP_Board
  pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, OUTPUT);
  extruder_fan_off();
#endif

#ifdef EXP_Board
  pin_mode(FAN_EXT_ON_PORT, FAN_EXT_ON_PIN, OUTPUT);
  extruder_block_fan_on();

  pin_mode(ILUM_PORT,ILUM_PIN, OUTPUT);
  ilum_on();

  pin_mode(BW_ON_PORT,BW_ON_PIN, OUTPUT);
  blower_off();

  pin_mode(R2C2_FAN_PORT,R2C2_FAN_PIN, OUTPUT);
  r2c2_fan_on();
#endif
}

void temperatureTimerCallback (tTimer *pTimer)
{
  /* Manage the temperatures */
  temp_tick();
}

#ifdef EXP_Board
  void shutdownTimerCallBack (tTimer *pTimer)
  {
    int i, j;
    int32_t a;

    sDownADC_raw[i_sDownADC_raw] = analog_read(SDOWN_ADC_SENSOR_ADC_CHANNEL);
    i_sDownADC_raw ++;
    if(i_sDownADC_raw >= 5)
      {
        i_sDownADC_raw = 0;
      }

    //Sort the array
    int32_t sortedArray[5];

    for(i = 0; i < 5; i++)
      {
        sortedArray[i] = sDownADC_raw[i];
      }

    bubble_sort(sortedArray, 5);

    sDown_filtered = sortedArray[2];
    // filter the ADC values with simple IIR
    //sDown_filtered = ((sDown_filtered * 2) + sDownADC_raw) / 3;

    if(sDown_filtered < 1200)   //2700 -> 24V Shutdown threshold at ~10V
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
            config.startpoint_e    = currentE;

            if(currentF < 500)
              {
                currentF = 500;
              }

            config.startpoint_feed_rate = currentF;
            config.startpoint_temperature = target_temp[EXTRUDER_0];
            config.startpoint_filament_coeff = filament_coeff;
            config.blowerSpeed = currenBWSpeed;

            config.status = 9;
            sd_printing = 0;

            write_config();

            queue_flush();
            reset_current_block();

            zero_z();

        }
      }

  }

  void bubble_sort(int list[], int n)
  {
    int c, d, t;

    for (c = 0 ; c < ( n - 1 ); c++)
    {
      for (d = 0 ; d < n - c - 1; d++)
      {
        if (list[d] > list[d+1])
        {
          /* Swapping */

          t         = list[d];
          list[d]   = list[d+1];
          list[d+1] = t;
        }
      }
    }
  }

  void blockFanTimerCallBack(tTimer *pTimer) {

    if (manualBlockFanControl == false)
      {
        extruderFanSpeed = config.blockControlM * extruderBlockTemp + config.blockControlB;

        if(extruderFanSpeed > 100) {
            extruderFanSpeed = 100;
        }
        if(extruderFanSpeed < 0) {
            extruderFanSpeed = 0;
        }

        extruder_block_fan_on();
        pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,extruderFanSpeed);
        pwm_set_enable(FAN_EXT_PWM_CHANNEL);
      }

    if(current_temp_r2c2 < 40 && in_power_saving)
      {
        r2c2_fan_off();
      } else {
          r2c2_fan_on();
      }

  }
#endif

void init(void)
{
  // set up inputs and outputs
  io_init();
  //temperature read
  adc_init();
  //pwm
  pwm_init();

#ifdef DEBUG_UART
  uart_init();
#endif

  /* Initialize Gcode parse variables */
  gcode_parse_init();

  //temperature interruption
  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 10, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;

#ifdef EXP_Board
  //shutdown interruption
  sDownADC_raw[0] = analog_read(SDOWN_ADC_SENSOR_ADC_CHANNEL);
  sDownADC_raw[1] = analog_read(SDOWN_ADC_SENSOR_ADC_CHANNEL);
  sDownADC_raw[2] = analog_read(SDOWN_ADC_SENSOR_ADC_CHANNEL);
  sDownADC_raw[3] = analog_read(SDOWN_ADC_SENSOR_ADC_CHANNEL);
  sDownADC_raw[4] = analog_read(SDOWN_ADC_SENSOR_ADC_CHANNEL);
  i_sDownADC_raw = 0;

  AddSlowTimer(&sDownTimer);
  StartSlowTimer(&sDownTimer,25,shutdownTimerCallBack);
  sDownTimer.AutoReload = 1;

  //extruder block fan timer
  AddSlowTimer(&blockFanTimer);
  StartSlowTimer(&blockFanTimer,1000,blockFanTimerCallBack);
  blockFanTimer.AutoReload = 1;
#endif

#ifdef DEBUG_UART
  uart_writestr("R2C2 Firmware initiated\n");
#endif
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
  protection_temperature = 0;

  //debug bip
  bip = 2;
  bip_switch = 0;
  position_ok = 0;

  init();
  read_config();

  if (config.uid != CFG_UID) {
      reset_config();
  }

  if(config.status == 9)
    {
      printerShutdown = true;
    }

  // grbl init
  plan_init();
  st_init();

  WDT_Init (WDT_CLKSRC_PCLK, WDT_MODE_RESET );
  WDT_Start (30000000);

  buzzer_init();
  buzzer_play(1000); /* low beep */
  buzzer_wait();

  //print_infi();

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

#ifdef EXP_Board
      //Logo Blink
      if(start_logo_blink && (blink_time > blink_interval)) {

          if(logo_state) {
              pwm_set_duty_cycle(LOGO_PWM_CHANNEL,0);
              logo_state = 0;
          } else {
              pwm_set_duty_cycle(LOGO_PWM_CHANNEL,100);
              logo_state = 1;
          }

          pwm_set_enable(LOGO_PWM_CHANNEL);

          blink_time = 0;

      }
#endif

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

#ifdef EXP_Board
          blink_time = 0;
          start_logo_blink = 1;
          stop_logo_blink = 0;
          logo_state = 0;
          blink_interval = 1000;

          ilum_off();

          extruder_block_fan_on();
          pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,0);
          pwm_set_enable(FAN_EXT_PWM_CHANNEL);
          manualBlockFanControl = true;

#endif

          leave_power_saving = 1;
          enter_power_saving = 0;
          in_power_saving = TRUE;
      }/* No need for else */


      //if not executing movements
      //nor in a error state
      //nor recovering from shutdown
      //nor recovering from pause
      //nor printing form sd card
      //then is ready
      if((plan_queue_empty())
          && (config.status != 0)
          && (config.status != 7)
          && (config.status != 9)
          && (!sd_printing)){

          config.status = 3;
      }/*no need for else*/

      if ((plan_queue_empty())
          && (sd_pause)) {

          tTarget new_pos;
          /*
           * Little retraction to avoid filament break
           */
          st_synchronize();
          new_pos = startpoint;
          new_pos.e = 0;
          plan_set_current_position (&new_pos);
          st_synchronize();
          new_pos.e = -2;
          new_pos.feed_rate = 6000;
          enqueue_moved(&new_pos);
          st_synchronize();

          zero_z();
          zero_x();
          zero_y();

          //config.status = 7;
          sd_pause = false;
          printerPause = true;
      }/*no need for else*/

      if ((plan_queue_empty())
          && (sd_resume)) {
          disableSerialReply = true;

          char str[80];
          gcode_parse_str("G28\n");
          //sprintf("Resuming from X%g Y%g Z%g E%g F%g and SD pos: %d\n",config.startpoint_x, config.startpoint_y, config.startpoint_z, config.startpoint_e, config.startpoint_feed_rate, config.sd_pos);
          //uart_writestr(str);

          //Load & Seek SD File
          FRESULT res;
          res = sd_init();
          if(res != FR_OK){
              //uart_writestr("Error Seeking File");
              sersendf("error initializing sd card - %d\n",res);
              sd_resume = false;
          }

          if(res == FR_OK)
            {
              sd_close(&file);
              //uart_writestr("Opening File: %s\n",config.filename);
              sd_open(&file, config.filename, FA_READ);
              res = f_lseek(&file, config.sd_pos);
              sd_pos = config.sd_pos;
            }

          if(res != FR_OK){
              //uart_writestr("Error Seeking File");
              sersendf("error restarting print");
              sd_resume = false;
          } else {
              /*
               * Clean Nozzle
               */
              gcode_parse_str("G92 E\n");
              gcode_parse_str("G1 E20 F300\n");
              gcode_parse_str("G1 E18 F6000\n");
              gcode_parse_str("G1 E20 F500\n");

              if(config.blowerSpeed == 0)
                {
                  gcode_parse_str("M107\n");
                }
              else
                {
                  sprintf(str,"M106 S%d\n",config.blowerSpeed);
                }

              /*
               * Set E POS
               */
              sprintf(str,"G92 E%s\n",double2str(config.startpoint_e));
              gcode_parse_str(str);
              //uart_writestr(&str);
              /*
               * MOVE X Y To initial position
               */
              sprintf(str,"G0 X%s F10000\n",double2str(config.startpoint_x));
              gcode_parse_str(str);
              //uart_writestr(&str);
              sprintf(str,"G0 Y%s F10000\n",double2str(config.startpoint_y));
              gcode_parse_str(str);
              //uart_writestr(&str);
              /*
               * Set Filament Coeff.
               */
              //sprintf(str,"M642 W%s\n",double2str(config.startpoint_filament_coeff));
              //gcode_parse_str(str);
              //sersendf(&str);
              filament_coeff = config.startpoint_filament_coeff;
              /*
               * MOVE Z To initial position
               */
              sprintf(str,"G0 Z%s\n",double2str(config.startpoint_z));
              gcode_parse_str(str);
              //uart_writestr(&str);
              /*
               * Reduce feedrate to print speed
               */
              sprintf(str,"G0 F%s\n",double2str(config.startpoint_feed_rate));
              gcode_parse_str(str);
              //sersendf(&str);

              //uart_writestr("End of resume\n");

              time_elapsed = config.time_elapsed;
              executed_lines = config.executed_lines;

              sd_restartPrint = true;
              sd_resume = false;

          }
      }/*no need for else*/

      if(plan_queue_empty() && sd_restartPrint)
        {
          config.status = 5;
          sd_printing = true;
          sd_pause = false;
          sd_resume = false;
          sd_restartPrint = false;
          printerShutdown = false;
          printerPause = false;

          disableSerialReply = false;
        }


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
              if(print2USB)
                {
                  sersendf(&sd_line_buf);
                } else {
                    sd_line_buf.seen_lf = 1;
                }

              executed_lines++;
          }else{
              sd_printing = false;
              print2USB = false;
              sersendf(";EOF\n");
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
