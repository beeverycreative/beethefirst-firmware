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
tTimer shutdownTimer;

tLineBuffer serial_line_buf;
tLineBuffer sd_line_buf;

/* initialize PWM */
void pwm_init(void){

  pwm_pins_init(BUZZER_PORT,BUZZER_PIN_Number);           //Buzzer pwm

  pwm_pins_init(EXTRUDER_0_HEATER_PORT,EXTRUDER_0_HEATER_PIN_Number);

  pwm_pins_init(FAN_EXT_V1_PORT,FAN_EXT_V1_PIN);
  pwm_pins_init(BW_V1_PORT,BW_V1_PIN);
  pwm_pins_init(LOGO_ON_PORT,LOGO_ON_PIN);


  init_pwm_peripheral();

  init_global_match(BUZZER_PWM_CHANNEL);         //Buzzer
  init_global_match(FAN_EXT_PWM_CHANNEL);               //Extruder Block Fan
  init_global_match(LOGO_PWM_CHANNEL);                  //Logo
  init_global_match(BW_PWM_CHANNEL);                    //Blower
  init_global_match(EXTRUDER_0_PWM_CHANNEL);         //Heater
  init_global_match(HEATED_BED_0_PWM_CHANNEL);         //Bed

  //Turn Logo On
  pwm_set_duty_cycle(LOGO_PWM_CHANNEL,100);
  pwm_set_enable(LOGO_PWM_CHANNEL);

  //Turn Extruder Block Fan On at 100%
  extruder_block_fan_on();
  pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,100);
  pwm_set_enable(FAN_EXT_PWM_CHANNEL);
}

/* Initialize ADC for reading sensors */
void adc_init(void)
{
  PINSEL_CFG_Type PinCfg;

  //EXTRUDER 0 ADC CONFIG
  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = EXTRUDER_0_SENSOR_ADC_PORT;
  PinCfg.Pinnum = EXTRUDER_0_SENSOR_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  //HEADTED BED ADC CONFIG
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

  //R2C2 TEMPERATURE ADC CONFIG
  PinCfg.Funcnum = PINSEL_FUNC_3; /*ADC Function*/
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = SDOWN_ADC_PORT;
  PinCfg.Pinnum = SDOWN_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  ADC_Init(LPC_ADC, 200000); /* ADC conversion rate <= 200Khz */

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

  pin_mode(FAN_EXT_ON_PORT, FAN_EXT_ON_PIN, OUTPUT);
  extruder_block_fan_on();

  pin_mode(ILUM_PORT,ILUM_PIN, OUTPUT);
  ilum_on();

  pin_mode(BW_ON_PORT,BW_ON_PIN, OUTPUT);
  blower_off();

  pin_mode(R2C2_FAN_PORT,R2C2_FAN_PIN, OUTPUT);
  r2c2_fan_on();
}

void temperatureTimerCallback (tTimer *pTimer)
{
  /* Manage the temperatures */
  temp_tick();
}

void shutdownTimerCallBack (tTimer *pTimer)
{

  sDownADC_raw = analog_read(SDOWN_ADC_SENSOR_ADC_CHANNEL);
  // filter the ADC values with simple IIR
  sDown_filtered = ((sDown_filtered * 2) + sDownADC_raw) / 3;
  if(sDown_filtered < 2200)
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

          config.status = 9;
          sd_printing = 0;

          write_config();

          queue_flush();
          reset_current_block();

          zero_z();

      }
    }
}

void printRegisters(void)
{
  LPC_SC_TypeDef scReg;
  uart_writestr("PCONP: ");   uart_sendHex(scReg.PCONP); uart_writestr("\n");
  uart_writestr("PCON: ");   uart_sendHex(scReg.PCON); uart_writestr("\n");
  uart_writestr("PCLKSEL0: ");   uart_sendHex(scReg.PCLKSEL0); uart_writestr("\n");

  uart_writestr("\n");

  LPC_ADC_TypeDef adcReg;

  uart_writestr("ADCR: ");   uart_sendHex(adcReg.ADCR); uart_writestr("\n");

  uart_writestr("\n");

  LPC_PINCON_TypeDef pinDefs;

  uart_writestr("PINSEL0: ");   uart_sendHex(pinDefs.PINSEL0); uart_writestr("\n");
  uart_writestr("PINSEL1: ");   uart_sendHex(pinDefs.PINSEL1); uart_writestr("\n");
  uart_writestr("PINSEL2: ");   uart_sendHex(pinDefs.PINSEL2); uart_writestr("\n");
  uart_writestr("PINSEL3: ");   uart_sendHex(pinDefs.PINSEL3); uart_writestr("\n");
  uart_writestr("PINSEL4: ");   uart_sendHex(pinDefs.PINSEL4); uart_writestr("\n");
  uart_writestr("PINSEL5: ");   uart_sendHex(pinDefs.PINSEL5); uart_writestr("\n");
  uart_writestr("PINSEL6: ");   uart_sendHex(pinDefs.PINSEL6); uart_writestr("\n");
  uart_writestr("PINSEL7: ");   uart_sendHex(pinDefs.PINSEL7); uart_writestr("\n");
  uart_writestr("PINSEL8: ");   uart_sendHex(pinDefs.PINSEL8); uart_writestr("\n");
  uart_writestr("PINSEL9: ");   uart_sendHex(pinDefs.PINSEL9); uart_writestr("\n");
  uart_writestr("PINSEL10: ");   uart_sendHex(pinDefs.PINSEL10); uart_writestr("\n");

  uart_writestr("\n");

  uart_writestr("PINMODE0: ");   uart_sendHex(pinDefs.PINMODE0); uart_writestr("\n");
  uart_writestr("PINMODE1: ");   uart_sendHex(pinDefs.PINMODE1); uart_writestr("\n");
  uart_writestr("PINMODE2: ");   uart_sendHex(pinDefs.PINMODE2); uart_writestr("\n");
  uart_writestr("PINMODE3: ");   uart_sendHex(pinDefs.PINMODE3); uart_writestr("\n");
  uart_writestr("PINMODE4: ");   uart_sendHex(pinDefs.PINMODE4); uart_writestr("\n");
  uart_writestr("PINMODE5: ");   uart_sendHex(pinDefs.PINMODE5); uart_writestr("\n");
  uart_writestr("PINMODE6: ");   uart_sendHex(pinDefs.PINMODE6); uart_writestr("\n");
  uart_writestr("PINMODE7: ");   uart_sendHex(pinDefs.PINMODE7); uart_writestr("\n");
  uart_writestr("PINMODE8: ");   uart_sendHex(pinDefs.PINMODE8); uart_writestr("\n");
  uart_writestr("PINMODE9: ");   uart_sendHex(pinDefs.PINMODE9); uart_writestr("\n");

  uart_writestr("\n");

  uart_writestr("PINMODE_OD0: ");   uart_sendHex(pinDefs.PINMODE_OD0); uart_writestr("\n");
  uart_writestr("PINMODE_OD1: ");   uart_sendHex(pinDefs.PINMODE_OD1); uart_writestr("\n");
  uart_writestr("PINMODE_OD2: ");   uart_sendHex(pinDefs.PINMODE_OD2); uart_writestr("\n");
  uart_writestr("PINMODE_OD3: ");   uart_sendHex(pinDefs.PINMODE_OD3); uart_writestr("\n");
  uart_writestr("PINMODE_OD4: ");   uart_sendHex(pinDefs.PINMODE_OD4); uart_writestr("\n");

  uart_writestr("\n");

  uart_writestr("I2CPADCFG: ");   uart_sendHex(pinDefs.I2CPADCFG); uart_writestr("\n");

}

void init(void)
{
  // set up inputs and outputs
  io_init();

  uart_init();
  uart_writestr("BTF Start\n");

  //temperature read
  adc_init();

  //printRegisters();

  //pwm
  pwm_init();

  /* Initialize Gcode parse variables */
  gcode_parse_init();

  //Shutdown interruption
  //AddSlowTimer(&shutdownTimer);
  //StartSlowTimer(&shutdownTimer, 5, shutdownTimerCallBack);
  //shutdownTimer.AutoReload = 1;

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

  blink_time = 0;
  stop_fan_time = 0;

  last_target_e = 0;
  filament_coeff = 1;

  // Set initial protection_temperature
  protection_temperature = 180;

  //Set Block temperature setpoints
  blockTemperatureFanStart = 10;
  blockTemperatureFanMax = 48;
  blockFanMinSpeed = 12;
  blockFanMaxSpeed = 100;
  blockControlM = (blockFanMaxSpeed - blockFanMinSpeed)/(blockTemperatureFanMax - blockTemperatureFanStart);
  blockControlB = blockFanMaxSpeed - blockControlM*blockTemperatureFanMax;

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
  buzzer_play (2000);

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

      //Wait 4 min to turn off R2C2 fan
      if(start_r2c2_fan && (stop_fan_time > 60000)) {

          start_r2c2_fan = 0;
          stop_r2c2_fan = 1;

          r2c2_fan_off();
      }

      if(stop_r2c2_fan && (stop_fan_time > 300000)) {

          //RESET
          delay_ms(1000);
          USBHwConnect(FALSE);
          go_to_reset(); // reinicia o sistema
      }

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

          //Turn All Fans Off
          //extruder_fan_off();
          blower_off();
          pwm_set_duty_cycle(BW_PWM_CHANNEL,0);
          pwm_set_disable(BW_PWM_CHANNEL);
          pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,0);
          pwm_set_disable(FAN_EXT_PWM_CHANNEL);
          extruder_block_fan_off();
          r2c2_fan_off();
          manualBlockFanControl = true;

          //Turn

          leave_power_saving = 1;
          enter_power_saving = 0;
      }/* No need for else */

      /*
       * If not in manual operation mode
       * control the extruder block fan speed
       */
      if(!manualBlockFanControl && false) {
          extruderFanSpeed = (int32_t)blockControlM*extruderBlockTemp + blockControlB;
          if(extruderFanSpeed < 0) {
              extruderFanSpeed = 0;
          } else if(extruderFanSpeed > blockFanMaxSpeed) {
              extruderFanSpeed = blockFanMaxSpeed;
          }

          extruder_block_fan_on();

          pwm_set_duty_cycle(FAN_EXT_PWM_CHANNEL,extruderFanSpeed);
          pwm_set_enable(FAN_EXT_PWM_CHANNEL);
      }

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
