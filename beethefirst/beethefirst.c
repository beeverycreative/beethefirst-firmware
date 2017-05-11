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
//#include "debug.h"
#include "config.h"
#include "temp.h"
#include "pause.h"
#include "lights.h"
#include "fans.h"
#include "ExpBoard.h"

#include "planner.h"
#include "stepper.h"
#include "sbl_config.h"
#include "pwm.h"

tTimer temperatureTimer;

tLineBuffer serial_line_buf;
tLineBuffer sd_line_buf;

/* initialize PWM */
void pwm_init(void){

  pwm_pins_init(EXTRUDER_0_HEATER_PORT,EXTRUDER_0_HEATER_PIN_Number,PINSEL_FUNC_2);

  pwm_pins_init(HEATED_BED_0_HEATER_PORT,HEATED_BED_0_HEATER_PIN_Number,PINSEL_FUNC_1);

  init_pwm_peripheral();

  init_global_match(EXTRUDER_0_PWM_CHANNEL);         //Heater

  init_global_match(HEATED_BED_0_PWM_CHANNEL);
}

/* Initialize ADC for reading sensors */
void adc_init(void)
{
  PINSEL_CFG_Type PinCfg;

  //Extruder 0 ADC Config
  //PinCfg.Funcnum = PINSEL_FUNC_1; /* ADC function */
  //PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  //PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  //PinCfg.Portnum = EXTRUDER_0_SENSOR_ADC_PORT;
  //PinCfg.Pinnum = EXTRUDER_0_SENSOR_ADC_PIN;
  //PINSEL_ConfigPin(&PinCfg);

  //Heated Bed ADC Config
  PinCfg.Funcnum = PINSEL_FUNC_1; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = HEATED_BED_0_ADC_PORT;
  PinCfg.Pinnum = HEATED_BED_0_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  //Chamber ADC Config
  PinCfg.Funcnum = PINSEL_FUNC_1; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = CHAMBER_ADC_PORT;
  PinCfg.Pinnum = CHAMBER_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  ADC_Init(LPC_ADC, 200000); /* ADC conversion rate = 200Khz */
}

void io_init(void)
{
  /* setup I/O pins */

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

  pin_mode(DOOR_PORT, DOOR_PIN, INPUT);
  pin_mode(EncA_PORT, EncA_PIN, INPUT);
  pin_mode(EncB_PORT, EncB_PIN, INPUT);

  pin_mode(CHAMBER_LIGHT_PORT,CHAMBER_LIGHT_PIN, OUTPUT);
  digital_write(CHAMBER_LIGHT_PORT, CHAMBER_LIGHT_PIN, 1); /*Turn Chamber Light ON*/

  pin_mode(SPOOL_LIGHT_PORT,SPOOL_LIGHT_PIN, OUTPUT);
  digital_write(SPOOL_LIGHT_PORT, SPOOL_LIGHT_PIN, 1); /*Turn Spool Light ON*/

  //pin_mode(CHAMBER_HEATER_PORT, CHAMBER_HEATER_PIN, OUTPUT);
  //digital_write(CHAMBER_HEATER_PORT, CHAMBER_HEATER_PORT, 0);

  pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, OUTPUT);
  digital_write(EXTRUDER_0_FAN_PORT,EXTRUDER_0_FAN_PIN,1);
}

void temperatureTimerCallback (tTimer *pTimer)
{
  /* Manage the temperatures */
  temp_tick();
}

/*
 * In bigger NXP chips (such as LPC17xx) there are a couple of dedicated interrupt pins (EINTn) which have their own interrupt handler.
 * The rest of GPIOs have to use one common interrupt (EINT3). You can then poll the interrupt status register to see which pins have triggered the interrupt.
 */
void EINT3_IRQHandler(void)
{
  //Test if a rising edge interrupt happened in the Encoder A channel
  if ((LPC_GPIOINT->IO0IntStatR & (1 << 3)) & (1 << 3))
    {
      //raising edge interrupt on pin 0.3 was fired
      LPC_GPIOINT->IO0IntClr |= (1 << 3); // clear the status
      //do your task
      if(EncA())        //read pin state to eliminate spike reads
        {
          if(EncB())
            {
              encoderPos -= 1;
            }
          else
            {
              encoderPos += 1;
            }

        }
    }

  return;
}

void init(void)
{
  // set up inputs and outputs
  io_init();
  //pwm
  pwm_init();
  //temperature read
  adc_init();

  //Setup I2C
  i2c_init();

  //setup spi max31855
  SPI_MAX_init();

  //Setup TX0 and RX0 as inputs and its interrupts
  GPIO_IntCmd(0,1 << 2,0);      //enable P0.2 interrupt on rising edge
  //GPIO_IntCmd(0,1 << 2,1);      //enable P0.2 interrupt on falling edge
  //GPIO_ClearInt(0,1 << 2);
  GPIO_IntCmd(0,1 << 3,0);      //enable P0.3 interrupt on rising edge
  GPIO_ClearInt(0,1 << 3);

  NVIC_EnableIRQ(EINT3_IRQn);


#if defined(DEBUG_UART) && !defined(BTF_SMOOTHIE)
  uart_init();
#endif

  /* Initialize Gcode parse variables */
  gcode_parse_init();

  //temperature interruption
  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 50, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;

#ifdef DEBUG_UART
  uart_writestr("R2C2 Firmware initiated\n");
#endif
}


void WDT_IRQHandler(void){
}

int app_main (void){
  long timer1 = 0;
  eParseResult parse_result;
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
  lastCmd_time = 0;
  powerSavingDelay = 5000;
  last_target_e = 0;
  filament_coeff = 1;
  feedrate_coeff = 1;

  // Set initial protection_temperature
  protection_temperature = 0;

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

  temp_set(0.0,EXTRUDER_0);
  temp_set(0.0,HEATED_BED_0);

  //print_infi();

  // main loop
  for (;;){
      WDT_Feed();

      /***********************************************************************
       *
       *                     Logo Light Control
       *
       ***********************************************************************/
      if(debugMode == false)
        {
          LogoLightControl();
        }

      /***********************************************************************
       *
       *                        POWER SAVING
       *
       ***********************************************************************/
      //Verify if printer should enter in power saving


      //connectedUSB = digital_read(1,30);
      if(
          !sd_printing
          && !sd_restartPrint
          && !sd_pause
          && lastCmd_time > (config.powerSavingWaitTime * 1000 - powerSavingDelay)
          && !in_power_saving
          && !is_heating_Process
          && !is_heating_MCode
          && !is_calibrating
          && !enter_power_saving
          && !debugMode
          && !printerPause
          && !transfer_mode)
        {
          temp_set(0, EXTRUDER_0);
          temp_set(0, HEATED_BED_0);
          enter_power_saving = 1;
          rest_time = 0;
        }

      //Power saving check
      if(!transfer_mode && enter_power_saving && (rest_time > powerSavingDelay) && !sd_printing)
        {

          synch_queue();
          home_z();

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
          temp_set(0, HEATED_BED_0);

          leave_power_saving = 1;
          enter_power_saving = 0;
          in_power_saving = true;
        }/* No need for else */


      /***********************************************************************
       *
       *                HANDLE CONFIG.SATTUS
       *
       ***********************************************************************/
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

      /***********************************************************************
       *
       *                        PAUSE PRINT
       *
       ***********************************************************************/
      if ((plan_queue_empty())
          && (sd_pause)) {
          pausePrint();
          lastCmd_time = 0;

      }/*no need for else*/

      /***********************************************************************
       *
       *                        RESUME PRINT
       *
       ***********************************************************************/
      if ((plan_queue_empty())
          && (sd_resume)) {
          resumePrint();
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
      /***********************************************************************
       *
       *                PROCESS CHARACTERS FROM USB PORT
       *
       ***********************************************************************/
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

      /***********************************************************************
       *
       *                PROCESS SD FILE
       *
       ***********************************************************************/
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
              //sersendf(";EOF\n");
              config.last_print_time = time_elapsed;
              config.last_print_filament = printed_filament;
              write_config();
              filament_coeff = 1;
              feedrate_coeff = 1;
              is_cooling_Process = true;
          }

      }/*no need for else*/

      /***********************************************************************
       *
       *        PARSE LINES FROM USB OR SD
       *
       ***********************************************************************/
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

      /***********************************************************************
       *
       *       TRANSFER MODE - RECEIVING SD FILE
       *
       ***********************************************************************/
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

      /***********************************************************************
       *
       *                Check Encoder Position
       *
       ***********************************************************************/
      if(sd_printing && (lineStop > 0) && (lineNumber >= lineStop) && enableEncoderPause)
        {
          if(plannedSegment != 0)
            {
              GetEncoderPos();
              //sersendf("Segment: %g Encoder: %g\n",plannedSegment,encoderMM);
              extrusionError = (plannedSegment - encoderMM)/plannedSegment;
              if(extrusionError > 0.1 || extrusionError < -0.1)
                {
                  initPause();
                  write_config();

                  sd_printing = false;
                  sd_pause = true;
                  sd_resume = false;
                  filamentErrorPause = true;
                }

              config.filament_in_spool -= encoderMM;
              write_config();

              plannedSegment = 0;
              encoderPos = 0;
            }
          lineStop = -1;
        }

      /***********************************************************************
       *
       *                Check Door
       *
       ***********************************************************************/
      /*
      if(sd_printing && enableDoorPause && !door())
        {
          initPause();
          write_config();

          sd_printing = false;
          sd_pause = true;
          sd_resume = false;
          doorPause = true;
        }
       */

  }
}
