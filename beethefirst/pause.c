#include "pause.h"

void initPause()
{
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
}

void pausePrint()
{
  tTarget new_pos;
  /*
   * Little retraction to avoid filament break
   */
  gcode_parse_str("G92 E\n");
  gcode_parse_str("G1 E-2 F6000\n");
  gcode_parse_str("G28\n");

  //config.status = 7;
  sd_pause = false;
  printerPause = true;
}



void resumePrint()
{
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
}
