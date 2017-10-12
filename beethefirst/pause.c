#include "pause.h"

void initPause(void)
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
  config.startpoint_feedrate_coeff = feedrate_coeff;
  config.blowerSpeed = currenBWSpeed;

  config.filament_in_spool -= printed_filament;
}

void enterShutDown(void)
{
  if(printerShutdown)
    {
      return;
    }

  if(printerPause)
    {
      //set status to shutdown
      config.status = 9;
      printerPause = false;
      printerShutdown = true;
    }
  else
    {
      initPause();
      sDownAfterPause = true;
    }

  write_config();
}

void pausePrint(void)
{
  tTarget new_pos;
  /*
   * Little retraction to avoid filament break
   */
  SetEPos(0);
  Extrude(-2,6000);
  home();

  sd_pause = false;
  printerPause = true;

  if(sDownAfterPause)
    {
      enterShutDown();
      sDownAfterPause = false;
    }
}



void resumePrint(void)
{
  disableSerialReply = true;

  char str[80];
  home();
  //Load & Seek SD File
  FRESULT res;
  res = sd_init();
  if(res != FR_OK){
      sersendf("error initializing sd card - %d\n",res);
      sd_resume = false;
  }

  if(res == FR_OK)
    {
      sd_close(&file);
      sd_open(&file, config.filename, FA_READ);
      res = f_lseek(&file, config.sd_pos);
      sd_pos = config.sd_pos;
    }

  if(res != FR_OK){
      sersendf("error restarting print");
      sd_resume = false;
  } else {
      /*
       * Clean Nozzle
       */
      SetEPos(0);
      Extrude(20,300);
      Extrude(-2,6000);
      Extrude(2,500);

      printed_filament = config.last_print_filament + 20;
      config.filament_in_spool += config.last_print_filament;

      if(config.blowerSpeed == 0)
        {
          disableBlower();
        }
      else
        {
          setBlowerSpeed(config.blowerSpeed);
          enableBlower();
        }

      /*
       * Set E POS
       */
      SetEPos(config.startpoint_e);
      /*
       * MOVE X Y To initial position
       */
      GoTo5D(config.startpoint_x,config.startpoint_y,startpoint.z,startpoint.e,10000);
      /*
       * Set Filament Coeff.
       */
      filament_coeff = config.startpoint_filament_coeff;
      /*
       * Set Feedrate Coeff.
       */
      feedrate_coeff = config.startpoint_feedrate_coeff;
      /*
       * MOVE Z To initial position
       */
      GoTo5D(startpoint.x,startpoint.y,config.startpoint_z,startpoint.e,10000);
      /*
       * Reduce feedrate to print speed
       */
      GoTo5D(startpoint.x,startpoint.y,startpoint.z,startpoint.e,1200);

      estimated_time = config.estimated_time;
      time_elapsed = config.time_elapsed;
      executed_lines = config.executed_lines;

      sd_restartPrint = true;
      sd_resume = false;
  }
}
