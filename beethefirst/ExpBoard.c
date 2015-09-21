#include "ExpBoard.h"

int32_t getMedianValue(int32_t array[5])
{
  int32_t sortedArray[5];

  for(int i = 0; i < 5; i++)
    {
      sortedArray[i] = array[i];
    }

  bubble_sort(sortedArray, 5);

  return sortedArray[2];
}

void bubble_sort(int32_t list[], int32_t n)
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

#ifdef EXP_Board
void verifySDownConditions(void)
{
  if(sDown_filtered < SDown_Threshold)
    {
      if(sd_printing)
        {
          initPause();
          config.status = 9;
          write_config();
          sd_printing = 0;

          queue_flush();
          reset_current_block();

          home_z();
        }
    }
}
#endif

#ifdef USE_BATT
void verifyBatteryLevels(void)
{
  //Power Supply Disconnected
  if(!ps_ext_state)
    {
      charge_time = 0;
      //Printing
      if(sd_printing)
        {
          //If not in Battery Mode operation
          if(!batteryMode)
            {
              batt_time = 0;
              batteryMode = true;
              charging = false;
            }
          //If printing in Battery Mode
          else
            {
              //Printing time exceeded... Start Pause Procedure
              if(batt_time > (config.batteryPrintTime * 1000)
                  && sd_pause == false)
                {
                  initPause();
                  write_config();

                  sd_printing = false;
                  sd_pause = true;
                  sd_resume = false;
                  printerPause = false;
                }

            }
        }
      //Transfering
      /*
      else if(transfer_mode)
        {

        }
        */
      if(!sd_printing && batteryMode)
        {
          //Printer Paused... Enter Shutdown
          if(batt_time > (config.batteryPrintTime * 1000)
              && printerPause)
            {
              printerShutdown = false;
              enter_power_saving = false;
              config.battery_ShutDown = 1;
              enterShutDown();
            }
          //Printer In Shutdown... Enter Power Saving
          else if(batt_time > (config.batteryPrintTime * 1000)
              && printerShutdown && !in_power_saving)
            {
              if(!enter_power_saving)
                {
                  in_power_saving = false;
                  powerSavingDelay = 5000;
                  enter_power_saving = true;
                  rest_time = 0;
                }
            }
          //Printer in Shutdown and Power Saving
          else if(batt_time > (config.batteryPrintTime * 1000)
              && in_power_saving && printerShutdown)
            {
              //If standby exceeded or Low Battery charge turn off pritner
              if(batt_time > (config.standByTime * 1000) || batt_filtered < 2475){
                  BATT_uC_disable();
              }
            }
        }
      //Not Currently Printing and Not entered battery mode
      else if(!sd_printing && !batteryMode)
        {
          if(!printerPause
              || !printerShutdown
              || !sd_pause
              || !sd_resume
              || !enter_power_saving
              || !in_power_saving)
            {
              //home();
              //BATT_uC_disable();
            }
        }
    }
  //Power Supply Connected
  else
    {
      //If power supply just connected
      if(batteryMode && charge_time < 60000)
        {
          STEP_uC_disable();
        }
      //If 1 minute passed since last power failure
      else if(batteryMode && charge_time >= 60000 && in_power_saving)
        {
          reinit_system();
        }
      else if(batteryMode && charge_time >= 60000 && !in_power_saving)
        {
          batteryMode = false;
        }

      //If not longer in battery mode, charge battery and resume print
      if(!batteryMode)
        {
          //If Auto-resume enabled resume print
          if(config.status == 9
              && config.autoResume == 1
              && sd_resume == false
              && config.battery_ShutDown == 1)
            {
              temp_set(config.startpoint_temperature, EXTRUDER_0);
              enqueue_wait_temp();

              config.battery_ShutDown = 0;
              write_config();

              sd_resume = true;
            }

          //Charge battery
          if(current_temp_r2c2 > 0 && current_temp_r2c2 < 40)
            {
              STEP_uC_enable();
            }
          else
            {
              STEP_uC_disable();
            }
        }
    }
}
#endif
