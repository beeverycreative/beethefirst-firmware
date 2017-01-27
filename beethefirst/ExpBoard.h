#include <stdint.h>
#include <stdlib.h>

#include "gcode_process.h"
#include "gcode_parse.h"
#include "temp.h"
#include "config.h"

#define SDown_Threshold         1400            //2700 -> 24V Shutdown threshold at ~10V
#define BatteryLevel_Threshold         2475

uint16_t getMedianValue(uint16_t array[sDownADC_length]);
void bubble_sort(uint16_t list[], uint16_t n);

void verifySDownConditions(void);

#ifdef USE_BATT
void verifyBatteryLevels(void);
#endif
