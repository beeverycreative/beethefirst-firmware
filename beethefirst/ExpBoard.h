#include <stdint.h>
#include <stdlib.h>

#include "gcode_process.h"
#include "gcode_parse.h"
#include "temp.h"
#include "config.h"

#define SDown_Threshold	1400      //2700 -> 24V Shutdown threshold at 16V
#define BatteryLevel_Threshold         2475
#define sDownADC_midpos 5

int32_t getMedianValue(int32_t array[sDownADC_length]);
uint16_t getMedian5Value(uint16_t array[5]);
int compare(const void *a, const void *b);
int compare16(const void *a, const void *b);
void bubble_sort(int32_t list[], int32_t n);

void verifySDownConditions(void);

#ifdef USE_BATT
void verifyBatteryLevels(void);
#endif
