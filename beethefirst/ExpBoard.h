#include <stdint.h>
#include <stdlib.h>

#include "gcode_process.h"
#include "gcode_parse.h"
#include "temp.h"
#include "config.h"

int32_t getMedianValue(int32_t array[sDownADC_length]);
void bubble_sort(int32_t list[], int32_t n);

void verifySDownConditions(void);

#ifdef USE_BATT
void verifyBatteryLevels(void);
#endif
