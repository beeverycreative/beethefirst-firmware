#include <stdbool.h>
#include "gcode_parse.h"
#include "gcode_process.h"
#include "config.h"
#include "temp.h"
#include "ff.h"

void initPause(void);
void pausePrint(void);
void resumePrint(void);
