#include <stdbool.h>
#include "gcode_parse.h"
#include "gcode_process.h"
#include "config.h"
#include "temp.h"
#include "ff.h"

extern void initPause();
extern void pausePrint();
extern void resumePrint();
