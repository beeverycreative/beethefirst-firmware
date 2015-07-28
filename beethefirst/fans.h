#ifndef _FANS_H
#define _FANS_H

#include <stdint.h>
#include "pinout.h"

uint32_t  currenBWSpeed;

void enableBlower(void);
void disableBlower(void);
void setBlowerSpeed(int16_t speed);


#endif  /*_FANS_H*/
