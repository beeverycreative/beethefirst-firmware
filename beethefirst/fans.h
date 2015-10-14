#ifndef _FANS_H
#define _FANS_H

#include <stdint.h>
#ifndef BTF_SMOOTHIE
#include "pinout.h"
#endif
#ifdef BTF_SMOOTHIE
#include "pinout_smoothie.h"
#endif

uint32_t  currenBWSpeed;

void enableBlower(void);
void disableBlower(void);
void setBlowerSpeed(int16_t speed);


#endif  /*_FANS_H*/
