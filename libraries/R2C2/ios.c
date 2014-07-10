/* Copyright (c) 2011-2013 BEEVC - Electronic Systems        */
/*
 * This file is part of BEESOFT software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version. BEESOFT is
 * distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details. You should have received a copy of the
 * GNU General Public License along with BEESOFT. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include "lpc_types.h"
#include "lpc17xx_gpio.h"
#include "ios.h"

/* Initialize all the IO pins */
/* Example of usage: pin_mode(PORT_0, X_STEP_PIN, OUTPUT); */
void pin_mode(uint8_t portNum, uint32_t bitValue, uint8_t dir)
{
    FIO_SetDir(portNum, bitValue, dir);
}

/* Example of usage: digital_write(PORT_0, X_STEP_PIN, HIGH); */
void digital_write(uint8_t portNum, uint32_t bitValue, uint8_t state)
{
    if (state)
        FIO_SetValue(portNum, bitValue);

    else
        FIO_ClearValue(portNum, bitValue);
}

/* Example of usage: value = digital_read(PORT_0, PIN); */
uint32_t digital_read(uint8_t portNum, uint32_t bitValue)
{
  return ((FIO_ReadValue(portNum) & bitValue)?1:0);
}
