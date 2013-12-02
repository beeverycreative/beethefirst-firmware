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

#ifndef IOS_H
#define IOS_H

#define INPUT 0
#define OUTPUT 1

#define HIGH 1
#define LOW 0


void digital_write(uint8_t portNum, uint32_t bitValue, uint8_t dir);
void pin_mode(uint8_t portNum, uint32_t bitValue, uint8_t dir);
uint32_t digital_read(uint8_t portNum, uint32_t bitValue);

#endif
