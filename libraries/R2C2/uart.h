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

#ifndef	_UART_H
#define	_UART_H

#include <stdint.h>

void uart_init(void);
char uart_data_available(void);
char uart_receive(void);
void uart_send(char byte);
void uart_writestr(char *data);
void uart_write_uint32(uint32_t v);
void uart_writedouble(double v);

// #define serial_writechar(x) uart_send(x)

#endif	/* _UART_H */
