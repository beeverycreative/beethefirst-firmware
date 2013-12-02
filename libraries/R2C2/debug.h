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

#ifndef	_DEBUG_H
#define	_DEBUG_H

#include "sersendf.h"
#include "uart.h"

// select one of sersendf (USB) or UART function for debug 
#define debug sersendf

//#define debug uart_writestr

#endif	/* _DEBUG_H */
