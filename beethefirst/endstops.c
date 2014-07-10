/* Copyright (c) 2013 BEEVC - Electronic Systems        */
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

#include <stdbool.h>

#include "config.h"
#include "pinout.h"

bool hit_home_stop_x (unsigned dir)
{
  if (config.home_direction_x < 0)
  {
    return x_min() && (dir != 0);
  }
  else
  {
    return x_min() && (dir == 0);
  }
}


bool hit_home_stop_y (unsigned dir)
{
  if (config.home_direction_y < 0)
  {
    return y_min() && (dir != 0);
  }
  else
  {
    return y_min() && (dir == 0);
  }
}

bool hit_home_stop_z (unsigned dir)
{
  if (config.home_direction_z < 0)
  {
    return z_min() && (dir != 0);
  }
  else
  {
    return z_min() && (dir == 0);
  }
}



