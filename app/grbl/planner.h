/*
  planner.h - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon  

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef planner_h
#define planner_h
                 
#include <inttypes.h>

typedef enum {
  AT_MOVE,
  AT_MOVE_ENDSTOP,
  AT_WAIT,
  AT_WAIT_TEMPS
  } eActionType;
        
typedef  struct {
  double  x;
  double  y;
  double  z;
  double  e;
  double  feed_rate;
  uint8_t invert_feed_rate;
} tTarget;

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  eActionType action_type;
  
  // Fields used by the bresenham algorithm for tracing the line
  uint32_t steps_x, steps_y, steps_z; // Step count along each axis
  uint32_t steps_e; 
  uint32_t direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  int32_t  step_event_count;          // The number of step events required to complete this block
  uint32_t nominal_rate;              // The nominal step rate for this block in step_events/minute
  
  // Fields used by the motion planner to manage acceleration
  double nominal_speed;               // The nominal speed for this block in mm/min  
  double entry_speed;                 // Entry speed at previous-current junction in mm/min
  double max_entry_speed;             // Maximum allowable junction entry speed in mm/min
  double millimeters;                 // The total travel of this block in mm
  uint8_t recalculate_flag;           // Planner flag to recalculate trapezoids on entry junction
  uint8_t nominal_length_flag;        // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  uint32_t initial_rate;              // The jerk-adjusted step rate at start of block  
  uint32_t final_rate;                // The minimal rate at exit
  int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
  uint32_t accelerate_until;          // The index of the step event on which to stop acceleration
  uint32_t decelerate_after;          // The index of the step event on which to start decelerating
  
  // extra
  uint8_t check_endstops; // for homing moves
} block_t;

        
typedef struct {
  eActionType ActionType;
  
  tTarget     target;  
  
  uint16_t    wait_param; // time or target temp
} tActionRequest;



extern tTarget startpoint;

      
// Initialize the motion plan subsystem      
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
//void plan_buffer_line(double x, double y, double z, double feed_rate, uint8_t invert_feed_rate);
void plan_buffer_line (tActionRequest *pAction);

void plan_buffer_action(tActionRequest *pAction);

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
block_t *plan_get_current_block();

// Enables or disables acceleration-management for upcoming blocks
void plan_set_acceleration_manager_enabled(uint8_t enabled);

// Is acceleration-management currently enabled?
int plan_is_acceleration_manager_enabled();

// Reset the position vector
void plan_set_current_position(tTarget *new_position); 

void plan_set_current_position_xyz(double x, double y, double z); 

void plan_set_feed_rate (tTarget *new_position);

uint8_t plan_queue_full (void);

uint8_t plan_queue_empty(void);

uint8_t plan_queue_size(void);


#endif
