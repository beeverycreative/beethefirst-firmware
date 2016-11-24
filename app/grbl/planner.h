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

#define BLOCK_BUFFER_SIZE 100

//BEETHEFIRST SETTINGS
#ifndef BTF_SMOOTHIE
  #define STEPS_MM_X              78.74
  #define STEPS_MM_Y              78.74
  #define STEPS_MM_Z              112.49
  #define STEPS_MM_E0             441.3897


  #define MAX_FEED_X              60000
  #define MAX_FEED_Y              60000
  #define MAX_FEED_Z              60000
  #define MAX_FEED_E0             60000

  #define SEARCH_FEED_X           1000
  #define SEARCH_FEED_Y           1000
  #define SEARCH_FEED_Z           1000
  #define SEARCH_FEED_E0          1000

  #define HOME_FEED_X             2000
  #define HOME_FEED_Y             2000
  #define HOME_FEED_Z             2000
  #define HOME_FEED_E0            2000

  #define HOME_DIR_X              -1
  #define HOME_DIR_Y              -1
  #define HOME_DIR_Z              1

  #define HOME_POS_X              -35
  #define HOME_POS_Y              67

  #define PRINT_VOL_X             190
  #define PRINT_VOL_Y             135
  #define PRINT_VOL_Z             123.495
#endif

#if defined(BTF_SMOOTHIE) && defined(BTF_SMOOTHIE_V1) && !defined(BTF_SMOOTHIE_V2)
  #define STEPS_MM_X              80
  #define STEPS_MM_Y              80
  #define STEPS_MM_Z              400
  #define STEPS_MM_E0             441.3897


  #define MAX_FEED_X              8000
  #define MAX_FEED_Y              8000
  #define MAX_FEED_Z              500
  #define MAX_FEED_E0             10000

  #define SEARCH_FEED_X           2000
  #define SEARCH_FEED_Y           2000
  #define SEARCH_FEED_Z           100
  #define SEARCH_FEED_E0          1000

  #define HOME_FEED_X             4000
  #define HOME_FEED_Y             4000
  #define HOME_FEED_Z             500
  #define HOME_FEED_E0            2000

  #define HOME_DIR_X              -1
  #define HOME_DIR_Y              -1
  #define HOME_DIR_Z              1

  #define HOME_POS_X              -122.5
  #define HOME_POS_Y              -172.5
  #define HOME_POS_Z              148.5

  #define PRINT_VOL_X             245
  #define PRINT_VOL_Y             345
  #define PRINT_VOL_Z             148.5
#endif

#if defined(BTF_SMOOTHIE) && defined(BTF_SMOOTHIE_V2) && !defined(BTF_SMOOTHIE_V1)
  #define STEPS_MM_X              80
  #define STEPS_MM_Y              80
  #define STEPS_MM_Z              2133.33
  #define STEPS_MM_E0             441.3897


  #define MAX_FEED_X              60000
  #define MAX_FEED_Y              60000
  #define MAX_FEED_Z              500
  #define MAX_FEED_E0             60000

  #define SEARCH_FEED_X           2000
  #define SEARCH_FEED_Y           2000
  #define SEARCH_FEED_Z           100
  #define SEARCH_FEED_E0          1000

  #define HOME_FEED_X             4000
  #define HOME_FEED_Y             4000
  #define HOME_FEED_Z             500
  #define HOME_FEED_E0            2000

  #define HOME_DIR_X              -1
  #define HOME_DIR_Y              -1
  #define HOME_DIR_Z              1

  #define HOME_POS_X              -45
  #define HOME_POS_Y              -52.5
  #define HOME_POS_Z              80

  #define PRINT_VOL_X             70
  #define PRINT_VOL_Y             120
  #define PRINT_VOL_Z             85
#endif
/*
double MAX_FEED_X;
double MAX_FEED_Y;
double MAX_FEED_Z;
double MAX_FEED_E0;

double SEARCH_FEED_X;
double SEARCH_FEED_Y;
double SEARCH_FEED_Z;
double SEARCH_FEED_E0;

double HOME_FEED_X;
double HOME_FEED_Y;
double HOME_FEED_Z;
double HOME_FEED_E0;

double HOME_DIR_X;
double HOME_DIR_Y;
double HOME_DIR_Z;

double HOME_POS_X;
double HOME_POS_Y;

double PRINT_VOL_X;
double PRINT_VOL_Y;
double PRINT_VOL_Z;
*/

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

// flush the queue for eg; emergency stop
void queue_flush(void);
#endif
