#include "MovementController.h"


/******************************************************************************
 *
 *                      enqueue_moved
 *
 ******************************************************************************/
void enqueue_moved (tTarget *pTarget)
{
  // grbl
  tActionRequest request;


  if (pTarget->x != startpoint.x || pTarget->y != startpoint.y ||
      pTarget->z != startpoint.z || pTarget->e != startpoint.e
  )
    {
      request.ActionType = AT_MOVE;
      request.target= *pTarget;
      request.target.invert_feed_rate =  false;

      plan_buffer_action (&request);
    }
  else
    {
      // no move, just set feed rate
      plan_set_feed_rate (pTarget);
    }
}

/******************************************************************************
 *
 *                      enqueue_wait
 *
 ******************************************************************************/
void enqueue_wait (void)
{
  tActionRequest request;

  request.ActionType = AT_WAIT;
  plan_buffer_action (&request);
}

/******************************************************************************
 *
 *                      synch_queue
 *
 ******************************************************************************/
void synch_queue (void)
{
  st_synchronize();
}

/******************************************************************************
 *
 *                      SpecialMoveXY
 *
 ******************************************************************************/
void SpecialMoveXY(double x, double y, double f)
{
  tActionRequest request;

  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = x;
  request.target.y = y;
  request.target.z = startpoint.z;
  request.target.e = startpoint.e;
  request.target.feed_rate = f;
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

/******************************************************************************
 *
 *                      SpecialMoveZ
 *
 ******************************************************************************/
void SpecialMoveZ(double z, double f)
{
  tActionRequest request;

  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = startpoint.x;
  request.target.y = startpoint.y;
  request.target.z = z;
  request.target.e = startpoint.e;
  request.target.feed_rate = f;
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

/******************************************************************************
 *
 *                      SpecialMove
 *
 ******************************************************************************/
void SpecialMove(double x, double y, double z, double e, double f)
{
  tActionRequest request;

  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = x;
  request.target.y = y;
  request.target.z = z;
  request.target.e = e;
  request.target.feed_rate = f;
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

/******************************************************************************
 *
 *                      SpecialMoveE
 *
 ******************************************************************************/
void SpecialMoveE (double e, double feed_rate)
{
  tTarget next_targetd;

  next_targetd = startpoint;
  next_targetd.e = startpoint.e + e;
  next_targetd.feed_rate = feed_rate;
  enqueue_moved(&next_targetd);
}

/******************************************************************************
 *
 *                      home_x
 *
 ******************************************************************************/
void home_x(void)
{
  double aux = config.acceleration;
  config.acceleration = 1000;

  int dir;
  int max_travel;

  if (HOME_DIR_X < 0)
    {
      dir = -1;
    }
  else
    {
      dir = 1;
    }
  max_travel = max (300, PRINT_VOL_X);

  // move to endstop
  SpecialMoveXY(startpoint.x + dir * max_travel, startpoint.y, HOME_FEED_X);
  synch_queue();

  // move forward a bit
  SpecialMoveXY(startpoint.x - dir * 3, startpoint.y, SEARCH_FEED_X);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x + dir * 6, startpoint.y, SEARCH_FEED_X);

  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;

  //R2C2: XICO B3.1 HOT FIX
  new_pos.x = HOME_POS_X;
  plan_set_current_position (&new_pos);

  config.acceleration = aux ;
  position_ok = 1;
}

/******************************************************************************
 *
 *                      home_y
 *
 ******************************************************************************/
void home_y(void)
{
  double aux = config.acceleration;
  config.acceleration = 1000;

  int dir;
  int max_travel;

  if (HOME_DIR_Y < 0)
    {
      dir = -1;
    }
  else
    {
      dir = 1;
    }
  max_travel = max (300, PRINT_VOL_Y);

  // move to endstop
  SpecialMoveXY(startpoint.x, startpoint.y + dir * max_travel, HOME_FEED_Y);
  synch_queue();

  // move forward a bit
  SpecialMoveXY(startpoint.x, startpoint.y - dir * 3, SEARCH_FEED_Y);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x, startpoint.y + dir * 6, SEARCH_FEED_Y);

  synch_queue();

  tTarget new_pos = startpoint;

  //R2C2: XICO B3.1 HOT FIX
  //new_pos.y = HOME_POS_Y;
  new_pos.y = -74.5;
  plan_set_current_position (&new_pos);

  config.acceleration = aux ;
  position_ok = 1;
}

/******************************************************************************
 *
 *                      home_z
 *
 ******************************************************************************/
void home_z(void)
{
  double aux = config.acceleration;
  config.acceleration = 1000;

  int dir;
  int max_travel;

  if (HOME_DIR_Z < 0)
    {
      dir = -1;
    }
  else
    {
      dir = 1;
    }
  max_travel = max (300, PRINT_VOL_Z);

  // move to endstop
  SpecialMoveZ(startpoint.z + dir * max_travel, HOME_FEED_Z);
  synch_queue();


  /**
   * Normal move Z
   **/

  tTarget next_targetd = startpoint;
  next_targetd.x = startpoint.x;
  next_targetd.y = startpoint.y;
  next_targetd.z = startpoint.z - dir * 10;
  next_targetd.e = startpoint.e;
  next_targetd.feed_rate =  HOME_FEED_Z;
  enqueue_moved(&next_targetd);
  synch_queue();
  /*
   * end
   */
   // move forward a bit
  //SpecialMoveZ(startpoint.z - dir * 1, config.search_feedrate_z);
  //synch_queue();

  // move back in to endstop slowly
  //SpecialMoveZ(startpoint.z + dir *15 , config.search_feedrate_z);
  SpecialMoveZ(startpoint.z + dir *15 , SEARCH_FEED_Z);
  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;
  new_pos.z = config.home_pos_z;

  plan_set_current_position (&new_pos);

  config.acceleration = aux ;
  position_ok = 1;
}

/******************************************************************************
 *
 *                      home_e
 *
 ******************************************************************************/
void home_e(void)
{
  double aux = config.acceleration;
  config.acceleration = 1000;

  // extruder only runs one way and we have no "endstop", just set this point as home
  //startpoint.E = current_position.E = 0;
  tTarget new_pos = startpoint;
  new_pos.e = 0;
  plan_set_current_position (&new_pos);

  config.acceleration = aux ;
  position_ok = 1;
}

/******************************************************************************
 *
 *                      home
 *
 ******************************************************************************/
void home(void)
{
  home_z();
  home_x();
  home_y();
  home_e();
}

/******************************************************************************
 *
 *                      GoTo5D
 *
 ******************************************************************************/
void GoTo5D(double x, double y, double z, double e, double f)
{
  tActionRequest request;

  request.ActionType = AT_MOVE;
  request.target.x = x;
  request.target.y = y;
  request.target.z = z;
  request.target.e = e;
  request.target.feed_rate = f;
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

/******************************************************************************
 *
 *                      Extrude
 *
 ******************************************************************************/
void Extrude(double e, double f)
{
  tActionRequest request;

    request.ActionType = AT_MOVE;
    request.target.x = startpoint.x;
    request.target.y = startpoint.y;
    request.target.z = startpoint.z;
    request.target.e = startpoint.e + e;
    request.target.feed_rate = f;
    request.target.invert_feed_rate =  false;
    plan_buffer_action (&request);
}
/******************************************************************************
 *
 *                      SetXPos
 *
 ******************************************************************************/
void SetXPos(double x)
{
  tTarget new_pos;

  // must have no moves pending if changing position
  synch_queue();
  new_pos = startpoint;

  new_pos.x = x;

  plan_set_current_position (&new_pos);
}

/******************************************************************************
 *
 *                      SetYPos
 *
 ******************************************************************************/
void SetYPos(double y)
{
  tTarget new_pos;

  // must have no moves pending if changing position
  synch_queue();
  new_pos = startpoint;

  new_pos.y = y;

  plan_set_current_position (&new_pos);
}

/******************************************************************************
 *
 *                      SetZPos
 *
 ******************************************************************************/
void SetZPos(double z)
{
  tTarget new_pos;

  // must have no moves pending if changing position
  synch_queue();
  new_pos = startpoint;

  new_pos.z = z;

  plan_set_current_position (&new_pos);
}

/******************************************************************************
 *
 *                      SetEPos
 *
 ******************************************************************************/
void SetEPos(double e)
{
  tTarget new_pos;

  // must have no moves pending if changing position
  synch_queue();
  new_pos = startpoint;

  new_pos.e = e;

  plan_set_current_position (&new_pos);
}
