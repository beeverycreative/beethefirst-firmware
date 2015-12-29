#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <stdbool.h>
#include "planner.h"
#include "config.h"
#include "stepper.h"

char position_ok;

//Filament encoder
int32_t encoderPos;
double encoderMM;
double encoderOffset;

void enqueue_moved (tTarget *pTarget);
void enqueue_wait (void);
void synch_queue (void);

void SpecialMoveXY(double x, double y, double f);
void SpecialMoveZ(double z, double f);
void SpecialMove(double x, double y, double z, double e, double f);
void SpecialMoveE (double e, double feed_rate);

void home_x(void);
void home_y(void);
void home_z(void);
void home_e(void);
void home(void);

void GoTo5D(double x, double y, double z, double e, double f);
void Extrude(double e, double f);

void SetXPos(double x);
void SetYPos(double y);
void SetZPos(double z);
void SetEPos(double e);

void GetEncoderPos(void);

void increaseLineNumber(void);

#endif /* _CONTROLLER_H */
