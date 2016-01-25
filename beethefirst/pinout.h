/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011-2013 BEEVC - Electronic Systems   */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _PINOUT_H
#define _PINOUT_H

#include "ios.h"
#include "machine.h"

#ifdef BTF_PLUS
  #define EXP_Board       //Expansion Board P02V04A
#endif
#ifdef BTF_ME
  #define EXP_Board       //Expansion Board P02V04A
#endif
#ifdef BTF_SCHOOL
  #define EXP_Board       //Expansion Board P02V04A
#endif
#ifdef BTF_PLUS_BATT
  #define EXP_Board       //Expansion Board P02V04A
  #define USE_BATT
#endif
#ifdef BTF_SCHOOL_BATT
  #define EXP_Board       //Expansion Board P02V04A
  #define USE_BATT
#endif

/*
        Machine Pin Definitions
*/

/*
        user defined pins
        adjust to suit your electronics,
        or adjust your electronics to suit this
*/

//x axis pins
#define X_STEP_PORT     1         /* P1.20 */
#define X_STEP_PIN      (1 << 20) /* P1.20 */

#define X_DIR_PORT      1         /* P1.23 */
#define X_DIR_PIN       (1 << 23) /* P1.23 */

#define X_ENABLE_PORT   1         /* P1.24 */
#define X_ENABLE_PIN    (1 << 24) /* P1.24 */

#define X_MIN_PORT      2         /* P2. 6 */
#define X_MIN_PIN       (1 <<  6) /* P2. 6 */

//y axis pins
#define Y_STEP_PORT     1         /* P1.25 */
#define Y_STEP_PIN      (1 << 25) /* P1.25 */

#define Y_DIR_PORT      1         /* P1.26 */
#define Y_DIR_PIN       (1 << 26) /* P1.26 */

#define Y_ENABLE_PORT   1         /* P1.28 */
#define Y_ENABLE_PIN    (1 << 28) /* P1.28 */

#define Y_MIN_PORT      2         /* P2. 7 */
#define Y_MIN_PIN       (1 <<  7) /* P2. 7 */

//z axis pins
#define Z_STEP_PORT     1         /* P1.29 */
#define Z_STEP_PIN      (1 << 29) /* P1.29 */

#define Z_DIR_PORT      0         /* P0. 0 */
#define Z_DIR_PIN       (1 <<  0) /* P0. 0 */

#define Z_ENABLE_PORT   0         /* P0. 1 */
#define Z_ENABLE_PIN    (1 <<  1) /* P0. 1 */

#define Z_MIN_PORT      2         /* P2. 8 */
#define Z_MIN_PIN       (1 <<  8) /* P2. 8 */

//e axis pins
#define E_STEP_PORT     0         /* P0.10 */
#define E_STEP_PIN      (1 << 10) /* P0.10 */

#define E_DIR_PORT      0         /* P0.11 */
#define E_DIR_PIN       (1 << 11) /* P0.11 */

#define E_ENABLE_PORT   2         /* P2.10 */
#define E_ENABLE_PIN    (1 << 10) /* P2.10 */

#define STEPPERS_RESET_PORT     0         /* P0.22 */
#define STEPPERS_RESET_PIN      (1 << 22) /* P0.22 */

#define EXTRUDER_0_HEATER_PORT          2        /* P2.4 */
#define EXTRUDER_0_HEATER_PIN           (1 << 4) /* P2.4 */
#define EXTRUDER_0_HEATER_PIN_Number    4
#define EXTRUDER_0_SENSOR_ADC_PORT      0        /* P0.2 */
#define EXTRUDER_0_SENSOR_ADC_PIN       2        /* P0.2 */
#define EXTRUDER_0_SENSOR_ADC_CHANNEL   7        /* P0.2 */
#define EXTRUDER_0_PWM_CHANNEL          5

#define EXTRUDER_0_FAN_PORT             2         /* P2.3 */
#define EXTRUDER_0_FAN_PIN              (1<<3)

/*
        Heated Bed
*/
#define HEATED_BED_0_HEATER_PORT        2        /* P2.5 */
#define HEATED_BED_0_HEATER_PIN         (1 << 5) /* P2.5 */
#define HEATED_BED_0_HEATER_PIN_Number  5
#define HEATED_BED_0_ADC_PORT           0        /* P0.3 */
#define HEATED_BED_0_ADC_PIN            3        /* P0.3 */
#define HEATED_BED_0_SENSOR_ADC_CHANNEL 6        /* P0.3 */
#define HEATED_BED_0_PWM_CHANNEL        6

#define heated_bed_on() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, HIGH);
#define heated_bed_off() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, LOW);

#define BUZZER_PORT                     2        /* P2.2 */
#define BUZZER_PIN                      (1 << 2) /* P2.2 */
#define BUZZER_PIN_Number               2
#define BUZZER_PWM_CHANNEL              3

#ifdef EXP_Board
/*
        Expansion Board
*/

  #define R2C2_TEMP_ADC_PORT              0       /* P0.25 */
  #define R2C2_TEMP_ADC_PIN               25      /* P0.25 */
  #define R2C2_TEMP_SENSOR_ADC_CHANNEL    2       /* P0.25 */

  #define R2C2_FAN_PORT                   1       /* P1.22 */
  #define R2C2_FAN_PIN                    (1 << 22)      /* P1.22 */

  #define ILUM_PORT                       1       /* P1.4 */
  #define ILUM_PIN                        (1 << 4)      /* P1.4 */

  #define BW_ON_PORT                      1       /* P1.1 */
  #define BW_ON_PIN                       (1 << 1)       /* P1.1 */
  #define BW_V1_PORT                      2       /* P2.3 */
  #define BW_V1_PIN                       3       /* P2.3 */
  #define BW_PWM_CHANNEL                  4       /* PWM4 */

  #define SDOWN_ADC_PORT                  1       /* P1.31 */
  #define SDOWN_ADC_PIN                   31      /* P1.31 */
  #define SDOWN_ADC_SENSOR_ADC_CHANNEL    5        /* P1.31 */

  #define FAN_EXT_ON_PORT                 1       /* P1.10 */
  #define FAN_EXT_ON_PIN                  (1 << 10)      /* P1.10 */
  #define FAN_EXT_V1_PORT                 2       /* P2.1 */
  #define FAN_EXT_V1_PIN                  1       /* P2.1 */
  #define FAN_EXT_PWM_CHANNEL             2       /* PWM2 */

  #define LOGO_ON_PORT                    2       /* P2.0 */
  #define LOGO_ON_PIN                     0       /* P2.0 */
  #define LOGO_PWM_CHANNEL                1       /* PWM1 */

  #define r2c2_fan_on() digital_write(R2C2_FAN_PORT, R2C2_FAN_PIN, HIGH);
  #define r2c2_fan_off() digital_write(R2C2_FAN_PORT, R2C2_FAN_PIN, LOW);

  #define blower_on() digital_write(BW_ON_PORT, BW_ON_PIN, HIGH);
  #define blower_off() digital_write(BW_ON_PORT, BW_ON_PIN, LOW);

  #define extruder_block_fan_on() digital_write(FAN_EXT_ON_PORT, FAN_EXT_ON_PIN, HIGH);
  #define extruder_block_fan_off() digital_write(FAN_EXT_ON_PORT, FAN_EXT_ON_PIN, LOW);

  //#define logo_on() digital_write(LOGO_ON_PORT, LOGO_ON_PIN, HIGH);
  //#define logo_off() digital_write(LOGO_ON_PORT, LOGO_ON_PIN, LOW);

  #define ilum_on() digital_write(ILUM_PORT, ILUM_PIN, HIGH);
  #define ilum_off() digital_write(ILUM_PORT, ILUM_PIN, LOW);

#endif

#ifdef USE_BATT
  #define BATT_ADC_PORT                  0       /* P0.26 */
  #define BATT_ADC_PIN                   26      /* P0.26 */
  #define BATT_ADC_SENSOR_ADC_CHANNEL    3       /* P0.26 */

  #define PS_EXT_READ_PORT                   0       /* P0.7 */
  #define PS_EXT_READ_PIN                    (1 << 7)      /* P0.7 */

  #define STEP_uC_ON_PORT                   0       /* P0.6 */
  #define STEP_uC_ON_PIN                    (1 << 6)      /* P0.6 */

  #define BATT_uC_ON_PORT                   1       /* P1.8 */
  #define BATT_uC_ON_PIN                    (1 << 8)      /* P1.8 */

  #define STEP_uC_enable() digital_write(STEP_uC_ON_PORT, STEP_uC_ON_PIN, 1)
  #define STEP_uC_disable() digital_write(STEP_uC_ON_PORT, STEP_uC_ON_PIN, 0)

  #define BATT_uC_enable() digital_write(BATT_uC_ON_PORT, BATT_uC_ON_PIN, 0)
  #define BATT_uC_disable() digital_write(BATT_uC_ON_PORT, BATT_uC_ON_PIN, 1)
#endif

/*
        X Stepper
*/
#define x_enable() digital_write(X_ENABLE_PORT, X_ENABLE_PIN, 0)
#define x_disable() digital_write(X_ENABLE_PORT, X_ENABLE_PIN, 1)
#define x_step() digital_write(X_STEP_PORT, X_STEP_PIN, 1)
#define x_unstep() digital_write(X_STEP_PORT, X_STEP_PIN, 0)
#define x_direction(dir) digital_write(X_DIR_PORT, X_DIR_PIN, dir)
#define x_min() ((digital_read(X_MIN_PORT, X_MIN_PIN))?0:1)

/*
        Y Stepper
*/
#define y_enable() digital_write(Y_ENABLE_PORT, Y_ENABLE_PIN, 0)
#define y_disable() digital_write(Y_ENABLE_PORT, Y_ENABLE_PIN, 1)
#define y_step() digital_write(Y_STEP_PORT, Y_STEP_PIN, 1)
#define y_unstep() digital_write(Y_STEP_PORT, Y_STEP_PIN, 0)
#define y_direction(dir) digital_write(Y_DIR_PORT, Y_DIR_PIN, dir)
#define y_min() ((digital_read(Y_MIN_PORT, Y_MIN_PIN))?0:1)

/*
        Z Stepper
*/
#define z_enable() digital_write(Z_ENABLE_PORT, Z_ENABLE_PIN, 0)
#define z_disable() digital_write(Z_ENABLE_PORT, Z_ENABLE_PIN, 1)
#define z_step() digital_write(Z_STEP_PORT, Z_STEP_PIN, 1)
#define z_unstep() digital_write(Z_STEP_PORT, Z_STEP_PIN, 0)
#define z_direction(dir) digital_write(Z_DIR_PORT, Z_DIR_PIN, dir)
#define z_min() ((digital_read(Z_MIN_PORT, Z_MIN_PIN))?0:1)

/*
        Extruder
*/
#define e_enable() digital_write(E_ENABLE_PORT, E_ENABLE_PIN, 0)
#define e_disable() digital_write(E_ENABLE_PORT, E_ENABLE_PIN, 1)
#define e_step() digital_write(E_STEP_PORT, E_STEP_PIN, 1)
#define e_unstep() digital_write(E_STEP_PORT, E_STEP_PIN, 0)
#define e_direction(dir) digital_write(E_DIR_PORT, E_DIR_PIN, dir)

#define extruder_heater_on() digital_write(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, HIGH);
#define extruder_heater_off() digital_write(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, LOW);

#define extruder_fan_on() digital_write(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, HIGH);
#define extruder_fan_off() digital_write(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, LOW);

/*
        Heated Bed
*/
#define heated_bed_on() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, HIGH);
#define heated_bed_off() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, LOW);


/*
        End Step - All Steppers
        (so we don't have to delay in interrupt context)
*/

#define unstep()                                                        do { x_unstep(); y_unstep(); z_unstep(); e_unstep(); } while (0)

#endif  /* _PINOUT_H */
