/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
  stepper_indirection.h - stepper motor driver indirection macros
  to allow some stepper functions to be done via SPI/I2c instead of direct pin manipulation
  Part of Marlin

  Copyright (c) 2015 Dominik Wenger

  Marlin is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Marlin is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Marlin.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPER_INDIRECTION_H
#define STEPPER_INDIRECTION_H

#include "MarlinConfig.h"
// X Stepper
#define X_ENABLE_INIT SET_OUTPUT(X_ENABLE_PIN)
#define X_ENABLE_WRITE(STATE) WRITE(X_ENABLE_PIN,STATE)
#define X_ENABLE_READ READ(X_ENABLE_PIN)
#define X_DIR_INIT SET_OUTPUT(X_DIR_PIN)
#define X_DIR_WRITE(STATE) WRITE(X_DIR_PIN,STATE)
#define X_DIR_READ READ(X_DIR_PIN)
#define X_STEP_INIT SET_OUTPUT(X_STEP_PIN)
#define X_STEP_WRITE(STATE) WRITE(X_STEP_PIN,STATE)
#define X_STEP_READ READ(X_STEP_PIN)

// Y Stepper

#define Y_ENABLE_INIT SET_OUTPUT(Y_ENABLE_PIN)
#define Y_ENABLE_WRITE(STATE) WRITE(Y_ENABLE_PIN,STATE)
#define Y_ENABLE_READ READ(Y_ENABLE_PIN)
#define Y_DIR_INIT SET_OUTPUT(Y_DIR_PIN)
#define Y_DIR_WRITE(STATE) WRITE(Y_DIR_PIN,STATE)
#define Y_DIR_READ READ(Y_DIR_PIN)
#define Y_STEP_INIT SET_OUTPUT(Y_STEP_PIN)
#define Y_STEP_WRITE(STATE) WRITE(Y_STEP_PIN,STATE)
#define Y_STEP_READ READ(Y_STEP_PIN)

// Z Stepper
#define Z_ENABLE_INIT SET_OUTPUT(Z_ENABLE_PIN)
#define Z_ENABLE_WRITE(STATE) WRITE(Z_ENABLE_PIN,STATE)
#define Z_ENABLE_READ READ(Z_ENABLE_PIN)
#define Z_DIR_INIT SET_OUTPUT(Z_DIR_PIN)
#define Z_DIR_WRITE(STATE) WRITE(Z_DIR_PIN,STATE)
#define Z_DIR_READ READ(Z_DIR_PIN)
#define Z_STEP_INIT SET_OUTPUT(Z_STEP_PIN)
#define Z_STEP_WRITE(STATE) WRITE(Z_STEP_PIN,STATE)
#define Z_STEP_READ READ(Z_STEP_PIN)

// X2 Stepper
#if HAS_X2_ENABLE
  #define X2_ENABLE_INIT SET_OUTPUT(X2_ENABLE_PIN)
  #define X2_ENABLE_WRITE(STATE) WRITE(X2_ENABLE_PIN,STATE)
  #define X2_ENABLE_READ READ(X2_ENABLE_PIN)
  #define X2_DIR_INIT SET_OUTPUT(X2_DIR_PIN)
  #define X2_DIR_WRITE(STATE) WRITE(X2_DIR_PIN,STATE)
  #define X2_DIR_READ READ(X2_DIR_PIN)
  #define X2_STEP_INIT SET_OUTPUT(X2_STEP_PIN)
  #define X2_STEP_WRITE(STATE) WRITE(X2_STEP_PIN,STATE)
  #define X2_STEP_READ READ(X2_STEP_PIN)
#endif

// Y2 Stepper
#if HAS_Y2_ENABLE
  #define Y2_ENABLE_INIT SET_OUTPUT(Y2_ENABLE_PIN)
  #define Y2_ENABLE_WRITE(STATE) WRITE(Y2_ENABLE_PIN,STATE)
  #define Y2_ENABLE_READ READ(Y2_ENABLE_PIN)
  #define Y2_DIR_INIT SET_OUTPUT(Y2_DIR_PIN)
  #define Y2_DIR_WRITE(STATE) WRITE(Y2_DIR_PIN,STATE)
  #define Y2_DIR_READ READ(Y2_DIR_PIN)
  #define Y2_STEP_INIT SET_OUTPUT(Y2_STEP_PIN)
  #define Y2_STEP_WRITE(STATE) WRITE(Y2_STEP_PIN,STATE)
  #define Y2_STEP_READ READ(Y2_STEP_PIN)
#endif

// Z2 Stepper
#if HAS_Z2_ENABLE
  #define Z2_ENABLE_INIT SET_OUTPUT(Z2_ENABLE_PIN)
  #define Z2_ENABLE_WRITE(STATE) WRITE(Z2_ENABLE_PIN,STATE)
  #define Z2_ENABLE_READ READ(Z2_ENABLE_PIN)
  #define Z2_DIR_INIT SET_OUTPUT(Z2_DIR_PIN)
  #define Z2_DIR_WRITE(STATE) WRITE(Z2_DIR_PIN,STATE)
  #define Z2_DIR_READ READ(Z2_DIR_PIN)
  #define Z2_STEP_INIT SET_OUTPUT(Z2_STEP_PIN)
  #define Z2_STEP_WRITE(STATE) WRITE(Z2_STEP_PIN,STATE)
  #define Z2_STEP_READ READ(Z2_STEP_PIN)
#endif

// E0 Stepper
#define E0_ENABLE_INIT SET_OUTPUT(E0_ENABLE_PIN)
#define E0_ENABLE_WRITE(STATE) WRITE(E0_ENABLE_PIN,STATE)
#define E0_ENABLE_READ READ(E0_ENABLE_PIN)
#define E0_DIR_INIT SET_OUTPUT(E0_DIR_PIN)
#define E0_DIR_WRITE(STATE) WRITE(E0_DIR_PIN,STATE)
#define E0_DIR_READ READ(E0_DIR_PIN)
#define E0_STEP_INIT SET_OUTPUT(E0_STEP_PIN)
#define E0_STEP_WRITE(STATE) WRITE(E0_STEP_PIN,STATE)
#define E0_STEP_READ READ(E0_STEP_PIN)

// E1 Stepper
#define E1_ENABLE_INIT SET_OUTPUT(E1_ENABLE_PIN)
#define E1_ENABLE_WRITE(STATE) WRITE(E1_ENABLE_PIN,STATE)
#define E1_ENABLE_READ READ(E1_ENABLE_PIN)
#define E1_DIR_INIT SET_OUTPUT(E1_DIR_PIN)
#define E1_DIR_WRITE(STATE) WRITE(E1_DIR_PIN,STATE)
#define E1_DIR_READ READ(E1_DIR_PIN)
#define E1_STEP_INIT SET_OUTPUT(E1_STEP_PIN)
#define E1_STEP_WRITE(STATE) WRITE(E1_STEP_PIN,STATE)
#define E1_STEP_READ READ(E1_STEP_PIN)

// E2 Stepper
#if ENABLED() && ENABLED(E2_IS_L6470)
  extern L6470 stepperE2;
  #define E2_ENABLE_INIT NOOP
  #define E2_ENABLE_WRITE(STATE) do{if(STATE) stepperE2.Step_Clock(stepperE2.getStatus() & STATUS_HIZ); else stepperE2.softFree();}while(0)
  #define E2_ENABLE_READ (stepperE2.getStatus() & STATUS_HIZ)
  #define E2_DIR_INIT NOOP
  #define E2_DIR_WRITE(STATE) stepperE2.Step_Clock(STATE)
  #define E2_DIR_READ (stepperE2.getStatus() & STATUS_DIR)
#else
  #define E2_ENABLE_INIT SET_OUTPUT(E2_ENABLE_PIN)
  #define E2_ENABLE_WRITE(STATE) WRITE(E2_ENABLE_PIN,STATE)
  #define E2_ENABLE_READ READ(E2_ENABLE_PIN)
  #define E2_DIR_INIT SET_OUTPUT(E2_DIR_PIN)
  #define E2_DIR_WRITE(STATE) WRITE(E2_DIR_PIN,STATE)
  #define E2_DIR_READ READ(E2_DIR_PIN)
#endif
#define E2_STEP_INIT SET_OUTPUT(E2_STEP_PIN)
#define E2_STEP_WRITE(STATE) WRITE(E2_STEP_PIN,STATE)
#define E2_STEP_READ READ(E2_STEP_PIN)
#define E3_ENABLE_INIT SET_OUTPUT(E3_ENABLE_PIN)
#define E3_ENABLE_WRITE(STATE) WRITE(E3_ENABLE_PIN,STATE)
#define E3_ENABLE_READ READ(E3_ENABLE_PIN)
#define E3_DIR_INIT SET_OUTPUT(E3_DIR_PIN)
#define E3_DIR_WRITE(STATE) WRITE(E3_DIR_PIN,STATE)
#define E3_DIR_READ READ(E3_DIR_PIN)
#define E3_STEP_INIT SET_OUTPUT(E3_STEP_PIN)
#define E3_STEP_WRITE(STATE) WRITE(E3_STEP_PIN,STATE)
#define E3_STEP_READ READ(E3_STEP_PIN)

/**
 * Extruder indirection for the single E axis
 */
#if EXTRUDERS > 3
  #define E_STEP_WRITE(v) { switch (current_block->active_extruder) { case 0: E0_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); break; case 3: E3_STEP_WRITE(v); } }
  #define NORM_E_DIR() { switch (current_block->active_extruder) { case 0: E0_DIR_WRITE(!INVERT_E0_DIR); break; case 1: E1_DIR_WRITE(!INVERT_E1_DIR); break; case 2: E2_DIR_WRITE(!INVERT_E2_DIR); break; case 3: E3_DIR_WRITE(!INVERT_E3_DIR); } }
  #define REV_E_DIR() { switch (current_block->active_extruder) { case 0: E0_DIR_WRITE(INVERT_E0_DIR); break; case 1: E1_DIR_WRITE(INVERT_E1_DIR); break; case 2: E2_DIR_WRITE(INVERT_E2_DIR); break; case 3: E3_DIR_WRITE(INVERT_E3_DIR); } }
#elif EXTRUDERS > 2
  #define E_STEP_WRITE(v) { switch (current_block->active_extruder) { case 0: E0_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); } }
  #define NORM_E_DIR() { switch (current_block->active_extruder) { case 0: E0_DIR_WRITE(!INVERT_E0_DIR); break; case 1: E1_DIR_WRITE(!INVERT_E1_DIR); break; case 2: E2_DIR_WRITE(!INVERT_E2_DIR); } }
  #define REV_E_DIR() { switch (current_block->active_extruder) { case 0: E0_DIR_WRITE(INVERT_E0_DIR); break; case 1: E1_DIR_WRITE(INVERT_E1_DIR); break; case 2: E2_DIR_WRITE(INVERT_E2_DIR); } }
#elif EXTRUDERS > 1
  #define E_STEP_WRITE(v) { if (current_block->active_extruder == 0) { E0_STEP_WRITE(v); } else { E1_STEP_WRITE(v); } }
  #define NORM_E_DIR() { if (current_block->active_extruder == 0) { E0_DIR_WRITE(!INVERT_E0_DIR); } else { E1_DIR_WRITE(!INVERT_E1_DIR); } }
  #define REV_E_DIR() { if (current_block->active_extruder == 0) { E0_DIR_WRITE(INVERT_E0_DIR); } else { E1_DIR_WRITE(INVERT_E1_DIR); } }
#else
  #define E_STEP_WRITE(v) E0_STEP_WRITE(v)
  #define NORM_E_DIR() E0_DIR_WRITE(!INVERT_E0_DIR)
  #define REV_E_DIR() E0_DIR_WRITE(INVERT_E0_DIR)
#endif

#endif // STEPPER_INDIRECTION_H
