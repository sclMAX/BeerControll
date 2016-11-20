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
 * Conditionals_post.h
 * Defines that depend on configuration but are not editable.
 */

#ifndef CONDITIONALS_POST_H
#define CONDITIONALS_POST_H

  #if ENABLED(EMERGENCY_PARSER)
    #define EMERGENCY_PARSER_CAPABILITIES " EMERGENCY_CODES:M108,M112,M410"
  #else
    #define EMERGENCY_PARSER_CAPABILITIES ""
  #endif

  /**
   * Axis lengths
   */
  #define X_MAX_LENGTH (X_MAX_POS - (X_MIN_POS))
  #define Y_MAX_LENGTH (Y_MAX_POS - (Y_MIN_POS))
  #define Z_MAX_LENGTH (Z_MAX_POS - (Z_MIN_POS))
   /**
   * Set the home position based on settings or manual overrides
   */
  #ifdef MANUAL_X_HOME_POS
    #define X_HOME_POS MANUAL_X_HOME_POS
  #elif ENABLED(BED_CENTER_AT_0_0)
    #if ENABLED(DELTA)
      #define X_HOME_POS 0
    #else
      #define X_HOME_POS ((X_MAX_LENGTH) * (X_HOME_DIR) * 0.5)
    #endif
  #else
    #if ENABLED(DELTA)
      #define X_HOME_POS ((X_MAX_LENGTH) * 0.5)
    #else
      #define X_HOME_POS (X_HOME_DIR < 0 ? X_MIN_POS : X_MAX_POS)
    #endif
  #endif

  #ifdef MANUAL_Y_HOME_POS
    #define Y_HOME_POS MANUAL_Y_HOME_POS
  #elif ENABLED(BED_CENTER_AT_0_0)
    #if ENABLED(DELTA)
      #define Y_HOME_POS 0
    #else
      #define Y_HOME_POS ((Y_MAX_LENGTH) * (Y_HOME_DIR) * 0.5)
    #endif
  #else
    #if ENABLED(DELTA)
      #define Y_HOME_POS ((Y_MAX_LENGTH) * 0.5)
    #else
      #define Y_HOME_POS (Y_HOME_DIR < 0 ? Y_MIN_POS : Y_MAX_POS)
    #endif
  #endif

  #ifdef MANUAL_Z_HOME_POS
    #define Z_HOME_POS MANUAL_Z_HOME_POS
  #else
    #define Z_HOME_POS (Z_HOME_DIR < 0 ? Z_MIN_POS : Z_MAX_POS)
  #endif

  /**
   * The BLTouch Probe emulates a servo probe
   */
  #if ENABLED(BLTOUCH)
    #undef Z_ENDSTOP_SERVO_NR
    #undef Z_SERVO_ANGLES
    #define Z_ENDSTOP_SERVO_NR 0
    #define Z_SERVO_ANGLES {10,90} // For BLTouch 10=deploy, 90=retract
    #undef DEACTIVATE_SERVOS_AFTER_MOVE
    #if ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)
      #undef Z_MIN_ENDSTOP_INVERTING
      #define Z_MIN_ENDSTOP_INVERTING false
    #endif
  #endif

  /**
   * Auto Bed Leveling and Z Probe Repeatability Test
   */
  #define HAS_PROBING_PROCEDURE (ENABLED(AUTO_BED_LEVELING_FEATURE) || ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST))

  // Boundaries for probing based on set limits
  #define MIN_PROBE_X (max(X_MIN_POS, X_MIN_POS + X_PROBE_OFFSET_FROM_EXTRUDER))
  #define MAX_PROBE_X (min(X_MAX_POS, X_MAX_POS + X_PROBE_OFFSET_FROM_EXTRUDER))
  #define MIN_PROBE_Y (max(Y_MIN_POS, Y_MIN_POS + Y_PROBE_OFFSET_FROM_EXTRUDER))
  #define MAX_PROBE_Y (min(Y_MAX_POS, Y_MAX_POS + Y_PROBE_OFFSET_FROM_EXTRUDER))

  #define HAS_Z_SERVO_ENDSTOP (defined(Z_ENDSTOP_SERVO_NR) && Z_ENDSTOP_SERVO_NR >= 0)

  /**
   * Z Sled Probe requires Z_SAFE_HOMING
   */
  #if ENABLED(Z_PROBE_SLED)
    #define Z_SAFE_HOMING
  #endif

  /**
   * DELTA should ignore Z_SAFE_HOMING
   */
  #if ENABLED(DELTA)
    #undef Z_SAFE_HOMING
  #endif

  /**
   * Safe Homing Options
   */
  #if ENABLED(Z_SAFE_HOMING)
    #ifndef Z_SAFE_HOMING_X_POINT
      #define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)
    #endif
    #ifndef Z_SAFE_HOMING_Y_POINT
      #define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)
    #endif
  #endif

  /**
   * Host keep alive
   */
  #ifndef DEFAULT_KEEPALIVE_INTERVAL
    #define DEFAULT_KEEPALIVE_INTERVAL 2
  #endif
  #define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)
  // MS1 MS2 Stepper Driver Microstepping mode table
  #define MICROSTEP1 LOW,LOW
  #define MICROSTEP2 HIGH,LOW
  #define MICROSTEP4 LOW,HIGH
  #define MICROSTEP8 HIGH,HIGH
  #define MICROSTEP16 HIGH,HIGH
  #if ENABLED(ULTIPANEL) && DISABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    #undef SD_DETECT_INVERTED
  #endif

  /**
   * Set defaults for missing (newer) options
   */
  #ifndef DISABLE_INACTIVE_X
    #define DISABLE_INACTIVE_X DISABLE_X
  #endif
  #ifndef DISABLE_INACTIVE_Y
    #define DISABLE_INACTIVE_Y DISABLE_Y
  #endif
  #ifndef DISABLE_INACTIVE_Z
    #define DISABLE_INACTIVE_Z DISABLE_Z
  #endif
  #ifndef DISABLE_INACTIVE_E
    #define DISABLE_INACTIVE_E DISABLE_E
  #endif

  // Power Signal Control Definitions
  // By default use ATX definition
  #ifndef POWER_SUPPLY
    #define POWER_SUPPLY 1
  #endif
  #if (POWER_SUPPLY == 1)     // 1 = ATX
    #define PS_ON_AWAKE  LOW
    #define PS_ON_ASLEEP HIGH
  #elif (POWER_SUPPLY == 2)   // 2 = X-Box 360 203W
    #define PS_ON_AWAKE  HIGH
    #define PS_ON_ASLEEP LOW
  #endif
  #define HAS_POWER_SWITCH (POWER_SUPPLY > 0 && PIN_EXISTS(PS_ON))

  /**
   * Temp Sensor defines
   */
  #if TEMP_SENSOR_0 == 0
    #undef HEATER_0_MINTEMP
    #undef HEATER_0_MAXTEMP
  #elif TEMP_SENSOR_0 > 0
    #define THERMISTORHEATER_0 TEMP_SENSOR_0
    #define HEATER_0_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_1 == 0
    #undef HEATER_1_MINTEMP
    #undef HEATER_1_MAXTEMP
  #elif TEMP_SENSOR_1 > 0
    #define THERMISTORHEATER_1 TEMP_SENSOR_1
    #define HEATER_1_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_2 == 0
    #undef HEATER_2_MINTEMP
    #undef HEATER_2_MAXTEMP
  #elif TEMP_SENSOR_2 > 0
    #define THERMISTORHEATER_2 TEMP_SENSOR_2
    #define HEATER_2_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_3 == 0
    #undef HEATER_3_MINTEMP
    #undef HEATER_3_MAXTEMP
  #elif TEMP_SENSOR_3 > 0
    #define THERMISTORHEATER_3 TEMP_SENSOR_3
    #define HEATER_3_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_BED == 0
    #undef BED_MINTEMP
    #undef BED_MAXTEMP
  #elif TEMP_SENSOR_BED > 0
    #define THERMISTORBED TEMP_SENSOR_BED
    #define BED_USES_THERMISTOR
  #endif

  /**
   * Flags for PID handling
   */
  #define HAS_PID_HEATING (ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED))
  #define HAS_PID_FOR_BOTH (ENABLED(PIDTEMP) && ENABLED(PIDTEMPBED))

  /**
   * Default hotend offsets, if not defined
   */
  #if HOTENDS > 1
    #ifndef HOTEND_OFFSET_X
      #define HOTEND_OFFSET_X { 0 } // X offsets for each extruder
    #endif
    #ifndef HOTEND_OFFSET_Y
      #define HOTEND_OFFSET_Y { 0 } // Y offsets for each extruder
    #endif
  #endif

  /**
   * ARRAY_BY_EXTRUDERS based on EXTRUDERS
   */
  #define ARRAY_BY_EXTRUDERS(args...) ARRAY_N(EXTRUDERS, args)
  #define ARRAY_BY_EXTRUDERS1(v1) ARRAY_BY_EXTRUDERS(v1, v1, v1, v1, v1, v1)

  /**
   * ARRAY_BY_HOTENDS based on HOTENDS
   */
  #define ARRAY_BY_HOTENDS(args...) ARRAY_N(HOTENDS, args)
  #define ARRAY_BY_HOTENDS1(v1) ARRAY_BY_HOTENDS(v1, v1, v1, v1, v1, v1)
  /**
   * Shorthand for pin tests, used wherever needed
   */
  #define HAS_TEMP_0 (PIN_EXISTS(TEMP_0) && TEMP_SENSOR_0 != 0 && TEMP_SENSOR_0 > -2)
  #define HAS_TEMP_1 (PIN_EXISTS(TEMP_1) && TEMP_SENSOR_1 != 0 && TEMP_SENSOR_1 > -2)
  #define HAS_TEMP_2 (PIN_EXISTS(TEMP_2) && TEMP_SENSOR_2 != 0 && TEMP_SENSOR_2 > -2)
  #define HAS_TEMP_3 (PIN_EXISTS(TEMP_3) && TEMP_SENSOR_3 != 0 && TEMP_SENSOR_3 > -2)
  #define HAS_TEMP_BED (PIN_EXISTS(TEMP_BED) && TEMP_SENSOR_BED != 0 && TEMP_SENSOR_BED > -2)
  #define HAS_HEATER_0 (PIN_EXISTS(HEATER_0))
  #define HAS_HEATER_1 (PIN_EXISTS(HEATER_1))
  #define HAS_HEATER_2 (PIN_EXISTS(HEATER_2))
  #define HAS_HEATER_3 (PIN_EXISTS(HEATER_3))
  #define HAS_KILL (PIN_EXISTS(KILL))
  #define HAS_X_MIN (PIN_EXISTS(X_MIN))
  #define HAS_X_MAX (PIN_EXISTS(X_MAX))
  #define HAS_Y_MIN (PIN_EXISTS(Y_MIN))
  #define HAS_Y_MAX (PIN_EXISTS(Y_MAX))
  #define HAS_Z_MIN (PIN_EXISTS(Z_MIN))
  #define HAS_Z_MAX (PIN_EXISTS(Z_MAX))
  #define HAS_Z2_MIN (PIN_EXISTS(Z2_MIN))
  #define HAS_Z2_MAX (PIN_EXISTS(Z2_MAX))
  #define HAS_Z_MIN_PROBE_PIN (PIN_EXISTS(Z_MIN_PROBE))
  #define HAS_X_ENABLE (PIN_EXISTS(X_ENABLE))
  #define HAS_X2_ENABLE (PIN_EXISTS(X2_ENABLE))
  #define HAS_Y_ENABLE (PIN_EXISTS(Y_ENABLE))
  #define HAS_Y2_ENABLE (PIN_EXISTS(Y2_ENABLE))
  #define HAS_Z_ENABLE (PIN_EXISTS(Z_ENABLE))
  #define HAS_Z2_ENABLE (PIN_EXISTS(Z2_ENABLE))
  #define HAS_E0_ENABLE (PIN_EXISTS(E0_ENABLE))
  #define HAS_E1_ENABLE (PIN_EXISTS(E1_ENABLE))
  #define HAS_E2_ENABLE (PIN_EXISTS(E2_ENABLE))
  #define HAS_E3_ENABLE (PIN_EXISTS(E3_ENABLE))
  #define HAS_E4_ENABLE (PIN_EXISTS(E4_ENABLE))
  #define HAS_X_DIR (PIN_EXISTS(X_DIR))
  #define HAS_X2_DIR (PIN_EXISTS(X2_DIR))
  #define HAS_Y_DIR (PIN_EXISTS(Y_DIR))
  #define HAS_Y2_DIR (PIN_EXISTS(Y2_DIR))
  #define HAS_Z_DIR (PIN_EXISTS(Z_DIR))
  #define HAS_Z2_DIR (PIN_EXISTS(Z2_DIR))
  #define HAS_E0_DIR (PIN_EXISTS(E0_DIR))
  #define HAS_E1_DIR (PIN_EXISTS(E1_DIR))
  #define HAS_E2_DIR (PIN_EXISTS(E2_DIR))
  #define HAS_E3_DIR (PIN_EXISTS(E3_DIR))
  #define HAS_E4_DIR (PIN_EXISTS(E4_DIR))
  #define HAS_X_STEP (PIN_EXISTS(X_STEP))
  #define HAS_X2_STEP (PIN_EXISTS(X2_STEP))
  #define HAS_Y_STEP (PIN_EXISTS(Y_STEP))
  #define HAS_Y2_STEP (PIN_EXISTS(Y2_STEP))
  #define HAS_Z_STEP (PIN_EXISTS(Z_STEP))
  #define HAS_Z2_STEP (PIN_EXISTS(Z2_STEP))
  #define HAS_E0_STEP (PIN_EXISTS(E0_STEP))
  #define HAS_E1_STEP (PIN_EXISTS(E1_STEP))
  #define HAS_E2_STEP (PIN_EXISTS(E2_STEP))
  #define HAS_E3_STEP (PIN_EXISTS(E3_STEP))
  #define HAS_E4_STEP (PIN_EXISTS(E4_STEP))
  #define HAS_BUZZER (PIN_EXISTS(BEEPER))

  #define HAS_MOTOR_CURRENT_PWM (PIN_EXISTS(MOTOR_CURRENT_PWM_XY) || PIN_EXISTS(MOTOR_CURRENT_PWM_Z) || PIN_EXISTS(MOTOR_CURRENT_PWM_E))

  #define HAS_TEMP_HOTEND (HAS_TEMP_0)

  /**
   * This value is used by M109 when trying to calculate a ballpark safe margin
   * to prevent wait-forever situation.
   */
  #ifndef EXTRUDE_MINTEMP
   #define EXTRUDE_MINTEMP 170
  #endif

  /**
   * Helper Macros for heaters and extruder fan
   */
  #define WRITE_HEATER_0P(v) WRITE(HEATER_0_PIN, v)
  #if HOTENDS > 1 || ENABLED(HEATERS_PARALLEL)
    #define WRITE_HEATER_1(v) WRITE(HEATER_1_PIN, v)
    #if HOTENDS > 2
      #define WRITE_HEATER_2(v) WRITE(HEATER_2_PIN, v)
      #if HOTENDS > 3
        #define WRITE_HEATER_3(v) WRITE(HEATER_3_PIN, v)
      #endif
    #endif
  #endif
  #if ENABLED(HEATERS_PARALLEL)
    #define WRITE_HEATER_0(v) { WRITE_HEATER_0P(v); WRITE_HEATER_1(v); }
  #else
    #define WRITE_HEATER_0(v) WRITE_HEATER_0P(v)
  #endif
  #define WRITE_FAN_N(n, v) WRITE_FAN##n(v)
  #define PROBE_SELECTED (ENABLED(FIX_MOUNTED_PROBE) || ENABLED(Z_PROBE_ALLEN_KEY) || HAS_Z_SERVO_ENDSTOP || ENABLED(Z_PROBE_SLED))
  #define PROBE_PIN_CONFIGURED (HAS_Z_MIN_PROBE_PIN || (HAS_Z_MIN && ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)))
  #if ENABLED(Z_PROBE_ALLEN_KEY)
    #define PROBE_IS_TRIGGERED_WHEN_STOWED_TEST
  #endif
  #undef X_PROBE_OFFSET_FROM_EXTRUDER
  #undef Y_PROBE_OFFSET_FROM_EXTRUDER
  #undef Z_PROBE_OFFSET_FROM_EXTRUDER
  #define X_PROBE_OFFSET_FROM_EXTRUDER 0
  #define Y_PROBE_OFFSET_FROM_EXTRUDER 0
  #define Z_PROBE_OFFSET_FROM_EXTRUDER 0

  /**
   * Buzzer/Speaker
   */
  #if ENABLED(LCD_USE_I2C_BUZZER)
    #ifndef LCD_FEEDBACK_FREQUENCY_HZ
      #define LCD_FEEDBACK_FREQUENCY_HZ 1000
    #endif
    #ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
      #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
    #endif
  #elif PIN_EXISTS(BEEPER)
    #ifndef LCD_FEEDBACK_FREQUENCY_HZ
      #define LCD_FEEDBACK_FREQUENCY_HZ 5000
    #endif
    #ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
      #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
    #endif
  #else
    #ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
      #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
    #endif
  #endif

  /**
   * Z_HOMING_HEIGHT / Z_PROBE_TRAVEL_HEIGHT
   */
  #ifndef Z_HOMING_HEIGHT
    #ifndef Z_PROBE_TRAVEL_HEIGHT
      #define Z_HOMING_HEIGHT 0
    #else
      #define Z_HOMING_HEIGHT Z_PROBE_TRAVEL_HEIGHT
    #endif
  #endif
  #ifndef Z_PROBE_TRAVEL_HEIGHT
    #define Z_PROBE_TRAVEL_HEIGHT Z_HOMING_HEIGHT
  #endif

#endif // CONDITIONALS_POST_H
