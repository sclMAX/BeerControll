/**
 * Conditionals_post.h
 * Defines that depend on configuration but are not editable.
 */

#ifndef CONDITIONALS_POST_H
#define CONDITIONALS_POST_H
  #define EMERGENCY_PARSER_CAPABILITIES ""
  /**
   * Auto Bed Leveling and Z Probe Repeatability Test
   */
  #define HAS_PROBING_PROCEDURE (ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST))
    /**
   * Host keep alive
   */
  #ifndef DEFAULT_KEEPALIVE_INTERVAL
    #define DEFAULT_KEEPALIVE_INTERVAL 2
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
  #if TEMP_SENSOR_0 == -3
    #define HEATER_0_USES_MAX6675
    #define MAX6675_IS_MAX31855
  #elif TEMP_SENSOR_0 == -2
    #define HEATER_0_USES_MAX6675
  #elif TEMP_SENSOR_0 == -1
    #define HEATER_0_USES_AD595
  #elif TEMP_SENSOR_0 == 0
    #undef HEATER_0_MINTEMP
    #undef HEATER_0_MAXTEMP
  #elif TEMP_SENSOR_0 > 0
    #define THERMISTORHEATER_0 TEMP_SENSOR_0
    #define HEATER_0_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_1 <= -2
    #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_1"
  #elif TEMP_SENSOR_1 == -1
    #define HEATER_1_USES_AD595
  #elif TEMP_SENSOR_1 == 0
    #undef HEATER_1_MINTEMP
    #undef HEATER_1_MAXTEMP
  #elif TEMP_SENSOR_1 > 0
    #define THERMISTORHEATER_1 TEMP_SENSOR_1
    #define HEATER_1_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_2 <= -2
    #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_2"
  #elif TEMP_SENSOR_2 == -1
    #define HEATER_2_USES_AD595
  #elif TEMP_SENSOR_2 == 0
    #undef HEATER_2_MINTEMP
    #undef HEATER_2_MAXTEMP
  #elif TEMP_SENSOR_2 > 0
    #define THERMISTORHEATER_2 TEMP_SENSOR_2
    #define HEATER_2_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_3 <= -2
    #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_3"
  #elif TEMP_SENSOR_3 == -1
    #define HEATER_3_USES_AD595
  #elif TEMP_SENSOR_3 == 0
    #undef HEATER_3_MINTEMP
    #undef HEATER_3_MAXTEMP
  #elif TEMP_SENSOR_3 > 0
    #define THERMISTORHEATER_3 TEMP_SENSOR_3
    #define HEATER_3_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_BED <= -2
    #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_BED"
  #elif TEMP_SENSOR_BED == -1
    #define BED_USES_AD595
  #elif TEMP_SENSOR_BED == 0
    #undef BED_MINTEMP
    #undef BED_MAXTEMP
  #elif TEMP_SENSOR_BED > 0
    #define THERMISTORBED TEMP_SENSOR_BED
    #define BED_USES_THERMISTOR
  #endif

  /**
   * Flags for PID handling
   */
  #define HAS_PID_HEATING (ENABLED(PIDTEMP))
  #define HAS_PID_FOR_BOTH (false)

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
  #define HAS_HEATER_BED (PIN_EXISTS(HEATER_BED))
  #define HAS_SUICIDE (PIN_EXISTS(SUICIDE))
  #define HAS_DIGIPOTSS (PIN_EXISTS(DIGIPOTSS))
  #define HAS_BUZZER (PIN_EXISTS(BEEPER))
  #define HAS_TEMP_HOTEND (HAS_TEMP_0 || ENABLED(HEATER_0_USES_MAX6675))

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
  #if HOTENDS > 1 
    #define WRITE_HEATER_1(v) WRITE(HEATER_1_PIN, v)
    #if HOTENDS > 2
      #define WRITE_HEATER_2(v) WRITE(HEATER_2_PIN, v)
      #if HOTENDS > 3
        #define WRITE_HEATER_3(v) WRITE(HEATER_3_PIN, v)
      #endif
    #endif
  #endif
  #define WRITE_HEATER_0(v) WRITE_HEATER_0P(v)
  #if HAS_HEATER_BED
    #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN, v)
  #endif
  
  /**
   * Buzzer/Speaker
   */
  #if PIN_EXISTS(BEEPER)
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
#endif // CONDITIONALS_POST_H
