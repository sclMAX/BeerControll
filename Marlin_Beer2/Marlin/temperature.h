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
 * temperature.h - temperature controller
 */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H
#include "thermistortables.h"
#include "MarlinConfig.h"
#ifndef SOFT_PWM_SCALE
  #define SOFT_PWM_SCALE 0
#endif

#if HOTENDS == 1
  #define HOTEND_LOOP() const int8_t e = 0;
  #define HOTEND_INDEX  0
  #define EXTRUDER_IDX  0
#else
  #define HOTEND_LOOP() for (int8_t e = 0; e < HOTENDS; e++)
  #define HOTEND_INDEX  e
  #define EXTRUDER_IDX  active_extruder
#endif

class Temperature {

  public:

    static float current_temperature[HOTENDS],
                 current_temperature_bed;
    static int   current_temperature_raw[HOTENDS],
                 target_temperature[HOTENDS],
                 current_temperature_bed_raw,
                 target_temperature_bed;

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static float redundant_temperature;
    #endif

    static unsigned char soft_pwm_bed;

    #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED)
      #define PID_dT ((OVERSAMPLENR * 12.0)/(F_CPU / 64.0 / 256.0))
    #endif

    #if ENABLED(PIDTEMP)

      #if ENABLED(PID_PARAMS_PER_HOTEND) && HOTENDS > 1

        static float Kp[HOTENDS], Ki[HOTENDS], Kd[HOTENDS];
        #if ENABLED()
          static float Kc[HOTENDS];
        #endif
        #define PID_PARAM(param, h) Temperature::param[h]

      #else

        static float Kp, Ki, Kd;
        #if ENABLED()
          static float Kc;
        #endif
        #define PID_PARAM(param, h) Temperature::param

      #endif // PID_PARAMS_PER_HOTEND

      // Apply the scale factors to the PID values
      #define scalePID_i(i)   ( (i) * PID_dT )
      #define unscalePID_i(i) ( (i) / PID_dT )
      #define scalePID_d(d)   ( (d) / PID_dT )
      #define unscalePID_d(d) ( (d) * PID_dT )

    #endif

    #if ENABLED(PIDTEMPBED)
      static float bedKp, bedKi, bedKd;
    #endif
    static bool tooColdToExtrude(uint8_t e) { UNUSED(e); return false; }
   

  private:

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static int redundant_temperature_raw;
      static float redundant_temperature;
    #endif

    static volatile bool temp_meas_ready;

    #if ENABLED(PIDTEMP)
      static float temp_iState[HOTENDS],
                   temp_dState[HOTENDS],
                   pTerm[HOTENDS],
                   iTerm[HOTENDS],
                   dTerm[HOTENDS];

     static float pid_error[HOTENDS],
                   temp_iState_min[HOTENDS],
                   temp_iState_max[HOTENDS];
      static bool pid_reset[HOTENDS];
    #endif

    #if ENABLED(PIDTEMPBED)
      static float temp_iState_bed,
                   temp_dState_bed,
                   pTerm_bed,
                   iTerm_bed,
                   dTerm_bed,
                   pid_error_bed,
                   temp_iState_min_bed,
                   temp_iState_max_bed;
    #else
      static millis_t next_bed_check_ms;
    #endif

    static unsigned long raw_temp_value[4],
                         raw_temp_bed_value;

    // Init min and max temp with extreme values to prevent false errors during startup
    static int minttemp_raw[HOTENDS],
               maxttemp_raw[HOTENDS],
               minttemp[HOTENDS],
               maxttemp[HOTENDS];

    #ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
      static int consecutive_low_temperature_error[HOTENDS];
    #endif

    #ifdef MILLISECONDS_PREHEAT_TIME
      static unsigned long preheat_end_time[HOTENDS];
    #endif

    #ifdef BED_MINTEMP
      static int bed_minttemp_raw;
    #endif

    #ifdef BED_MAXTEMP
      static int bed_maxttemp_raw;
    #endif
    static unsigned char soft_pwm[HOTENDS];

  public:

    /**
     * Instance Methods
     */

    Temperature();

    void init();

    /**
     * Static (class) methods
     */
    static float analog2temp(int raw, uint8_t e);
    static float analog2tempBed(int raw);

    /**
     * Called from the Temperature ISR
     */
    static void isr();

    /**
     * Call periodically to manage heaters
     */
    static void manage_heater();

    /**
     * Preheating hotends
     */
    #ifdef MILLISECONDS_PREHEAT_TIME
      static bool is_preheating(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        return preheat_end_time[HOTEND_INDEX] && PENDING(millis(), preheat_end_time[HOTEND_INDEX]);
      }
      static void start_preheat_time(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        preheat_end_time[HOTEND_INDEX] = millis() + MILLISECONDS_PREHEAT_TIME;
      }
      static void reset_preheat_time(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        preheat_end_time[HOTEND_INDEX] = 0;
      }
    #else
      #define is_preheating(n) (false)
    #endif

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static float analog2widthFil(); // Convert raw Filament Width to millimeters
      static int widthFil_to_size_ratio(); // Convert raw Filament Width to an extrusion ratio
    #endif


    //high level conversion routines, for use outside of temperature.cpp
    //inline so that there is no performance decrease.
    //deg=degreeCelsius

    static float degHotend(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return current_temperature[HOTEND_INDEX];
    }
    static float degBed() { return current_temperature_bed; }

    #if ENABLED(SHOW_TEMP_ADC_VALUES)
    static float rawHotendTemp(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return current_temperature_raw[HOTEND_INDEX];
    }
    static float rawBedTemp() { return current_temperature_bed_raw; }
    #endif

    static float degTargetHotend(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return target_temperature[HOTEND_INDEX];
    }
    static float degTargetBed() { return target_temperature_bed; }
    static void setTargetHotend(const float& celsius, uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      #ifdef MILLISECONDS_PREHEAT_TIME
        if (celsius == 0.0f)
          reset_preheat_time(HOTEND_INDEX);
        else if (target_temperature[HOTEND_INDEX] == 0.0f)
          start_preheat_time(HOTEND_INDEX);
      #endif
      target_temperature[HOTEND_INDEX] = celsius;
    }

    static void setTargetBed(const float& celsius) {
      target_temperature_bed = celsius;
    }

    static bool isHeatingHotend(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return target_temperature[HOTEND_INDEX] > current_temperature[HOTEND_INDEX];
    }
    static bool isHeatingBed() { return target_temperature_bed > current_temperature_bed; }

    static bool isCoolingHotend(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return target_temperature[HOTEND_INDEX] < current_temperature[HOTEND_INDEX];
    }
    static bool isCoolingBed() { return target_temperature_bed < current_temperature_bed; }

    /**
     * The software PWM power for a heater
     */
    static int getHeaterPower(int heater);

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

    /**
     * Perform auto-tuning for hotend or bed in response to M303
     */
    #if HAS_PID_HEATING
      static void PID_autotune(float temp, int hotend, int ncycles, bool set_result=false);
    #endif

    /**
     * Update the temp manager when PID values change
     */
    static void updatePID();
  private:

    static void set_current_temp_raw();

    static void updateTemperaturesFromRawValues();
    static void checkExtruderAutoFans();
    static float get_pid_output(int e);
    #if ENABLED(PIDTEMPBED)
      static float get_pid_output_bed();
    #endif
    static void _temp_error(int e, const char* serial_msg, const char* lcd_msg);
    static void min_temp_error(uint8_t e);
    static void max_temp_error(uint8_t e);
};

extern Temperature thermalManager;

#endif // TEMPERATURE_H
