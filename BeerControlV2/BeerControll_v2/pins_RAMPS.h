#ifndef BOARD_NAME
  #define BOARD_NAME "RAMPS 1.4"
#endif

#define LARGE_FLASH true
#define SERVO0_PIN       11
#define SERVO1_PIN          6
#define SERVO2_PIN          5
#define SERVO3_PIN          4
#define X_MIN_PIN           3
#ifndef X_MAX_PIN
  #define X_MAX_PIN         2
#endif
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  32
#endif

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62

#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

#define SDSS               53
#define LED_PIN            13

// Use the RAMPS 1.4 Analog input 5 on the AUX2 connector
#define FILWIDTH_PIN        5 // ANALOG NUMBERING

// define digital pin 4 for the filament runout sensor. Use the RAMPS 1.4 digital input 4 on the servos connector
#define FIL_RUNOUT_PIN      4

#define PS_ON_PIN          12
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_1_PIN         15   // ANALOG NUMBERING
#define TEMP_BED_PIN       14   // ANALOG NUMBERING
/**
 * Hi Voltage PWM Pin Assignments
 */

#ifndef MOSFET_D_PIN
  #define MOSFET_D_PIN  -1
#endif
#ifndef RAMPS_D8_PIN
  #define RAMPS_D8_PIN   8
#endif
#ifndef RAMPS_D9_PIN
  #define RAMPS_D9_PIN   9
#endif
#ifndef RAMPS_D10_PIN
  #define RAMPS_D10_PIN 10
#endif

#define HEATER_0_PIN    RAMPS_D10_PIN // Hotend, Hotend, Bed
#define HEATER_1_PIN    RAMPS_D9_PIN
#define HEATER_BED_PIN  RAMPS_D8_PIN

#ifndef FAN_PIN
  #define FAN_PIN 4      // IO pin. Buffer needed
#endif

/**
 * LCD Controller Pin Assignments
 */
#define LCD_PINS_RS 16
#define LCD_PINS_ENABLE 17
#define LCD_PINS_D4 23
#define LCD_PINS_D5 25
#define LCD_PINS_D6 27
#define LCD_PINS_D7 29
#define BEEPER_PIN 37
#define BTN_EN1 31
#define BTN_EN2 33
#define BTN_ENC 35
#define SD_DETECT_PIN 49
#define KILL_PIN 41
#define MAX6675_SS       66 // Do not use pin 53 if there is even the remote possibility of using Display/SD card

