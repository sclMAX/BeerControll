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
 * Configuration_adv.h
 *
 * Advanced settings.
 * Only change these if you know exactly what you're doing.
 * Some of these settings can damage your printer if improperly set!
 *
 * Basic settings can be found in Configuration.h
 *
 */
#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

/**
 *
 *  ***********************************
 *  **  ATTENTION TO ALL DEVELOPERS  **
 *  ***********************************
 *
 * You must increment this version number for every significant change such as,
 * but not limited to: ADD, DELETE RENAME OR REPURPOSE any directive/option.
 *
 * Note: Update also Version.h !
 */
#define CONFIGURATION_ADV_H_VERSION 010100

// @section temperature

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

#if DISABLED(PIDTEMPBED)
  #define BED_CHECK_INTERVAL 5000 // ms between checks in bang-bang control
  #if ENABLED(BED_LIMIT_SWITCHING)
    #define BED_HYSTERESIS 2 // Only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS
  #endif
#endif

#if ENABLED(PIDTEMP)
  // this adds an experimental additional term to the heating power, proportional to the extrusion speed.
  // if Kc is chosen well, the additional required power due to increased melting should be compensated.
  //#define PID_EXTRUSION_SCALING
  #if ENABLED(PID_EXTRUSION_SCALING)
    #define DEFAULT_Kc (100) //heating power=Kc*(e_speed)
    #define LPQ_MAX_LEN 50
  #endif
#endif

/**
 * Automatic Temperature:
 * The hotend target temperature is calculated by all the buffered lines of gcode.
 * The maximum buffered steps/sec of the extruder motor is called "se".
 * Start autotemp mode with M109 S<mintemp> B<maxtemp> F<factor>
 * The target temperature is set to mintemp+factor*se[steps/sec] and is limited by
 * mintemp and maxtemp. Turn this off by executing M109 without F*
 * Also, if the temperature is set to a value below mintemp, it will not be changed by autotemp.
 * On an Ultimaker, some initial testing worked with M109 S215 B260 F1 in the start.gcode
 */
//#define AUTOTEMP
#if ENABLED(AUTOTEMP)
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif

//Show Temperature ADC value
//The M105 command return, besides traditional information, the ADC value read from temperature sensors.
//#define SHOW_TEMP_ADC_VALUES

/**
 * High Temperature Thermistor Support
 *
 * Thermistors able to support high temperature tend to have a hard time getting
 * good readings at room and lower temperatures. This means HEATER_X_RAW_LO_TEMP
 * will probably be caught when the heating element first turns on during the
 * preheating process, which will trigger a min_temp_error as a safety measure
 * and force stop everything.
 * To circumvent this limitation, we allow for a preheat time (during which,
 * min_temp_error won't be triggered) and add a min_temp buffer to handle
 * aberrant readings.
 *
 * If you want to enable this feature for your hotend thermistor(s)
 * uncomment and set values > 0 in the constants below
 */

// The number of consecutive low temperature errors that can occur
// before a min_temp_error is triggered. (Shouldn't be more than 10.)
//#define MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED 0

// The number of milliseconds a hotend will preheat before starting to check
// the temperature. This value should NOT be set to the time it takes the
// hot end to reach the target temperature, but the time it takes to reach
// the minimum temperature your thermistor can read. The lower the better/safer.
// This shouldn't need to be more than 30 seconds (30000)
//#define MILLISECONDS_PREHEAT_TIME 0

// @section extruder

//  extruder run-out prevention.
//if the machine is idle, and the temperature over MINTEMP, every couple of SECONDS some filament is extruded
//#define EXTRUDER_RUNOUT_PREVENT
#define EXTRUDER_RUNOUT_MINTEMP 190
#define EXTRUDER_RUNOUT_SECONDS 30
#define EXTRUDER_RUNOUT_ESTEPS 14   // mm filament
#define EXTRUDER_RUNOUT_SPEED 1500  // extrusion speed
#define EXTRUDER_RUNOUT_EXTRUDE 100

// @section temperature

//These defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

//This is for controlling a fan to cool down the stepper drivers
//it will turn on when any driver is enabled
//and turn off after the set amount of seconds from last driver being disabled again
#define CONTROLLERFAN_PIN -1 //Pin used for the fan to cool controller (-1 to disable)
#define CONTROLLERFAN_SECS 60 //How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED 255  // == full speed

// When first starting the main fan, run it at full speed for the
// given number of milliseconds.  This gets the fan spinning reliably
// before setting a PWM value. (Does not work with software PWM for fan on Sanguinololu)
//#define FAN_KICKSTART_TIME 100

// This defines the minimal speed for the main fan, run in PWM mode
// to enable uncomment and set minimal PWM speed for reliable running (1-255)
// if fan speed is [1 - (FAN_MIN_PWM-1)] it is set to FAN_MIN_PWM
//#define FAN_MIN_PWM 50

// @section extruder

// Extruder cooling fans
// Configure fan pin outputs to automatically turn on/off when the associated
// extruder temperature is above/below EXTRUDER_AUTO_FAN_TEMPERATURE.
// Multiple extruders can be assigned to the same pin in which case
// the fan will turn on when any selected extruder is above the threshold.
#define EXTRUDER_0_AUTO_FAN_PIN -1
#define EXTRUDER_1_AUTO_FAN_PIN -1
#define EXTRUDER_2_AUTO_FAN_PIN -1
#define EXTRUDER_3_AUTO_FAN_PIN -1
#define EXTRUDER_AUTO_FAN_TEMPERATURE 50
#define EXTRUDER_AUTO_FAN_SPEED   255  // == full speed

//===========================================================================
//============================ Mechanical Settings ==========================
//===========================================================================

// @section homing

// If you want endstops to stay on (by default) even when not homing
// enable this option. Override at any time with M120, M121.
//#define ENDSTOPS_ALWAYS_ON_DEFAULT

// @section extras

//#define Z_LATE_ENABLE // Enable Z the last moment. Needed if your Z driver overheats.

// Dual X Steppers
// Uncomment this option to drive two X axis motors.
// The next unused E driver will be assigned to the second X stepper.
//#define X_DUAL_STEPPER_DRIVERS
#if ENABLED(X_DUAL_STEPPER_DRIVERS)
  // Set true if the two X motors need to rotate in opposite directions
  #define INVERT_X2_VS_X_DIR true
#endif


// Dual Y Steppers
// Uncomment this option to drive two Y axis motors.
// The next unused E driver will be assigned to the second Y stepper.
//#define Y_DUAL_STEPPER_DRIVERS
#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
  // Set true if the two Y motors need to rotate in opposite directions
  #define INVERT_Y2_VS_Y_DIR true
#endif

// A single Z stepper driver is usually used to drive 2 stepper motors.
// Uncomment this option to use a separate stepper driver for each Z axis motor.
// The next unused E driver will be assigned to the second Z stepper.
//#define Z_DUAL_STEPPER_DRIVERS

#if ENABLED(Z_DUAL_STEPPER_DRIVERS)


#endif // Z_DUAL_STEPPER_DRIVERS
// @section homing

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2
#define HOMING_BUMP_DIVISOR {2, 2, 4}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)
//#define QUICK_HOME  //if this is defined, if both x and y are to be homed, a diagonal move will be performed initially.

// When G28 is called, this option will make Y home before X
//#define HOME_Y_BEFORE_X

// @section machine

#define AXIS_RELATIVE_MODES {false, false, false, false}

// Allow duplication mode with a basic dual-nozzle extruder
//#define DUAL_NOZZLE_DUPLICATION_MODE

// By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

// Default stepper release if idle. Set to 0 to deactivate.
// Steppers will shut down DEFAULT_STEPPER_DEACTIVE_TIME seconds after the last move when DISABLE_INACTIVE_? is true.
// Time can be set by M18 and M84.
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X true
#define DISABLE_INACTIVE_Y true
#define DISABLE_INACTIVE_Z true  // set to false if the nozzle will fall down on your printed part when print has finished.
#define DISABLE_INACTIVE_E true

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// @section lcd

#if ENABLED(ULTIPANEL)
  #define MANUAL_FEEDRATE {50*60, 50*60, 4*60, 60} // Feedrates for manual moves along X, Y, Z, E from panel
  #define ULTIPANEL_FEEDMULTIPLY  // Comment to disable setting feedrate multiplier via encoder
#endif

// @section extras

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN

// Frequency limit
// See nophead's blog for more info
// Not working O
//#define XY_FREQUENCY_LIMIT  15

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
#define DIGIPOT_MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)

// Motor Current controlled via PWM (Overridable on supported boards with PWM-driven motor driver current)
//#define PWM_MOTOR_CURRENT {1300, 1300, 1250} // Values in milliamps

// uncomment to enable an I2C based DIGIPOT like on the Azteeg X3 Pro
//#define DIGIPOT_I2C
// Number of channels available for I2C digipot, For Azteeg X3 Pro we have 8
#define DIGIPOT_I2C_NUM_CHANNELS 8
// actual motor currents in Amps, need as many here as DIGIPOT_I2C_NUM_CHANNELS
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value

//#define CHDK 4        //Pin for triggering CHDK to take a picture see how to use it here http://captain-slow.dk/2014/03/09/3d-printing-timelapses/
#define CHDK_DELAY 50 //How long in ms the pin should stay HIGH before going LOW again


// @section safety

// The hardware watchdog should reset the microcontroller disabling all outputs,
// in case the firmware gets stuck and doesn't do temperature regulation.
#define USE_WATCHDOG

#if ENABLED(USE_WATCHDOG)
  // If you have a watchdog reboot in an ArduinoMega2560 then the device will hang forever, as a watchdog reset will leave the watchdog on.
  // The "WATCHDOG_RESET_MANUAL" goes around this by not using the hardware reset.
  //  However, THIS FEATURE IS UNSAFE!, as it will only work if interrupts are disabled. And the code could hang in an interrupt routine with interrupts disabled.
  //#define WATCHDOG_RESET_MANUAL
#endif

const unsigned int dropsegments = 5; //everything with less than this number of steps will be ignored as move and joined with the next movement

// @section temperature

// Control heater 0 and heater 1 in parallel.
//#define HEATERS_PARALLEL

//===========================================================================
//================================= Buffers =================================
//===========================================================================

// @section hidden

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ring-buffering.
#if ENABLED(SDSUPPORT)
  #define BLOCK_BUFFER_SIZE 16   // SD,LCD,Buttons take more memory, block buffer needs to be smaller
#else
  #define BLOCK_BUFFER_SIZE 16 // maximize block buffer
#endif

// @section serial

// The ASCII buffer for serial input
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

// Transfer Buffer Size
// To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.
// To buffer a simple "ok" you need 4 bytes.
// For ADVANCED_OK (M105) you need 32 bytes.
// For debug-echo: 128 bytes for the optimal speed.
// Other output doesn't need to be that speedy.
// :[0,2,4,8,16,32,64,128,256]
#define TX_BUFFER_SIZE 0

// Enable an emergency-command parser to intercept certain commands as they
// enter the serial receive buffer, so they cannot be blocked.
// Currently handles M108, M112, M410
// Does not work on boards using AT90USB (USBCON) processors!
//#define EMERGENCY_PARSER

// Bad Serial-connections can miss a received command by sending an 'ok'
// Therefore some clients abort after 30 seconds in a timeout.
// Some other clients start sending commands while receiving a 'wait'.
// This "wait" is only sent when the buffer is empty. 1 second is a good value here.
//#define NO_TIMEOUTS 1000 // Milliseconds

// Some clients will have this feature soon. This could make the NO_TIMEOUTS unnecessary.
//#define ADVANCED_OK

/******************************************************************************\
 * enable this section if you have TMC26X motor drivers.
 * you need to import the TMC26XStepper library into the Arduino IDE for this
 ******************************************************************************/

// @section tmc

//#define HAVE_TMCDRIVER
#if ENABLED(HAVE_TMCDRIVER)

  //#define X_IS_TMC
  #define X_MAX_CURRENT 1000  //in mA
  #define X_SENSE_RESISTOR 91 //in mOhms
  #define X_MICROSTEPS 16     //number of microsteps

  //#define X2_IS_TMC
  #define X2_MAX_CURRENT 1000  //in mA
  #define X2_SENSE_RESISTOR 91 //in mOhms
  #define X2_MICROSTEPS 16     //number of microsteps

  //#define Y_IS_TMC
  #define Y_MAX_CURRENT 1000  //in mA
  #define Y_SENSE_RESISTOR 91 //in mOhms
  #define Y_MICROSTEPS 16     //number of microsteps

  //#define Y2_IS_TMC
  #define Y2_MAX_CURRENT 1000  //in mA
  #define Y2_SENSE_RESISTOR 91 //in mOhms
  #define Y2_MICROSTEPS 16     //number of microsteps

  //#define Z_IS_TMC
  #define Z_MAX_CURRENT 1000  //in mA
  #define Z_SENSE_RESISTOR 91 //in mOhms
  #define Z_MICROSTEPS 16     //number of microsteps

  //#define Z2_IS_TMC
  #define Z2_MAX_CURRENT 1000  //in mA
  #define Z2_SENSE_RESISTOR 91 //in mOhms
  #define Z2_MICROSTEPS 16     //number of microsteps

  //#define E0_IS_TMC
  #define E0_MAX_CURRENT 1000  //in mA
  #define E0_SENSE_RESISTOR 91 //in mOhms
  #define E0_MICROSTEPS 16     //number of microsteps

  //#define E1_IS_TMC
  #define E1_MAX_CURRENT 1000  //in mA
  #define E1_SENSE_RESISTOR 91 //in mOhms
  #define E1_MICROSTEPS 16     //number of microsteps

  //#define E2_IS_TMC
  #define E2_MAX_CURRENT 1000  //in mA
  #define E2_SENSE_RESISTOR 91 //in mOhms
  #define E2_MICROSTEPS 16     //number of microsteps

  //#define E3_IS_TMC
  #define E3_MAX_CURRENT 1000  //in mA
  #define E3_SENSE_RESISTOR 91 //in mOhms
  #define E3_MICROSTEPS 16     //number of microsteps

#endif

/******************************************************************************\
 * enable this section if you have L6470  motor drivers.
 * you need to import the L6470 library into the Arduino IDE for this
 ******************************************************************************/

// @section l6470

//#define HAVE_L6470DRIVER
#if ENABLED(HAVE_L6470DRIVER)

  //#define X_IS_L6470
  #define X_MICROSTEPS 16     //number of microsteps
  #define X_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define X_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define X_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define X2_IS_L6470
  #define X2_MICROSTEPS 16     //number of microsteps
  #define X2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define X2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define X2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define Y_IS_L6470
  #define Y_MICROSTEPS 16     //number of microsteps
  #define Y_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define Y_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Y_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define Y2_IS_L6470
  #define Y2_MICROSTEPS 16     //number of microsteps
  #define Y2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define Y2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Y2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define Z_IS_L6470
  #define Z_MICROSTEPS 16     //number of microsteps
  #define Z_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define Z_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Z_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define Z2_IS_L6470
  #define Z2_MICROSTEPS 16     //number of microsteps
  #define Z2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define Z2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Z2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define E0_IS_L6470
  #define E0_MICROSTEPS 16     //number of microsteps
  #define E0_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define E0_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define E0_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define E1_IS_L6470
  #define E1_MICROSTEPS 16     //number of microsteps
  #define E1_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define E1_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define E1_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define E2_IS_L6470
  #define E2_MICROSTEPS 16     //number of microsteps
  #define E2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define E2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define E2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define E3_IS_L6470
  #define E3_MICROSTEPS 16     //number of microsteps
  #define E3_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define E3_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define E3_STALLCURRENT 1500 //current in mA where the driver will detect a stall

#endif

/**
 * TWI/I2C BUS
 *
 * This feature is an EXPERIMENTAL feature so it shall not be used on production
 * machines. Enabling this will allow you to send and receive I2C data from slave
 * devices on the bus.
 *
 * ; Example #1
 * ; This macro send the string "Marlin" to the slave device with address 0x63 (99)
 * ; It uses multiple M155 commands with one B<base 10> arg
 * M155 A99  ; Target slave address
 * M155 B77  ; M
 * M155 B97  ; a
 * M155 B114 ; r
 * M155 B108 ; l
 * M155 B105 ; i
 * M155 B110 ; n
 * M155 S1   ; Send the current buffer
 *
 * ; Example #2
 * ; Request 6 bytes from slave device with address 0x63 (99)
 * M156 A99 B5
 *
 * ; Example #3
 * ; Example serial output of a M156 request
 * echo:i2c-reply: from:99 bytes:5 data:hello
 */

// @section i2cbus

//#define EXPERIMENTAL_I2CBUS

#endif // CONFIGURATION_ADV_H
