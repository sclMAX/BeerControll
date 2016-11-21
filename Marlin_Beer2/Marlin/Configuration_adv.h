#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H
  #define CONFIGURATION_ADV_H_VERSION 010100
  #if DISABLED(PIDTEMPBED)
    #define BED_CHECK_INTERVAL 5000 // ms between checks in bang-bang control
  #endif

  #if ENABLED(PIDTEMP)
    // @section temperature

    //These defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
    //The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
    #define TEMP_SENSOR_AD595_OFFSET 0.0
    #define TEMP_SENSOR_AD595_GAIN   1.0
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
    // @section homing

    //homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
    #define X_HOME_BUMP_MM 5
    #define Y_HOME_BUMP_MM 5
    #define Z_HOME_BUMP_MM 2
    #define HOMING_BUMP_DIVISOR {2, 2, 4}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)
    //#define QUICK_HOME  //if this is defined, if both x and y are to be homed, a diagonal move will be performed initially.
    #define AXIS_RELATIVE_MODES {false, false, false, false}
    // By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
    #define INVERT_X_STEP_PIN false
    #define INVERT_Y_STEP_PIN false
    #define INVERT_Z_STEP_PIN false
    #define INVERT_E_STEP_PIN false
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
    // Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
    #define DIGIPOT_MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)

    // Motor Current controlled via PWM (Overridable on supported boards with PWM-driven motor driver current)
    //#define PWM_MOTOR_CURRENT {1300, 1300, 1250} // Values in milliamps
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
    const unsigned int dropsegments = 5; //everything with less than this number of steps will be ignored as move and joined with the next movement
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
    // For debug-echo: 128 bytes for the optimal speed.
    // Other output doesn't need to be that speedy.
    // :[0,2,4,8,16,32,64,128,256]
    #define TX_BUFFER_SIZE 0

    // Enable an emergency-command parser to intercept certain commands as they
    // enter the serial receive buffer, so they cannot be blocked.
    // Currently handles M108, M112, M410
    // Does not work on boards using AT90USB (USBCON) processors!
    //#define EMERGENCY_PARSER
    #endif

#endif // CONFIGURATION_ADV_H
