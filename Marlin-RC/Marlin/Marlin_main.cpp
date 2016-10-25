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
 *
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 *  - http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"
#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "endstops.h"
#include "temperature.h"
#include "cardreader.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "nozzle.h"
#include "duration_t.h"
#include "types.h"

#if ENABLED(USE_WATCHDOG)
  #include "watchdog.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

#if ENABLED(DAC_STEPPER_CURRENT)
  #include "stepper_dac.h"
#endif
/**
 * Look here for descriptions of G-codes:
 *  - http://linuxcnc.org/handbook/gcode/g-code.html
 *  - http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 *
 * Help us document these G-codes online:
 *  - https://github.com/MarlinFirmware/Marlin/wiki/G-Code-in-Marlin
 *  - http://reprap.org/wiki/G-code
 *
 * -----------------
 * Implemented Codes
 * -----------------
 *
 * "G" Codes
 *
 * G0  -> G1
 * G1  - Coordinated Movement X Y Z E
 * G2  - CW ARC
 * G3  - CCW ARC
 * G4  - Dwell S<seconds> or P<milliseconds>
 * G5  - Cubic B-spline with XYZE destination and IJPQ offsets
 * G10 - Retract filament according to settings of M207
 * G11 - Retract recover filament according to settings of M208
 * G12 - Clean tool
 * G20 - Set input units to inches
 * G21 - Set input units to millimeters
 * G28 - Home one or more axes
 * G29 - Detailed Z probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
 * G30 - Single Z probe, probes bed at current XY location.
 * G31 - Dock sled (Z_PROBE_SLED only)
 * G32 - Undock sled (Z_PROBE_SLED only)
 * G90 - Use Absolute Coordinates
 * G91 - Use Relative Coordinates
 * G92 - Set current position to coordinates given
 *
 * "M" Codes
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 * M1   - Same as M0
 * M17  - Enable/Power all stepper motors
 * M18  - Disable all stepper motors; same as M84
 * M20  - List SD card
 * M21  - Init SD card
 * M22  - Release SD card
 * M23  - Select SD file (M23 filename.g)
 * M24  - Start/resume SD print
 * M25  - Pause SD print
 * M26  - Set SD position in bytes (M26 S12345)
 * M27  - Report SD print status
 * M28  - Start SD write (M28 filename.g)
 * M29  - Stop SD write
 * M30  - Delete file from SD (M30 filename.g)
 * M31  - Output time since last M109 or SD card start to serial
 * M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
 *        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
 *        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
 *        The '#' is necessary when calling from within sd files, as it stops buffer prereading
 * M33  - Get the longname version of a path
 * M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
 * M48  - Measure Z_Probe repeatability. M48 [P # of points] [X position] [Y position] [V_erboseness #] [E_ngage Probe] [L # of legs of travel]
 * M75  - Start the print job timer
 * M76  - Pause the print job timer
 * M77  - Stop the print job timer
 * M78  - Show statistical information about the print jobs
 * M80  - Turn on Power Supply
 * M81  - Turn off Power Supply
 * M82  - Set E codes absolute (default)
 * M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 * M84  - Disable steppers until next move,
 *        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 * M92  - Set planner.axis_steps_per_mm - same syntax as G92
 * M104 - Set extruder target temp
 * M105 - Read current temp
 * M106 - Fan on
 * M107 - Fan off
 * M108 - Stop the waiting for heaters in M109, M190, M303. Does not affect the target temperature.
 * M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
 *        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
 * M110 - Set the current line number
 * M111 - Set debug flags with S<mask>. See flag bits defined in enum.h.
 * M112 - Emergency stop
 * M113 - Get or set the timeout interval for Host Keepalive "busy" messages
 * M114 - Output current position to serial port
 * M115 - Capabilities string
 * M117 - Display a message on the controller screen
 * M119 - Output Endstop status to serial port
 * M120 - Enable endstop detection
 * M121 - Disable endstop detection
 * M140 - Set bed target temp
 * M145 - Set the heatup state H<hotend> B<bed> F<fan speed> for S<material> (0=PLA, 1=ABS)
 * M149 - Set temperature units
 * M163 - Set a single proportion for a mixing extruder. Requires MIXING_EXTRUDER.
 * M164 - Save the mix as a virtual extruder. Requires MIXING_EXTRUDER and MIXING_VIRTUAL_TOOLS.
 * M165 - Set the proportions for a mixing extruder. Use parameters ABCDHI to set the mixing factors. Requires MIXING_EXTRUDER.
 * M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
 * M200 - Set filament diameter, D<diameter>, setting E axis units to cubic. (Use S0 to revert to linear units.)
 * M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 * M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
 * M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 * M204 - Set default acceleration: P for Printing moves, R for Retract only (no X, Y, Z) moves and T for Travel (non printing) moves (ex. M204 P800 T3000 R9000) in units/sec^2
 * M205 - Set advanced settings. Current units apply:
            S<print> T<travel> minimum speeds
            B<minimum segment time>
            X<max xy jerk>, Z<max Z jerk>, E<max E jerk>
 * M206 - Set additional homing offset
 * M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>
 * M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>
 * M209 - Turn Automatic Retract Detection on/off: S<bool> (For slicers that don't support G10/11).
          Every normal extrude-only move will be classified as retract depending on the direction.
 * M218 - Set a tool offset: T<index> X<offset> Y<offset>
 * M220 - Set Feedrate Percentage: S<percent> ("FR" on your LCD)
 * M221 - Set Flow Percentage: S<percent>
 * M226 - Wait until the specified pin reaches the state required: P<pin number> S<pin state>
 * M240 - Trigger a camera to take a photograph
 * M250 - Set LCD contrast C<contrast value> (value 0..63)
 * M280 - Set servo position absolute. P: servo index, S: angle or microseconds
 * M300 - Play beep sound S<frequency Hz> P<duration ms>
 * M301 - Set PID parameters P I and D
 * M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
 * M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
 * M304 - Set bed PID parameters P I and D
 * M380 - Activate solenoid on active extruder
 * M381 - Disable all solenoids
 * M400 - Finish all moves
 * M401 - Lower Z probe if present
 * M402 - Raise Z probe if present
 * M404 - Display or set the Nominal Filament Width: [ N<diameter> ]
 * M405 - Enable Filament Sensor extrusion control. Optional delay between sensor and extruder: D<cm>
 * M406 - Disable Filament Sensor extrusion control
 * M407 - Display measured filament diameter in millimeters
 * M410 - Quickstop. Abort all the planned moves
 * M420 - Enable/Disable Mesh Leveling (with current values) S1=enable S0=disable
 * M421 - Set a single Z coordinate in the Mesh Leveling grid. X<units> Y<units> Z<units>
 * M428 - Set the home_offset logically based on the current_position
 * M500 - Store parameters in EEPROM
 * M501 - Read parameters from EEPROM (if you need reset them after you changed them temporarily).
 * M502 - Revert to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
 * M503 - Print the current settings (from memory not from EEPROM). Use S0 to leave off headings.
 * M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
 * M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
 * M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
 * M851 - Set Z probe's Z offset in current units. (Negative values apply to probes that extend below the nozzle.)
 * M907 - Set digital trimpot motor current using axis codes.
 * M908 - Control digital trimpot directly.
 * M909 - DAC_STEPPER_CURRENT: Print digipot/DAC current value
 * M910 - DAC_STEPPER_CURRENT: Commit digipot/DAC value to external EEPROM via I2C
 * M350 - Set microstepping mode.
 * M351 - Toggle MS1 MS2 pins directly.
 *
 * ************ SCARA Specific - This can change to suit future G-code regulations
 * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 * M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
 * M365 - SCARA calibration: Scaling factor, X, Y, Z axis
 * ************* SCARA End ***************
 *
 * ************ Custom codes - This can change to suit future G-code regulations
 * M100 - Watch Free Memory (For Debugging Only)
 * M928 - Start SD logging (M928 filename.g) - ended by M29
 * M999 - Restart after being stopped by error
 *
 * "T" Codes
 *
 * T0-T3 - Select a tool by index (usually an extruder) [ F<units/min> ]
 *
 */

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  void gcode_M100();
#endif
bool Running = true;
uint8_t marlin_debug_flags = DEBUG_NONE;
float current_position[NUM_AXIS] = { 0.0 };
static float destination[NUM_AXIS] = { 0.0 };
bool axis_known_position[3] = { false };
bool axis_homed[3] = { false };

static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static char command_queue[BUFSIZE][MAX_CMD_SIZE];
static char* current_command, *current_command_args;
static uint8_t cmd_queue_index_r = 0,
               cmd_queue_index_w = 0,
               commands_in_queue = 0;

#if ENABLED(INCH_MODE_SUPPORT)
  float linear_unit_factor = 1.0;
  float volumetric_unit_factor = 1.0;
#endif
#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnit input_temp_units = TEMPUNIT_C;
#endif

/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
const float homing_feedrate_mm_m[] = {
  HOMING_FEEDRATE_XY, HOMING_FEEDRATE_XY,
  HOMING_FEEDRATE_Z, 0
};
static float feedrate_mm_m = 1500.0, saved_feedrate_mm_m;
int feedrate_percentage = 100, saved_feedrate_percentage;

bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int extruder_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(100);
bool volumetric_enabled = false;
float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(DEFAULT_NOMINAL_FILAMENT_DIA);
float volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(1.0);

// The distance that XYZ has been offset by G92. Reset by G28.
float position_shift[3] = { 0 };

// This offset is added to the configured home position.
// Set by M206, M428, or menu item. Saved to EEPROM.
float home_offset[3] = { 0 };

// Software Endstops. Default to configured limits.
float sw_endstop_min[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float sw_endstop_max[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

#if FAN_COUNT > 0
  int fanSpeeds[FAN_COUNT] = { 0 };
#endif

// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder = 0;

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

volatile bool wait_for_heatup = true;

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

static int serial_count = 0;

// GCode parameter pointer used by code_seen(), code_value_float(), etc.
static char* seen_pointer;

// Next Immediate GCode Command pointer. NULL if none.
const char* queued_commands_P = NULL;

const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42

// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;
Stopwatch print_job_timer = Stopwatch();

// Buzzer - I2C on the LCD or a BEEPER_PIN
#if HAS_BUZZER
  Buzzer buzzer;
  #define BUZZ(d,f) buzzer.tone(d, f)
#else
  #define BUZZ(d,f) NOOP
#endif

static uint8_t target_extruder;
#define PLANNER_XY_FEEDRATE() (min(planner.max_feedrate_mm_s[X_AXIS], planner.max_feedrate_mm_s[Y_AXIS]))

#if defined(XY_PROBE_SPEED)
  #define XY_PROBE_FEEDRATE_MM_M XY_PROBE_SPEED
#else
  #define XY_PROBE_FEEDRATE_MM_M MMS_TO_MMM(PLANNER_XY_FEEDRATE())
#endif

// Extruder offsets
#if HOTENDS > 1
  float hotend_offset[][HOTENDS] = {
    HOTEND_OFFSET_X,
    HOTEND_OFFSET_Y
    #ifdef HOTEND_OFFSET_Z
      , HOTEND_OFFSET_Z
    #endif
  };
#endif

#if HAS_Z_SERVO_ENDSTOP
  const int z_servo_angle[2] = Z_SERVO_ANGLES;
#endif

#if ENABLED(ULTIPANEL) && HAS_POWER_SWITCH
  bool powersupply = true;
#endif
static bool home_all_axis = true;
static bool send_ok[BUFSIZE];
#if ENABLED(PID_EXTRUSION_SCALING)
  int lpq_len = 20;
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  static MarlinBusyState busy_state = NOT_BUSY;
  static millis_t next_busy_signal_ms = 0;
  uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
  #define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
  #define host_keepalive() ;
  #define KEEPALIVE_STATE(n) ;
#endif // HOST_KEEPALIVE_FEATURE

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

void get_available_commands();
void process_next_command();
void prepare_move_to_destination();
void set_current_from_steppers_for_axis(AxisEnum axis);

#if ENABLED(ARC_SUPPORT)
  void plan_arc(float target[NUM_AXIS], float* offset, uint8_t clockwise);
#endif
void serial_echopair_P(const char* s_P, char v)          { serialprintPGM(s_P); SERIAL_CHAR(v); }
void serial_echopair_P(const char* s_P, int v)           { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, long v)          { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, float v)         { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, double v)        { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, unsigned long v) { serialprintPGM(s_P); SERIAL_ECHO(v); }

void tool_change(const uint8_t tmp_extruder, const float fr_mm_m=0.0, bool no_move=false);
static void report_current_position();

#if ENABLED(DEBUG_LEVELING_FEATURE)
  void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
    serialprintPGM(prefix);
    SERIAL_ECHOPAIR("(", x);
    SERIAL_ECHOPAIR(", ", y);
    SERIAL_ECHOPAIR(", ", z);
    SERIAL_ECHOPGM(")");

    if (suffix) serialprintPGM(suffix);
    else SERIAL_EOL;
  }

  void print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
    print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }
  #define DEBUG_POS(SUFFIX,VAR) do { \
    print_xyz(PSTR(STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); } while(0)
#endif

/**
 * sync_plan_position
 * Set planner / stepper positions to the cartesian current_position.
 * The stepper code translates these coordinates into step units.
 * Allows translation between steps and millimeters for cartesian & core robots
 */
inline void sync_plan_position() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position", current_position);
  #endif
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
inline void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_AXIS]); }
#define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()
extern "C" {
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void* __brkval;

  int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}
/**
 * Inject the next "immediate" command, when possible.
 * Return true if any immediate commands remain to inject.
 */
static bool drain_queued_commands_P() {
  if (queued_commands_P != NULL) {
    size_t i = 0;
    char c, cmd[30];
    strncpy_P(cmd, queued_commands_P, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
    cmd[i] = '\0';
    if (enqueue_and_echo_command(cmd)) {   // success?
      if (c)                               // newline char?
        queued_commands_P += i + 1;        // advance to the next command
      else
        queued_commands_P = NULL;          // nul char? no more commands
    }
  }
  return (queued_commands_P != NULL);      // return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_queued_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueue_and_echo_commands_P(const char* pgcode) {
  queued_commands_P = pgcode;
  drain_queued_commands_P(); // first command executed asap (when possible)
}

void clear_command_queue() {
  cmd_queue_index_r = cmd_queue_index_w;
  commands_in_queue = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
  send_ok[cmd_queue_index_w] = say_ok;
  cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
  commands_in_queue++;
}

/**
 * Copy a command directly into the main command buffer, from RAM.
 * Returns true if successfully adds the command
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok=false) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  _commit_command(say_ok);
  return true;
}

void enqueue_and_echo_command_now(const char* cmd) {
  while (!enqueue_and_echo_command(cmd)) idle();
}

/**
 * Enqueue with Serial Echo
 */
bool enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
  if (_enqueuecommand(cmd, say_ok)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_Enqueueing);
    SERIAL_ECHO(cmd);
    SERIAL_ECHOLNPGM("\"");
    return true;
  }
  return false;
}

void setup_killpin() {
  #if HAS_KILL
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN, HIGH);
  #endif
}

// Set home pin
void setup_homepin(void) {
  #if HAS_HOME
    SET_INPUT(HOME_PIN);
    WRITE(HOME_PIN, HIGH);
  #endif
}


void setup_photpin() {
  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif
}

void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init() {
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servo[0].attach(SERVO0_PIN);
    servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servo[1].attach(SERVO1_PIN);
    servo[1].detach();
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servo[2].attach(SERVO2_PIN);
    servo[2].detach();
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servo[3].attach(SERVO3_PIN);
    servo[3].detach();
  #endif

  #if HAS_Z_SERVO_ENDSTOP
    /**
     * Set position of Z Servo Endstop
     *
     * The servo might be deployed and positioned too low to stow
     * when starting up the machine or rebooting the board.
     * There's no way to know where the nozzle is positioned until
     * homing has been done - no homing with z-probe without init!
     *
     */
    STOW_Z_SERVO();
  #endif
}

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void disableStepperDrivers() {
    pinMode(STEPPER_RESET_PIN, OUTPUT);
    digitalWrite(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void enableStepperDrivers() { pinMode(STEPPER_RESET_PIN, INPUT); }  // set to input, which allows it to be pulled high by pullups
#endif

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 */
void setup() {

  #ifdef DISABLE_JTAG
    // Disable JTAG on AT90USB chips to free up pins for IO
    MCUCR = 0x80;
    MCUCR = 0x80;
  #endif

  setup_killpin();

  setup_powerhold();

  #if HAS_STEPPER_RESET
    disableStepperDrivers();
  #endif

  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if (mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if (mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if (mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR = 0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(" " SHORT_BUILD_VERSION);

  #ifdef STRING_DISTRIBUTION_DATE
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif // STRING_CONFIG_H_AUTHOR
  #endif // STRING_DISTRIBUTION_DATE

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  // Send "ok" after commands by default
  for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  Config_RetrieveSettings();

  // Initialize current position based on home_offset
  memcpy(current_position, home_offset, sizeof(home_offset));

  // Vital to init stepper/planner equivalent for current_position
  SYNC_PLAN_POSITION_KINEMATIC();

  thermalManager.init();    // Initialize temperature loop

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif

  stepper.init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  servo_init();

  #if HAS_CONTROLLERFAN
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif

  #if HAS_STEPPER_RESET
    enableStepperDrivers();
  #endif
  #if ENABLED(DAC_STEPPER_CURRENT)
    dac_init();
  #endif

  #if ENABLED(Z_PROBE_SLED) && PIN_EXISTS(SLED)
    pinMode(SLED_PIN, OUTPUT);
    digitalWrite(SLED_PIN, LOW); // turn it off
  #endif // Z_PROBE_SLED

  setup_homepin();

  #ifdef STAT_LED_RED
    pinMode(STAT_LED_RED, OUTPUT);
    digitalWrite(STAT_LED_RED, LOW); // turn it off
  #endif

  #ifdef STAT_LED_BLUE
    pinMode(STAT_LED_BLUE, OUTPUT);
    digitalWrite(STAT_LED_BLUE, LOW); // turn it off
  #endif

  lcd_init();
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD)
      safe_delay(BOOTSCREEN_TIMEOUT);
    #elif ENABLED(ULTRA_LCD)
      bootscreen();
      lcd_init();
    #endif
  #endif

  #if MIXING_VIRTUAL_TOOLS > 1
    // Initialize mixing to 100% color 1
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      mixing_factor[i] = (i == 0) ? 1 : 0;
    for (uint8_t t = 0; t < MIXING_VIRTUAL_TOOLS; t++)
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
        mixing_virtual_tool_mix[t][i] = mixing_factor[i];
  #endif
}

/**
 * The main Marlin program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop() {
  if (commands_in_queue < BUFSIZE) get_available_commands();
  if (commands_in_queue) {
      process_next_command();
     // The queue may be reset by a command handler or by code invoked by idle() within a handler
    if (commands_in_queue) {
      --commands_in_queue;
      cmd_queue_index_r = (cmd_queue_index_r + 1) % BUFSIZE;
    }
  }
  endstops.report_state();
  idle();
}

void gcode_line_error(const char* err, bool doFlush = true) {
  SERIAL_ERROR_START;
  serialprintPGM(err);
  SERIAL_ERRORLN(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

inline void get_serial_commands() {
  static char serial_line_buffer[MAX_CMD_SIZE];
  static boolean serial_comment_mode = false;
  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {

    char serial_char = MYSERIAL.read();

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false; // end of line == end of comment

      if (!serial_count) continue; // skip empty lines

      serial_line_buffer[serial_count] = 0; // terminate string
      serial_count = 0; //reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces
      char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line
      char* apos = strchr(command, '*');

      if (npos) {

        boolean M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }

        if (apos) {
          byte checksum = 0, count = 0;
          while (command[count] != '*') checksum ^= command[count++];

          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
          // if no errors, continue parsing
        }
        else {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
        // if no errors, continue parsing
      }
      else if (apos) { // No '*' without 'N'
        gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
        return;
      }

      // Movement commands alert when stopped
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }
      // If command was e-stop process now
      if (strcmp(command, "M108") == 0) wait_for_heatup = false;
      if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
      if (strcmp(command, "M410") == 0) { quickstop_stepper(); }
      // Add the command to the queue
      _enqueuecommand(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') {  // Handle escapes
      if (MYSERIAL.available() > 0) {
        // if we have one more character, copy it over
        serial_char = MYSERIAL.read();
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // it's not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }

  } // queue has space, serial has data
}

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (queued_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void get_available_commands() {

  // if any immediate commands remain, don't get other commands yet
  if (drain_queued_commands_P()) return;

  get_serial_commands();
}

inline bool code_has_value() {
  int i = 1;
  char c = seen_pointer[i];
  while (c == ' ') c = seen_pointer[++i];
  if (c == '-' || c == '+') c = seen_pointer[++i];
  if (c == '.') c = seen_pointer[++i];
  return NUMERIC(c);
}

inline float code_value_float() {
  float ret;
  char* e = strchr(seen_pointer, 'E');
  if (e) {
    *e = 0;
    ret = strtod(seen_pointer + 1, NULL);
    *e = 'E';
  }
  else
    ret = strtod(seen_pointer + 1, NULL);
  return ret;
}

inline unsigned long code_value_ulong() { return strtoul(seen_pointer + 1, NULL, 10); }

inline long code_value_long() { return strtol(seen_pointer + 1, NULL, 10); }

inline int code_value_int() { return (int)strtol(seen_pointer + 1, NULL, 10); }

inline uint16_t code_value_ushort() { return (uint16_t)strtoul(seen_pointer + 1, NULL, 10); }

inline uint8_t code_value_byte() { return (uint8_t)(constrain(strtol(seen_pointer + 1, NULL, 10), 0, 255)); }

inline bool code_value_bool() { return code_value_byte() > 0; }

#if ENABLED(INCH_MODE_SUPPORT)
  inline void set_input_linear_units(LinearUnit units) {
    switch (units) {
      case LINEARUNIT_INCH:
        linear_unit_factor = 25.4;
        break;
      case LINEARUNIT_MM:
      default:
        linear_unit_factor = 1.0;
        break;
    }
    volumetric_unit_factor = pow(linear_unit_factor, 3.0);
  }

  inline float axis_unit_factor(int axis) {
    return (axis == E_AXIS && volumetric_enabled ? volumetric_unit_factor : linear_unit_factor);
  }

  inline float code_value_linear_units() { return code_value_float() * linear_unit_factor; }
  inline float code_value_axis_units(int axis) { return code_value_float() * axis_unit_factor(axis); }
  inline float code_value_per_axis_unit(int axis) { return code_value_float() / axis_unit_factor(axis); }

#else

  inline float code_value_linear_units() { return code_value_float(); }
  inline float code_value_axis_units(int axis) { UNUSED(axis); return code_value_float(); }
  inline float code_value_per_axis_unit(int axis) { UNUSED(axis); return code_value_float(); }

#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  inline void set_input_temp_units(TempUnit units) { input_temp_units = units; }

  float code_value_temp_abs() {
    switch (input_temp_units) {
      case TEMPUNIT_C:
        return code_value_float();
      case TEMPUNIT_F:
        return (code_value_float() - 32) * 0.5555555556;
      case TEMPUNIT_K:
        return code_value_float() - 272.15;
      default:
        return code_value_float();
    }
  }

  float code_value_temp_diff() {
    switch (input_temp_units) {
      case TEMPUNIT_C:
      case TEMPUNIT_K:
        return code_value_float();
      case TEMPUNIT_F:
        return code_value_float() * 0.5555555556;
      default:
        return code_value_float();
    }
  }
#else
  float code_value_temp_abs() { return code_value_float(); }
  float code_value_temp_diff() { return code_value_float(); }
#endif

FORCE_INLINE millis_t code_value_millis() { return code_value_ulong(); }
inline millis_t code_value_millis_from_seconds() { return code_value_float() * 1000; }

bool code_seen(char code) {
  seen_pointer = strchr(current_command_args, code);
  return (seen_pointer != NULL); // Return TRUE if the code-letter was found
}

/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_extruder_from_command(int code) {
  if (code_seen('T')) {
    if (code_value_byte() >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_CHAR('M');
      SERIAL_ECHO(code);
      SERIAL_ECHOPAIR(" " MSG_INVALID_EXTRUDER " ", code_value_byte());
      SERIAL_EOL;
      return true;
    }
    target_extruder = code_value_byte();
  }
  else
    target_extruder = active_extruder;

  return false;
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
  static inline type pgm_read_any(const type *p)  \
  { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[3] =        \
      { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
  static inline type array(int axis)          \
  { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,   MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,   MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);
/**
 * Software endstops can be used to monitor the open end of
 * an axis that has a hardware endstop on the other end. Or
 * they can prevent axes from moving past endstops and grinding.
 *
 * To keep doing their job as the coordinate system changes,
 * the software endstop positions must be refreshed to remain
 * at the same positions relative to the machine.
 */
static void update_software_endstops(AxisEnum axis) {
  float offs = LOGICAL_POSITION(0, axis);
  {
    sw_endstop_min[axis] = base_min_pos(axis) + offs;
    sw_endstop_max[axis] = base_max_pos(axis) + offs;
  }

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR("For ", axis_codes[axis]);
      SERIAL_ECHOPAIR(" axis:\n home_offset = ", home_offset[axis]);
      SERIAL_ECHOPAIR("\n position_shift = ", position_shift[axis]);
      SERIAL_ECHOPAIR("\n sw_endstop_min = ", sw_endstop_min[axis]);
      SERIAL_ECHOPAIR("\n sw_endstop_max = ", sw_endstop_max[axis]);
      SERIAL_EOL;
    }
  #endif
}

/**
 * Change the home offset for an axis, update the current
 * position and the software endstops to retain the same
 * relative distance to the new home.
 *
 * Since this changes the current_position, code should
 * call sync_plan_position soon after this.
 */
static void set_home_offset(AxisEnum axis, float v) {
  current_position[axis] += v - home_offset[axis];
  home_offset[axis] = v;
  update_software_endstops(axis);
}

static void set_axis_is_at_home(AxisEnum axis) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> set_axis_is_at_home(", axis);
      SERIAL_ECHOLNPGM(")");
    }
  #endif

  position_shift[axis] = 0;

  #if ENABLED(SCARA)

    if (axis == X_AXIS || axis == Y_AXIS) {

      float homeposition[3];
      LOOP_XYZ(i) homeposition[i] = LOGICAL_POSITION(base_home_pos(i), i);

      // SERIAL_ECHOPGM("homeposition[x]= "); SERIAL_ECHO(homeposition[0]);
      // SERIAL_ECHOPGM("homeposition[y]= "); SERIAL_ECHOLN(homeposition[1]);

      /**
       * Works out real Homeposition angles using inverse kinematics,
       * and calculates homing offset using forward kinematics
       */
      inverse_kinematics(homeposition);
      forward_kinematics_SCARA(delta);

      // SERIAL_ECHOPAIR("Delta X=", delta[X_AXIS]);
      // SERIAL_ECHOPGM(" Delta Y="); SERIAL_ECHOLN(delta[Y_AXIS]);

      current_position[axis] = LOGICAL_POSITION(delta[axis], axis);

      /**
       * SCARA home positions are based on configuration since the actual
       * limits are determined by the inverse kinematic transform.
       */
      sw_endstop_min[axis] = base_min_pos(axis); // + (delta[axis] - base_home_pos(axis));
      sw_endstop_max[axis] = base_max_pos(axis); // + (delta[axis] - base_home_pos(axis));
    }
    else
  #endif
  {
    current_position[axis] = LOGICAL_POSITION(base_home_pos(axis), axis);
    update_software_endstops(axis);
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR("> home_offset[", axis_codes[axis]);
        SERIAL_ECHOPAIR("] = ", home_offset[axis]);
        SERIAL_EOL;
        DEBUG_POS("", current_position);
      }
    #endif
  }
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR("<<< set_axis_is_at_home(", axis);
      SERIAL_ECHOLNPGM(")");
    }
  #endif
}

/**
 * Some planner shorthand inline functions
 */
inline float get_homing_bump_feedrate(AxisEnum axis) {
  const int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
  int hbd = homing_bump_divisor[axis];
  if (hbd < 1) {
    hbd = 10;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate_mm_m[axis] / hbd;
}
//
// line_to_current_position
// Move the planner to the current position from wherever it last moved
// (or from wherever it has been told it is located).
//
inline void line_to_current_position() {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(feedrate_mm_m), active_extruder);
}

inline void line_to_z(float zPosition) {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], MMM_TO_MMS(feedrate_mm_m), active_extruder);
}

inline void line_to_axis_pos(AxisEnum axis, float where, float fr_mm_m = 0.0) {
  float old_feedrate_mm_m = feedrate_mm_m;
  current_position[axis] = where;
  feedrate_mm_m = (fr_mm_m != 0.0) ? fr_mm_m : homing_feedrate_mm_m[axis];
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(feedrate_mm_m), active_extruder);
  stepper.synchronize();
  feedrate_mm_m = old_feedrate_mm_m;
}

//
// line_to_destination
// Move the planner, not necessarily synced with current_position
//
inline void line_to_destination(float fr_mm_m) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], MMM_TO_MMS(fr_mm_m), active_extruder);
}
inline void line_to_destination() { line_to_destination(feedrate_mm_m); }

inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void do_blocking_move_to(float x, float y, float z, float fr_mm_m /*=0.0*/) {
  float old_feedrate_mm_m = feedrate_mm_m;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, x, y, z);
  #endif
  // If Z needs to raise, do it before moving XY
  if (current_position[Z_AXIS] < z) {
    feedrate_mm_m = (fr_mm_m != 0.0) ? fr_mm_m : homing_feedrate_mm_m[Z_AXIS];
    current_position[Z_AXIS] = z;
    line_to_current_position();
  }

  feedrate_mm_m = (fr_mm_m != 0.0) ? fr_mm_m : XY_PROBE_FEEDRATE_MM_M;
  current_position[X_AXIS] = x;
  current_position[Y_AXIS] = y;
  line_to_current_position();

  // If Z needs to lower, do it after moving XY
  if (current_position[Z_AXIS] > z) {
    feedrate_mm_m = (fr_mm_m != 0.0) ? fr_mm_m : homing_feedrate_mm_m[Z_AXIS];
    current_position[Z_AXIS] = z;
    line_to_current_position();
  }

  stepper.synchronize();

  feedrate_mm_m = old_feedrate_mm_m;
}
void do_blocking_move_to_x(float x, float fr_mm_m/*=0.0*/) {
  do_blocking_move_to(x, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_m);
}
void do_blocking_move_to_z(float z, float fr_mm_m/*=0.0*/) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z, fr_mm_m);
}
void do_blocking_move_to_xy(float x, float y, float fr_mm_m/*=0.0*/) {
  do_blocking_move_to(x, y, current_position[Z_AXIS], fr_mm_m);
}

//
// Prepare to do endstop or probe moves
// with custom feedrates.
//
//  - Save current feedrates
//  - Reset the rate multiplier
//  - Reset the command timeout
//  - Enable the endstops (for endstop moves)
//
static void setup_for_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("setup_for_endstop_or_probe_move", current_position);
  #endif
  saved_feedrate_mm_m = feedrate_mm_m;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
  refresh_cmd_timeout();
}

static void clean_up_after_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("clean_up_after_endstop_or_probe_move", current_position);
  #endif
  feedrate_mm_m = saved_feedrate_mm_m;
  feedrate_percentage = saved_feedrate_percentage;
  refresh_cmd_timeout();
}

#if ENABLED(Z_PROBE_ALLEN_KEY) || ENABLED(Z_PROBE_SLED) || ENABLED(Z_SAFE_HOMING) || HAS_PROBING_PROCEDURE || HOTENDS > 1 || ENABLED(NOZZLE_CLEAN_FEATURE) || ENABLED(NOZZLE_PARK_FEATURE)
  static bool axis_unhomed_error(const bool x, const bool y, const bool z) {
    const bool xx = x && !axis_homed[X_AXIS],
               yy = y && !axis_homed[Y_AXIS],
               zz = z && !axis_homed[Z_AXIS];
    if (xx || yy || zz) {
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOME " ");
      if (xx) SERIAL_ECHOPGM(MSG_X);
      if (yy) SERIAL_ECHOPGM(MSG_Y);
      if (zz) SERIAL_ECHOPGM(MSG_Z);
      SERIAL_ECHOLNPGM(" " MSG_FIRST);

      #if ENABLED(ULTRA_LCD)
        char message[3 * (LCD_WIDTH) + 1] = ""; // worst case is kana.utf with up to 3*LCD_WIDTH+1
        strcat_P(message, PSTR(MSG_HOME " "));
        if (xx) strcat_P(message, PSTR(MSG_X));
        if (yy) strcat_P(message, PSTR(MSG_Y));
        if (zz) strcat_P(message, PSTR(MSG_Z));
        strcat_P(message, PSTR(" " MSG_FIRST));
        lcd_setstatus(message);
      #endif
      return true;
    }
    return false;
  }
#endif

#if ENABLED(Z_PROBE_SLED)

  #ifndef SLED_DOCKING_OFFSET
    #define SLED_DOCKING_OFFSET 0
  #endif

  /**
   * Method to dock/undock a sled designed by Charles Bell.
   *
   * stow[in]     If false, move to MAX_X and engage the solenoid
   *              If true, move to MAX_X and release the solenoid
   */
  static void dock_sled(bool stow) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR("dock_sled(", stow);
        SERIAL_ECHOLNPGM(")");
      }
    #endif

    // Dock sled a bit closer to ensure proper capturing
    do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));

    #if PIN_EXISTS(SLED)
      digitalWrite(SLED_PIN, !stow); // switch solenoid
    #endif
  }

#endif // Z_PROBE_SLED
#if ENABLED(Z_PROBE_ALLEN_KEY)
  void run_deploy_moves_script() {
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_1_X, Z_PROBE_ALLEN_KEY_DEPLOY_1_Y, Z_PROBE_ALLEN_KEY_DEPLOY_1_Z, Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE);
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_2_X, Z_PROBE_ALLEN_KEY_DEPLOY_2_Y, Z_PROBE_ALLEN_KEY_DEPLOY_2_Z, Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE);
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_3_X, Z_PROBE_ALLEN_KEY_DEPLOY_3_Y, Z_PROBE_ALLEN_KEY_DEPLOY_3_Z, Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE);
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_4_X, Z_PROBE_ALLEN_KEY_DEPLOY_4_Y, Z_PROBE_ALLEN_KEY_DEPLOY_4_Z, Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE);
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_5_X, Z_PROBE_ALLEN_KEY_DEPLOY_5_Y, Z_PROBE_ALLEN_KEY_DEPLOY_5_Z, Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE);
    #endif
  }
  void run_stow_moves_script() {
    #if defined(Z_PROBE_ALLEN_KEY_STOW_1_X) || defined(Z_PROBE_ALLEN_KEY_STOW_1_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_1_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_X
        #define Z_PROBE_ALLEN_KEY_STOW_1_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_Y
        #define Z_PROBE_ALLEN_KEY_STOW_1_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_Z
        #define Z_PROBE_ALLEN_KEY_STOW_1_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_1_X, Z_PROBE_ALLEN_KEY_STOW_1_Y, Z_PROBE_ALLEN_KEY_STOW_1_Z, Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE);
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_STOW_2_X) || defined(Z_PROBE_ALLEN_KEY_STOW_2_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_2_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_X
        #define Z_PROBE_ALLEN_KEY_STOW_2_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_Y
        #define Z_PROBE_ALLEN_KEY_STOW_2_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_Z
        #define Z_PROBE_ALLEN_KEY_STOW_2_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_2_X, Z_PROBE_ALLEN_KEY_STOW_2_Y, Z_PROBE_ALLEN_KEY_STOW_2_Z, Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE);
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_STOW_3_X) || defined(Z_PROBE_ALLEN_KEY_STOW_3_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_3_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_X
        #define Z_PROBE_ALLEN_KEY_STOW_3_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_Y
        #define Z_PROBE_ALLEN_KEY_STOW_3_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_Z
        #define Z_PROBE_ALLEN_KEY_STOW_3_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_3_X, Z_PROBE_ALLEN_KEY_STOW_3_Y, Z_PROBE_ALLEN_KEY_STOW_3_Z, Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE);
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_STOW_4_X) || defined(Z_PROBE_ALLEN_KEY_STOW_4_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_4_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_X
        #define Z_PROBE_ALLEN_KEY_STOW_4_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_Y
        #define Z_PROBE_ALLEN_KEY_STOW_4_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_Z
        #define Z_PROBE_ALLEN_KEY_STOW_4_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_4_X, Z_PROBE_ALLEN_KEY_STOW_4_Y, Z_PROBE_ALLEN_KEY_STOW_4_Z, Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE);
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_STOW_5_X) || defined(Z_PROBE_ALLEN_KEY_STOW_5_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_5_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_X
        #define Z_PROBE_ALLEN_KEY_STOW_5_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_Y
        #define Z_PROBE_ALLEN_KEY_STOW_5_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_Z
        #define Z_PROBE_ALLEN_KEY_STOW_5_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE 0.0
      #endif
      do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_5_X, Z_PROBE_ALLEN_KEY_STOW_5_Y, Z_PROBE_ALLEN_KEY_STOW_5_Z, Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE);
    #endif
  }
#endif

/**
 * Home an individual axis
 */

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(AxisEnum axis) {
  #define HOMEAXIS_DO(LETTER) \
    ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (!(axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0)) return;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> homeaxis(", axis);
      SERIAL_ECHOLNPGM(")");
    }
  #endif

  int axis_home_dir = home_dir(axis);
  // Set the axis position as setup for the move
  current_position[axis] = 0;
  sync_plan_position();

  // Move towards the endstop until an endstop is triggered
  line_to_axis_pos(axis, 1.5 * max_length(axis) * axis_home_dir);

  // Set the axis position as setup for the move
  current_position[axis] = 0;
  sync_plan_position();

  // Move away from the endstop by the axis HOME_BUMP_MM
  line_to_axis_pos(axis, -home_bump_mm(axis) * axis_home_dir);

  // Move slowly towards the endstop until triggered
  line_to_axis_pos(axis, 2 * home_bump_mm(axis) * axis_home_dir, get_homing_bump_feedrate(axis));

  // reset current_position to 0 to reflect hitting endpoint
  current_position[axis] = 0;
  sync_plan_position();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("> TRIGGER ENDSTOP", current_position);
  #endif

  // Set the axis position to its home position (plus home offsets)
  set_axis_is_at_home(axis);

  SYNC_PLAN_POSITION_KINEMATIC();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("> AFTER set_axis_is_at_home", current_position);
  #endif

  destination[axis] = current_position[axis];
  endstops.hit_on_purpose(); // clear endstop hit flags
  axis_known_position[axis] = true;
  axis_homed[axis] = true;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR("<<< homeaxis(", axis);
      SERIAL_ECHOLNPGM(")");
    }
  #endif
}
/**
 * ***************************************************************************
 * ***************************** G-CODE HANDLING *****************************
 * ***************************************************************************
 */

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void gcode_get_destination() {
  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value_axis_units(i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }

  if (code_seen('F') && code_value_linear_units() > 0.0)
    feedrate_mm_m = code_value_linear_units();

  // Get ABCDHI mixing factors
  #if ENABLED(DIRECT_MIXING_IN_G1)
    gcode_get_mix();
  #endif
}

void unknown_command_error() {
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
  SERIAL_ECHO(current_command);
  SERIAL_ECHOLNPGM("\"");
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void host_keepalive() {
    millis_t ms = millis();
    if (host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_BUSY_PROCESSING);
          break;
        case PAUSED_FOR_USER:
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_USER);
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
  }

#endif //HOST_KEEPALIVE_FEATURE

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1() {
  if (IsRunning()) {
    gcode_get_destination(); // For X Y Z E F
    prepare_move_to_destination();
  }
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 */
#if ENABLED(ARC_SUPPORT)
  inline void gcode_G2_G3(bool clockwise) {
    if (IsRunning()) {
      gcode_get_destination();
      // Center of arc as offset from current_position
      float arc_offset[2] = {
        code_seen('I') ? code_value_axis_units(X_AXIS) : 0,
        code_seen('J') ? code_value_axis_units(Y_AXIS) : 0
      };

      // Send an arc to the planner
      plan_arc(destination, arc_offset, clockwise);

      refresh_cmd_timeout();
    }
  }
#endif

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t dwell_ms = 0;

  if (code_seen('P')) dwell_ms = code_value_millis(); // milliseconds to wait
  if (code_seen('S')) dwell_ms = code_value_millis_from_seconds(); // seconds to wait

  stepper.synchronize();
  refresh_cmd_timeout();
  dwell_ms += previous_cmd_ms;  // keep track of when we started waiting

  if (!lcd_hasstatus()) LCD_MESSAGEPGM(MSG_DWELL);

  while (PENDING(millis(), dwell_ms)) idle();
}

#if ENABLED(NOZZLE_CLEAN_FEATURE)
  /**
   * G12: Clean the nozzle
   */
  inline void gcode_G12() {
    // Don't allow nozzle cleaning without homing first
    if (axis_unhomed_error(true, true, true)) { return; }

    uint8_t const pattern = code_seen('P') ? code_value_ushort() : 0;
    uint8_t const strokes = code_seen('S') ? code_value_ushort() : NOZZLE_CLEAN_STROKES;
    uint8_t const objects = code_seen('T') ? code_value_ushort() : 3;

    Nozzle::clean(pattern, strokes, objects);
  }
#endif

#if ENABLED(INCH_MODE_SUPPORT)
  /**
   * G20: Set input mode to inches
   */
  inline void gcode_G20() {
    set_input_linear_units(LINEARUNIT_INCH);
  }

  /**
   * G21: Set input mode to millimeters
   */
  inline void gcode_G21() {
    set_input_linear_units(LINEARUNIT_MM);
  }
#endif

#if ENABLED(NOZZLE_PARK_FEATURE)
  /**
   * G27: Park the nozzle
   */
  inline void gcode_G27() {
    // Don't allow nozzle parking without homing first
    if (axis_unhomed_error(true, true, true)) { return; }
    uint8_t const z_action = code_seen('P') ? code_value_ushort() : 0;
    Nozzle::park(z_action);
  }
#endif // NOZZLE_PARK_FEATURE

#if ENABLED(QUICK_HOME)

  static void quick_home_xy() {

    // Pretend the current position is 0,0
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
    sync_plan_position();

    int x_axis_home_dir = home_dir(X_AXIS);

    float mlx = max_length(X_AXIS),
          mly = max_length(Y_AXIS),
          mlratio = mlx > mly ? mly / mlx : mlx / mly,
          fr_mm_m = min(homing_feedrate_mm_m[X_AXIS], homing_feedrate_mm_m[Y_AXIS]) * sqrt(sq(mlratio) + 1.0);

    do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir(Y_AXIS), fr_mm_m);
    endstops.hit_on_purpose(); // clear endstop hit flags
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;

  }

#endif // QUICK_HOME

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28() {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM(">>> gcode_G28");
  #endif

  // Wait for planner moves to finish!
  stepper.synchronize();

  // Always home with tool 0 active
  #if HOTENDS > 1
    uint8_t old_tool_index = active_extruder;
    tool_change(0, 0, true);
  #endif
  setup_for_endstop_or_probe_move();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> endstops.enable(true)");
  #endif
  endstops.enable(true); // Enable endstops for next homing move

  bool homeX = code_seen('X'), homeY = code_seen('Y'), homeZ = code_seen('Z');

  home_all_axis = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

  set_destination_to_current();

  #if Z_HOME_DIR > 0  // If homing away from BED do Z first

    if (home_all_axis || homeZ) {
      HOMEAXIS(Z);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> HOMEAXIS(Z)", current_position);
      #endif
    }

  #else

    if (home_all_axis || homeX || homeY) {
      // Raise Z before homing any other axes and z is not already high enough (never lower z)
      destination[Z_AXIS] = LOGICAL_Z_POSITION(Z_HOMING_HEIGHT);
      if (destination[Z_AXIS] > current_position[Z_AXIS]) {

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOPAIR("Raise Z (before homing) to ", destination[Z_AXIS]);
            SERIAL_EOL;
          }
        #endif

        do_blocking_move_to_z(destination[Z_AXIS]);
      }
    }

  #endif

  #if ENABLED(QUICK_HOME)

    if (home_all_axis || (homeX && homeY)) quick_home_xy();

  #endif

  #if ENABLED(HOME_Y_BEFORE_X)

    // Home Y
    if (home_all_axis || homeY) {
      HOMEAXIS(Y);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
      #endif
    }

  #endif

  // Home X
  if (home_all_axis || homeX) {
    HOMEAXIS(X);
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> homeX", current_position);
    #endif
  }

  #if DISABLED(HOME_Y_BEFORE_X)
    // Home Y
    if (home_all_axis || homeY) {
      HOMEAXIS(Y);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
      #endif
    }
  #endif

  // Home Z last if homing towards the bed
  #if Z_HOME_DIR < 0

    if (home_all_axis || homeZ) {

      #if ENABLED(Z_SAFE_HOMING)

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOLNPGM("> Z_SAFE_HOMING >>>");
          }
        #endif

        if (home_all_axis) {

          /**
            * At this point we already have Z at Z_HOMING_HEIGHT height
            * No need to move Z any more as this height should already be safe
            * enough to reach Z_SAFE_HOMING XY positions.
            * Just make sure the planner is in sync.
            */
          SYNC_PLAN_POSITION_KINEMATIC();

          /**
            * Move the Z probe (or just the nozzle) to the safe homing point
            */
          destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - (X_PROBE_OFFSET_FROM_EXTRUDER));
          destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - (Y_PROBE_OFFSET_FROM_EXTRUDER));
          destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              DEBUG_POS("> Z_SAFE_HOMING > home_all_axis", current_position);
              DEBUG_POS("> Z_SAFE_HOMING > home_all_axis", destination);
            }
          #endif

          // Move in the XY plane
          do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
        }

        // Let's see if X and Y are homed
        if (axis_unhomed_error(true, true, false)) return;

        /**
          * Make sure the Z probe is within the physical limits
          * NOTE: This doesn't necessarily ensure the Z probe is also
          * within the bed!
          */
        float cpx = RAW_CURRENT_POSITION(X_AXIS), cpy = RAW_CURRENT_POSITION(Y_AXIS);
        if (   cpx >= X_MIN_POS - (X_PROBE_OFFSET_FROM_EXTRUDER)
            && cpx <= X_MAX_POS - (X_PROBE_OFFSET_FROM_EXTRUDER)
            && cpy >= Y_MIN_POS - (Y_PROBE_OFFSET_FROM_EXTRUDER)
            && cpy <= Y_MAX_POS - (Y_PROBE_OFFSET_FROM_EXTRUDER)) {

          // Home the Z axis
          HOMEAXIS(Z);
        }
        else {
          LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
        }

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOLNPGM("<<< Z_SAFE_HOMING");
          }
        #endif

      #else // !Z_SAFE_HOMING

        HOMEAXIS(Z);

      #endif // !Z_SAFE_HOMING

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> (home_all_axis || homeZ) > final", current_position);
      #endif

    } // home_all_axis || homeZ

  #endif // Z_HOME_DIR < 0

  SYNC_PLAN_POSITION_KINEMATIC();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> endstops.not_homing()");
  #endif
  endstops.not_homing();
  endstops.hit_on_purpose(); // clear endstop hit flags
  clean_up_after_endstop_or_probe_move();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< gcode_G28");
  #endif

  // Restore the active tool after homing
  #if HOTENDS > 1
    tool_change(old_tool_index, 0, true);
  #endif

  report_current_position();
}

#if HAS_PROBING_PROCEDURE

  void out_of_range_error(const char* p_edge) {
    SERIAL_PROTOCOLPGM("?Probe ");
    serialprintPGM(p_edge);
    SERIAL_PROTOCOLLNPGM(" position out of range.");
  }

#endif

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
  bool didE = code_seen('E');

  if (!didE) stepper.synchronize();

  bool didXYZ = false;
  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      float p = current_position[i],
            v = code_value_axis_units(i);

      current_position[i] = v;

      if (i != E_AXIS) {
        position_shift[i] += v - p; // Offset the coordinate space
        update_software_endstops((AxisEnum)i);
        didXYZ = true;
      }
    }
  }
  if (didXYZ)
    SYNC_PLAN_POSITION_KINEMATIC();
  else if (didE)
    sync_plan_position_e();
}

#if ENABLED(ULTIPANEL)

  /**
   * M0: Unconditional stop - Wait for user button press on LCD
   * M1: Conditional stop   - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    char* args = current_command_args;

    millis_t codenum = 0;
    bool hasP = false, hasS = false;
    if (code_seen('P')) {
      codenum = code_value_millis(); // milliseconds to wait
      hasP = codenum > 0;
    }
    if (code_seen('S')) {
      codenum = code_value_millis_from_seconds(); // seconds to wait
      hasS = codenum > 0;
    }

    if (!hasP && !hasS && *args != '\0')
      lcd_setstatus(args, true);
    else {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        dontExpireStatus();
      #endif
    }

    lcd_ignore_click();
    stepper.synchronize();
    refresh_cmd_timeout();
    if (codenum > 0) {
      codenum += previous_cmd_ms;  // wait until this time for a click
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      while (PENDING(millis(), codenum) && !lcd_clicked()) idle();
      KEEPALIVE_STATE(IN_HANDLER);
      lcd_ignore_click(false);
    }
    else {
      if (!lcd_detected()) return;
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      while (!lcd_clicked()) idle();
      KEEPALIVE_STATE(IN_HANDLER);
    }
    if (IS_SD_PRINTING)
      LCD_MESSAGEPGM(MSG_RESUMING);
    else
      LCD_MESSAGEPGM(WELCOME_MSG);
  }

#endif // ULTIPANEL

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  enable_all_steppers();
}

/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
  char buffer[21];
  duration_t elapsed = print_job_timer.duration();
  elapsed.toString(buffer);

  lcd_setstatus(buffer);

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("Print time: ");
  SERIAL_ECHOLN(buffer);

  thermalManager.autotempShutdown();
}
/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *  S<byte> Pin status from 0 - 255
 */
inline void gcode_M42() {
  if (!code_seen('S')) return;

  int pin_status = code_value_int();
  if (pin_status < 0 || pin_status > 255) return;

  int pin_number = code_seen('P') ? code_value_int() : LED_PIN;
  if (pin_number < 0) return;

  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (pin_number == sensitive_pins[i]) return;

  pinMode(pin_number, OUTPUT);
  digitalWrite(pin_number, pin_status);
  analogWrite(pin_number, pin_status);

  #if FAN_COUNT > 0
    switch (pin_number) {
      #if HAS_FAN0
        case FAN_PIN: fanSpeeds[0] = pin_status; break;
      #endif
      #if HAS_FAN1
        case FAN1_PIN: fanSpeeds[1] = pin_status; break;
      #endif
      #if HAS_FAN2
        case FAN2_PIN: fanSpeeds[2] = pin_status; break;
      #endif
    }
  #endif
}

#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)

  /**
   * M48: Z probe repeatability measurement function.
   *
   * Usage:
   *   M48 <P#> <X#> <Y#> <V#> <E> <L#>
   *     P = Number of sampled points (4-50, default 10)
   *     X = Sample X position
   *     Y = Sample Y position
   *     V = Verbose level (0-4, default=1)
   *     E = Engage Z probe for each reading
   *     L = Number of legs of movement before probe
   *     S = Schizoid (Or Star if you prefer)
   *
   * This function assumes the bed has been homed.  Specifically, that a G28 command
   * as been issued prior to invoking the M48 Z probe repeatability measurement function.
   * Any information generated by a prior G29 Bed leveling command will be lost and need to be
   * regenerated.
   */
  inline void gcode_M48() {

    if (axis_unhomed_error(true, true, true)) return;

    int8_t verbose_level = code_seen('V') ? code_value_byte() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      SERIAL_PROTOCOLLNPGM("?Verbose Level not plausible (0-4).");
      return;
    }

    if (verbose_level > 0)
      SERIAL_PROTOCOLLNPGM("M48 Z-Probe Repeatability test");

    int8_t n_samples = code_seen('P') ? code_value_byte() : 10;
    if (n_samples < 4 || n_samples > 50) {
      SERIAL_PROTOCOLLNPGM("?Sample size not plausible (4-50).");
      return;
    }

    float  X_current = current_position[X_AXIS],
           Y_current = current_position[Y_AXIS];

    bool stow_probe_after_each = code_seen('E');

    float X_probe_location = code_seen('X') ? code_value_axis_units(X_AXIS) : X_current + X_PROBE_OFFSET_FROM_EXTRUDER;
    float Y_probe_location = code_seen('Y') ? code_value_axis_units(Y_AXIS) : Y_current + Y_PROBE_OFFSET_FROM_EXTRUDER;
    if (HYPOT(RAW_X_POSITION(X_probe_location), RAW_Y_POSITION(Y_probe_location)) > DELTA_PROBEABLE_RADIUS) {
       SERIAL_PROTOCOLLNPGM("? (X,Y) location outside of probeable radius.");
       return;
    }
    bool seen_L = code_seen('L');
    uint8_t n_legs = seen_L ? code_value_byte() : 0;
    if (n_legs > 15) {
      SERIAL_PROTOCOLLNPGM("?Number of legs in movement not plausible (0-15).");
      return;
    }
    if (n_legs == 1) n_legs = 2;

    bool schizoid_flag = code_seen('S');
    if (schizoid_flag && !seen_L) n_legs = 7;

    /**
     * Now get everything to the specified probe point So we can safely do a
     * probe to get us close to the bed.  If the Z-Axis is far from the bed,
     * we don't want to use that as a starting point for each probe.
     */
    if (verbose_level > 2)
      SERIAL_PROTOCOLLNPGM("Positioning the probe...");
    setup_for_endstop_or_probe_move();

    // Move to the first point, deploy, and probe
    probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, verbose_level);

    randomSeed(millis());

    double mean = 0, sigma = 0, sample_set[n_samples];
    for (uint8_t n = 0; n < n_samples; n++) {
      if (n_legs) {
        int dir = (random(0, 10) > 5.0) ? -1 : 1;  // clockwise or counter clockwise
        float angle = random(0.0, 360.0),
              radius = random(
                5, X_MAX_LENGTH / 8
              );

        if (verbose_level > 3) {
          SERIAL_ECHOPAIR("Starting radius: ", radius);
          SERIAL_ECHOPAIR("   angle: ", angle);
          SERIAL_ECHOPGM(" Direction: ");
          if (dir > 0) SERIAL_ECHOPGM("Counter-");
          SERIAL_ECHOLNPGM("Clockwise");
        }

        for (uint8_t l = 0; l < n_legs - 1; l++) {
          double delta_angle;

          if (schizoid_flag)
            // The points of a 5 point star are 72 degrees apart.  We need to
            // skip a point and go to the next one on the star.
            delta_angle = dir * 2.0 * 72.0;

          else
            // If we do this line, we are just trying to move further
            // around the circle.
            delta_angle = dir * (float) random(25, 45);

          angle += delta_angle;

          while (angle > 360.0)   // We probably do not need to keep the angle between 0 and 2*PI, but the
            angle -= 360.0;       // Arduino documentation says the trig functions should not be given values
          while (angle < 0.0)     // outside of this range.   It looks like they behave correctly with
            angle += 360.0;       // numbers outside of the range, but just to be safe we clamp them.

          X_current = X_probe_location - (X_PROBE_OFFSET_FROM_EXTRUDER) + cos(RADIANS(angle)) * radius;
          Y_current = Y_probe_location - (Y_PROBE_OFFSET_FROM_EXTRUDER) + sin(RADIANS(angle)) * radius;
          // If we have gone out too far, we can do a simple fix and scale the numbers
          // back in closer to the origin.
          while (HYPOT(X_current, Y_current) > DELTA_PROBEABLE_RADIUS) {
            X_current /= 1.25;
            Y_current /= 1.25;
            if (verbose_level > 3) {
              SERIAL_ECHOPAIR("Pulling point towards center:", X_current);
              SERIAL_ECHOPAIR(", ", Y_current);
              SERIAL_EOL;
            }
          }
          
          if (verbose_level > 3) {
            SERIAL_PROTOCOLPGM("Going to:");
            SERIAL_ECHOPAIR(" X", X_current);
            SERIAL_ECHOPAIR(" Y", Y_current);
            SERIAL_ECHOPAIR(" Z", current_position[Z_AXIS]);
            SERIAL_EOL;
          }
          do_blocking_move_to_xy(X_current, Y_current);
        } // n_legs loop
      } // n_legs

      // Probe a single point
      sample_set[n] = probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, verbose_level);

      /**
       * Get the current mean for the data points we have so far
       */
      double sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);

      /**
       * Now, use that mean to calculate the standard deviation for the
       * data points we have so far
       */
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++)
        sum += sq(sample_set[j] - mean);

      sigma = sqrt(sum / (n + 1));
      if (verbose_level > 0) {
        if (verbose_level > 1) {
          SERIAL_PROTOCOL(n + 1);
          SERIAL_PROTOCOLPGM(" of ");
          SERIAL_PROTOCOL((int)n_samples);
          SERIAL_PROTOCOLPGM("   z: ");
          SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
          if (verbose_level > 2) {
            SERIAL_PROTOCOLPGM(" mean: ");
            SERIAL_PROTOCOL_F(mean, 6);
            SERIAL_PROTOCOLPGM("   sigma: ");
            SERIAL_PROTOCOL_F(sigma, 6);
          }
        }
        SERIAL_EOL;
      }

    } // End of probe loop

    if (STOW_PROBE()) return;

    if (verbose_level > 0) {
      SERIAL_PROTOCOLPGM("Mean: ");
      SERIAL_PROTOCOL_F(mean, 6);
      SERIAL_EOL;
    }

    SERIAL_PROTOCOLPGM("Standard Deviation: ");
    SERIAL_PROTOCOL_F(sigma, 6);
    SERIAL_EOL; SERIAL_EOL;

    clean_up_after_endstop_or_probe_move();

    report_current_position();
  }

#endif // Z_MIN_PROBE_REPEATABILITY_TEST

/**
 * M75: Start print timer
 */
inline void gcode_M75() { print_job_timer.start(); }

/**
 * M76: Pause print timer
 */
inline void gcode_M76() { print_job_timer.pause(); }

/**
 * M77: Stop print timer
 */
inline void gcode_M77() { print_job_timer.stop(); }

/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
  if (get_target_extruder_from_command(104)) return;
  if (DEBUGGING(DRYRUN)) return;
  if (code_seen('S')) {
    thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);
    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      /**
       * Stop the timer at the end of print, starting is managed by
       * 'heat and wait' M109.
       * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
       * stand by mode, for instance in a dual extruder setup, without affecting
       * the running print timer.
       */
      if (code_value_temp_abs() <= (EXTRUDE_MINTEMP)/2) {
        print_job_timer.stop();
        LCD_MESSAGEPGM(WELCOME_MSG);
      }
    #endif

    if (code_value_temp_abs() > thermalManager.degHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }
}

#if HAS_TEMP_HOTEND || HAS_TEMP_BED

  void print_heaterstates() {
    #if HAS_TEMP_HOTEND
      SERIAL_PROTOCOLPGM(" T:");
      SERIAL_PROTOCOL_F(thermalManager.degHotend(target_extruder), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(thermalManager.degTargetHotend(target_extruder), 1);
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        SERIAL_PROTOCOLPAIR(" (", thermalManager.current_temperature_raw[target_extruder] / OVERSAMPLENR);
        SERIAL_CHAR(')');
      #endif
    #endif
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM(" B:");
      SERIAL_PROTOCOL_F(thermalManager.degBed(), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(thermalManager.degTargetBed(), 1);
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        SERIAL_PROTOCOLPAIR(" (", thermalManager.current_temperature_bed_raw / OVERSAMPLENR);
        SERIAL_CHAR(')');
      #endif
    #endif
    #if HOTENDS > 1
      HOTEND_LOOP() {
        SERIAL_PROTOCOLPAIR(" T", e);
        SERIAL_PROTOCOLCHAR(':');
        SERIAL_PROTOCOL_F(thermalManager.degHotend(e), 1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(thermalManager.degTargetHotend(e), 1);
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          SERIAL_PROTOCOLPAIR(" (", thermalManager.current_temperature_raw[e] / OVERSAMPLENR);
          SERIAL_CHAR(')');
        #endif
      }
    #endif
    SERIAL_PROTOCOLPGM(" @:");
    SERIAL_PROTOCOL(thermalManager.getHeaterPower(target_extruder));
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM(" B@:");
      SERIAL_PROTOCOL(thermalManager.getHeaterPower(-1));
    #endif
    #if HOTENDS > 1
      HOTEND_LOOP() {
        SERIAL_PROTOCOLPAIR(" @", e);
        SERIAL_PROTOCOLCHAR(':');
        SERIAL_PROTOCOL(thermalManager.getHeaterPower(e));
      }
    #endif
  }
#endif

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
  if (get_target_extruder_from_command(105)) return;

  #if HAS_TEMP_HOTEND || HAS_TEMP_BED
    SERIAL_PROTOCOLPGM(MSG_OK);
    print_heaterstates();
  #else // !HAS_TEMP_HOTEND && !HAS_TEMP_BED
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_EOL;
}

#if FAN_COUNT > 0

  /**
   * M106: Set Fan Speed
   *
   *  S<int>   Speed between 0-255
   *  P<index> Fan index, if more than one fan
   */
  inline void gcode_M106() {
    uint16_t s = code_seen('S') ? code_value_ushort() : 255,
             p = code_seen('P') ? code_value_ushort() : 0;
    NOMORE(s, 255);
    if (p < FAN_COUNT) fanSpeeds[p] = s;
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() {
    uint16_t p = code_seen('P') ? code_value_ushort() : 0;
    if (p < FAN_COUNT) fanSpeeds[p] = 0;
  }

#endif // FAN_COUNT > 0

/**
  * M108: Stop the waiting for heaters in M109, M190, M303. Does not affect the target temperature.
  */
inline void gcode_M108() { wait_for_heatup = false; }
/**
  * M112: Emergency Stop
  */
inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }
/**
  * M410: Quickstop - Abort all planned moves
  *
  * This will stop the carriages mid-move, so most likely they
  * will be out of sync with the stepper position after this.
  */
inline void gcode_M410() { quickstop_stepper(); }
#ifndef MIN_COOLING_SLOPE_DEG
  #define MIN_COOLING_SLOPE_DEG 1.50
#endif
#ifndef MIN_COOLING_SLOPE_TIME
  #define MIN_COOLING_SLOPE_TIME 60
#endif

/**
 * M109: Sxxx Wait for extruder(s) to reach temperature. Waits only when heating.
 *       Rxxx Wait for extruder(s) to reach temperature. Waits when heating and cooling.
 */
inline void gcode_M109() {

  if (get_target_extruder_from_command(109)) return;
  if (DEBUGGING(DRYRUN)) return;
  bool no_wait_for_cooling = code_seen('S');
  if (no_wait_for_cooling || code_seen('R')) {
    thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);
    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      /**
       * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
       * stand by mode, for instance in a dual extruder setup, without affecting
       * the running print timer.
       */
      if (code_value_temp_abs() <= (EXTRUDE_MINTEMP)/2) {
        print_job_timer.stop();
        LCD_MESSAGEPGM(WELCOME_MSG);
      }
      /**
       * We do not check if the timer is already running because this check will
       * be done for us inside the Stopwatch::start() method thus a running timer
       * will not restart.
       */
      else print_job_timer.start();
    #endif

    if (thermalManager.isHeatingHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }

  #if ENABLED(AUTOTEMP)
    planner.autotemp_M109();
  #endif

  #if TEMP_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    // Loop until the temperature has stabilized
    #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
  #else
    // Loop until the temperature is very close target
    #define TEMP_CONDITIONS (wants_to_cool ? thermalManager.isCoolingHotend(target_extruder) : thermalManager.isHeatingHotend(target_extruder))
  #endif //TEMP_RESIDENCY_TIME > 0

  float theTarget = -1.0, old_temp = 9999.0;
  bool wants_to_cool = false;
  wait_for_heatup = true;
  millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

  KEEPALIVE_STATE(NOT_BUSY);

  do {
    // Target temperature might be changed during the loop
    if (theTarget != thermalManager.degTargetHotend(target_extruder)) {
      wants_to_cool = thermalManager.isCoolingHotend(target_extruder);
      theTarget = thermalManager.degTargetHotend(target_extruder);

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;
    }

    now = millis();
    if (ELAPSED(now, next_temp_ms)) { //Print temp & remaining time every 1s while waiting
      next_temp_ms = now + 1000UL;
      print_heaterstates();
      #if TEMP_RESIDENCY_TIME > 0
        SERIAL_PROTOCOLPGM(" W:");
        if (residency_start_ms) {
          long rem = (((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
          SERIAL_PROTOCOLLN(rem);
        }
        else {
          SERIAL_PROTOCOLLNPGM("?");
        }
      #else
        SERIAL_EOL;
      #endif
    }

    idle();
    refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

    float temp = thermalManager.degHotend(target_extruder);

    #if TEMP_RESIDENCY_TIME > 0

      float temp_diff = fabs(theTarget - temp);

      if (!residency_start_ms) {
        // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
      }
      else if (temp_diff > TEMP_HYSTERESIS) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = now;
      }

    #endif //TEMP_RESIDENCY_TIME > 0

    // Prevent a wait-forever situation if R is misused i.e. M109 R0
    if (wants_to_cool) {
      // break after MIN_COOLING_SLOPE_TIME seconds
      // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
        next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
        old_temp = temp;
      }
    }

  } while (wait_for_heatup && TEMP_CONDITIONS);

  LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
  KEEPALIVE_STATE(IN_HANDLER);
}

#if HAS_TEMP_BED

  #ifndef MIN_COOLING_SLOPE_DEG_BED
    #define MIN_COOLING_SLOPE_DEG_BED 1.50
  #endif
  #ifndef MIN_COOLING_SLOPE_TIME_BED
    #define MIN_COOLING_SLOPE_TIME_BED 60
  #endif

  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M190() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_BED_HEATING);
    bool no_wait_for_cooling = code_seen('S');
    if (no_wait_for_cooling || code_seen('R')) {
      thermalManager.setTargetBed(code_value_temp_abs());
      #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
        if (code_value_temp_abs() > BED_MINTEMP) {
          /**
          * We start the timer when 'heating and waiting' command arrives, LCD
          * functions never wait. Cooling down managed by extruders.
          *
          * We do not check if the timer is already running because this check will
          * be done for us inside the Stopwatch::start() method thus a running timer
          * will not restart.
          */
          print_job_timer.start();
        }
      #endif
    }

    #if TEMP_BED_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_BED_CONDITIONS (wants_to_cool ? thermalManager.isCoolingBed() : thermalManager.isHeatingBed())
    #endif //TEMP_BED_RESIDENCY_TIME > 0

    float theTarget = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    KEEPALIVE_STATE(NOT_BUSY);

    target_extruder = active_extruder; // for print_heaterstates

    do {
      // Target temperature might be changed during the loop
      if (theTarget != thermalManager.degTargetBed()) {
        wants_to_cool = thermalManager.isCoolingBed();
        theTarget = thermalManager.degTargetBed();

        // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
        if (no_wait_for_cooling && wants_to_cool) break;
      }

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_heaterstates();
        #if TEMP_BED_RESIDENCY_TIME > 0
          SERIAL_PROTOCOLPGM(" W:");
          if (residency_start_ms) {
            long rem = (((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_PROTOCOLLN(rem);
          }
          else {
            SERIAL_PROTOCOLLNPGM("?");
          }
        #else
          SERIAL_EOL;
        #endif
      }

      idle();
      refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

      float temp = thermalManager.degBed();

      #if TEMP_BED_RESIDENCY_TIME > 0

        float temp_diff = fabs(theTarget - temp);

        if (!residency_start_ms) {
          // Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_BED_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif //TEMP_BED_RESIDENCY_TIME > 0

      // Prevent a wait-forever situation if R is misused i.e. M190 R0
      if (wants_to_cool) {
        // break after MIN_COOLING_SLOPE_TIME_BED seconds
        // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_BED
        if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
          if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
          next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
          old_temp = temp;
        }
      }

    } while (wait_for_heatup && TEMP_BED_CONDITIONS);

    LCD_MESSAGEPGM(MSG_BED_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif // HAS_TEMP_BED

/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
  if (code_seen('N')) gcode_N = code_value_long();
}

/**
 * M111: Set the debug level
 */
inline void gcode_M111() {
  marlin_debug_flags = code_seen('S') ? code_value_byte() : (uint8_t) DEBUG_NONE;

  const static char str_debug_1[] PROGMEM = MSG_DEBUG_ECHO;
  const static char str_debug_2[] PROGMEM = MSG_DEBUG_INFO;
  const static char str_debug_4[] PROGMEM = MSG_DEBUG_ERRORS;
  const static char str_debug_8[] PROGMEM = MSG_DEBUG_DRYRUN;
  const static char str_debug_16[] PROGMEM = MSG_DEBUG_COMMUNICATION;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    const static char str_debug_32[] PROGMEM = MSG_DEBUG_LEVELING;
  #endif

  const static char* const debug_strings[] PROGMEM = {
    str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16,
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      str_debug_32
    #endif
  };

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_DEBUG_PREFIX);
  if (marlin_debug_flags) {
    uint8_t comma = 0;
    for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
      if (TEST(marlin_debug_flags, i)) {
        if (comma++) SERIAL_CHAR(',');
        serialprintPGM((char*)pgm_read_word(&(debug_strings[i])));
      }
    }
  }
  else {
    SERIAL_ECHOPGM(MSG_DEBUG_OFF);
  }
  SERIAL_EOL;
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * M113: Get or set Host Keepalive interval (0 to disable)
   *
   *   S<seconds> Optional. Set the keepalive interval.
   */
  inline void gcode_M113() {
    if (code_seen('S')) {
      host_keepalive_interval = code_value_byte();
      NOMORE(host_keepalive_interval, 60);
    }
    else {
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("M113 S", (unsigned long)host_keepalive_interval);
      SERIAL_EOL;
    }
  }

#endif

/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
  if (DEBUGGING(DRYRUN)) return;
  if (code_seen('S')) thermalManager.setTargetBed(code_value_temp_abs());
}

#if ENABLED(ULTIPANEL)

  /**
   * M145: Set the heatup state for a material in the LCD menu
   *   S<material> (0=PLA, 1=ABS)
   *   H<hotend temp>
   *   B<bed temp>
   *   F<fan speed>
   */
  inline void gcode_M145() {
    int8_t material = code_seen('S') ? (int8_t)code_value_int() : 0;
    if (material < 0 || material > 1) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_MATERIAL_INDEX);
    }
    else {
      int v;
      switch (material) {
        case 0:
          if (code_seen('H')) {
            v = code_value_int();
            preheatHotendTemp1 = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP - 15);
          }
          if (code_seen('F')) {
            v = code_value_int();
            preheatFanSpeed1 = constrain(v, 0, 255);
          }
          #if TEMP_SENSOR_BED != 0
            if (code_seen('B')) {
              v = code_value_int();
              preheatBedTemp1 = constrain(v, BED_MINTEMP, BED_MAXTEMP - 15);
            }
          #endif
          break;
        case 1:
          if (code_seen('H')) {
            v = code_value_int();
            preheatHotendTemp2 = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP - 15);
          }
          if (code_seen('F')) {
            v = code_value_int();
            preheatFanSpeed2 = constrain(v, 0, 255);
          }
          #if TEMP_SENSOR_BED != 0
            if (code_seen('B')) {
              v = code_value_int();
              preheatBedTemp2 = constrain(v, BED_MINTEMP, BED_MAXTEMP - 15);
            }
          #endif
          break;
      }
    }
  }

#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  /**
   * M149: Set temperature units
   */
  inline void gcode_M149() {
    if (code_seen('C')) {
      set_input_temp_units(TEMPUNIT_C);
    } else if (code_seen('K')) {
      set_input_temp_units(TEMPUNIT_K);
    } else if (code_seen('F')) {
      set_input_temp_units(TEMPUNIT_F);
    }
  }
#endif

#if HAS_POWER_SWITCH

  /**
   * M80: Turn on Power Supply
   */
  inline void gcode_M80() {
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); //GND

    /**
     * If you have a switch on suicide pin, this is useful
     * if you want to start another print with suicide feature after
     * a print without suicide...
     */
    #if HAS_SUICIDE
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    #if ENABLED(ULTIPANEL)
      powersupply = true;
      LCD_MESSAGEPGM(WELCOME_MSG);
      lcd_update();
    #endif
  }

#endif // HAS_POWER_SWITCH

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  thermalManager.disable_all_heaters();
  stepper.finish_and_disable();
  #if FAN_COUNT > 0
    #if FAN_COUNT > 1
      for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
    #else
      fanSpeeds[0] = 0;
    #endif
  #endif
  delay(1000); // Wait 1 second before switching off
  #if HAS_SUICIDE
    stepper.synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
  #endif
  #if ENABLED(ULTIPANEL)
    #if HAS_POWER_SWITCH
      powersupply = false;
    #endif
    LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");
    lcd_update();
  #endif
}


/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable all stepper motors
 */
inline void gcode_M18_M84() {
  if (code_seen('S')) {
    stepper_inactive_time = code_value_millis_from_seconds();
  }
  else {
    bool all_axis = !((code_seen('X')) || (code_seen('Y')) || (code_seen('Z')) || (code_seen('E')));
    if (all_axis) {
      stepper.finish_and_disable();
    }
    else {
      stepper.synchronize();
      if (code_seen('X')) disable_x();
      if (code_seen('Y')) disable_y();
      if (code_seen('Z')) disable_z();
      #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
        if (code_seen('E')) {
          disable_e0();
          disable_e1();
          disable_e2();
          disable_e3();
        }
      #endif
    }
  }
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (code_seen('S')) max_inactive_time = code_value_millis_from_seconds();
}

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
 *      (Follows the same syntax as G92)
 */
inline void gcode_M92() {
  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS) {
        float value = code_value_per_axis_unit(i);
        if (value < 20.0) {
          float factor = planner.axis_steps_per_mm[i] / value; // increase e constants if M92 E14 is given for netfab.
          planner.max_e_jerk *= factor;
          planner.max_feedrate_mm_s[i] *= factor;
          planner.max_acceleration_steps_per_s2[i] *= factor;
        }
        planner.axis_steps_per_mm[i] = value;
      }
      else {
        planner.axis_steps_per_mm[i] = code_value_per_axis_unit(i);
      }
    }
  }
  planner.refresh_positioning();
}

/**
 * Output the current position to serial
 */
static void report_current_position() {
  SERIAL_PROTOCOLPGM("X:");
  SERIAL_PROTOCOL(current_position[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(current_position[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(current_position[Z_AXIS]);
  SERIAL_PROTOCOLPGM(" E:");
  SERIAL_PROTOCOL(current_position[E_AXIS]);

  stepper.report_positions();

  #if ENABLED(SCARA)
    SERIAL_PROTOCOLPGM("SCARA Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta:");
    SERIAL_PROTOCOL(delta[Y_AXIS]);
    SERIAL_EOL;

    SERIAL_PROTOCOLPGM("SCARA Cal - Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta (90):");
    SERIAL_PROTOCOL(delta[Y_AXIS] - delta[X_AXIS] - 90);
    SERIAL_EOL;

    SERIAL_PROTOCOLPGM("SCARA step Cal - Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS] / 90 * planner.axis_steps_per_mm[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta:");
    SERIAL_PROTOCOL((delta[Y_AXIS] - delta[X_AXIS]) / 90 * planner.axis_steps_per_mm[Y_AXIS]);
    SERIAL_EOL; SERIAL_EOL;
  #endif
}

/**
 * M114: Output current position to serial port
 */
inline void gcode_M114() { report_current_position(); }

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
  SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() {
  lcd_setstatus(current_command_args);
}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() { endstops.M119(); }

/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
inline void gcode_M120() { endstops.enable_globally(true); }

/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
inline void gcode_M121() { endstops.enable_globally(false); }

/**
 * M200: Set filament diameter and set E axis units to cubic units
 *
 *    T<extruder> - Optional extruder number. Current extruder if omitted.
 *    D<linear> - Diameter of the filament. Use "D0" to switch back to linear units on the E axis.
 */
inline void gcode_M200() {

  if (get_target_extruder_from_command(200)) return;

  if (code_seen('D')) {
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    volumetric_enabled = (code_value_linear_units() != 0.0);
    if (volumetric_enabled) {
      filament_size[target_extruder] = code_value_linear_units();
      // make sure all extruders have some sane value for the filament size
      for (uint8_t i = 0; i < COUNT(filament_size); i++)
        if (! filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
    }
  }
  else {
    //reserved for setting filament diameter via UFID or filament measuring device
    return;
  }
  calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 */
inline void gcode_M201() {
  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      planner.max_acceleration_mm_per_s2[i] = code_value_axis_units(i);
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  planner.reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    LOOP_XYZE(i) {
      if (code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value_axis_units(i) * planner.axis_steps_per_mm[i];
    }
  }
#endif


/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 */
inline void gcode_M203() {
  LOOP_XYZE(i)
    if (code_seen(axis_codes[i]))
      planner.max_feedrate_mm_s[i] = code_value_axis_units(i);
}

/**
 * M204: Set Accelerations in units/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P = Printing moves
 *    R = Retract only (no X, Y, Z) moves
 *    T = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
inline void gcode_M204() {
  if (code_seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    planner.travel_acceleration = planner.acceleration = code_value_linear_units();
    SERIAL_ECHOPAIR("Setting Print and Travel Acceleration: ", planner.acceleration);
    SERIAL_EOL;
  }
  if (code_seen('P')) {
    planner.acceleration = code_value_linear_units();
    SERIAL_ECHOPAIR("Setting Print Acceleration: ", planner.acceleration);
    SERIAL_EOL;
  }
  if (code_seen('R')) {
    planner.retract_acceleration = code_value_linear_units();
    SERIAL_ECHOPAIR("Setting Retract Acceleration: ", planner.retract_acceleration);
    SERIAL_EOL;
  }
  if (code_seen('T')) {
    planner.travel_acceleration = code_value_linear_units();
    SERIAL_ECHOPAIR("Setting Travel Acceleration: ", planner.travel_acceleration);
    SERIAL_EOL;
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (units/s)
 *    T = Min Travel Feed Rate (units/s)
 *    B = Min Segment Time (µs)
 *    X = Max XY Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205() {
  if (code_seen('S')) planner.min_feedrate_mm_s = code_value_linear_units();
  if (code_seen('T')) planner.min_travel_feedrate_mm_s = code_value_linear_units();
  if (code_seen('B')) planner.min_segment_time = code_value_millis();
  if (code_seen('X')) planner.max_xy_jerk = code_value_linear_units();
  if (code_seen('Z')) planner.max_z_jerk = code_value_axis_units(Z_AXIS);
  if (code_seen('E')) planner.max_e_jerk = code_value_axis_units(E_AXIS);
}

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
  LOOP_XYZ(i)
    if (code_seen(axis_codes[i]))
      set_home_offset((AxisEnum)i, code_value_axis_units(i));

  #if ENABLED(SCARA)
    if (code_seen('T')) set_home_offset(X_AXIS, code_value_axis_units(X_AXIS)); // Theta
    if (code_seen('P')) set_home_offset(Y_AXIS, code_value_axis_units(Y_AXIS)); // Psi
  #endif

  SYNC_PLAN_POSITION_KINEMATIC();
  report_current_position();
}

#if HOTENDS > 1

  /**
   * M218 - set hotend offset (in linear units)
   *
   *   T<tool>
   *   X<xoffset>
   *   Y<yoffset>
   */
  inline void gcode_M218() {
    if (get_target_extruder_from_command(218)) return;

    if (code_seen('X')) hotend_offset[X_AXIS][target_extruder] = code_value_axis_units(X_AXIS);
    if (code_seen('Y')) hotend_offset[Y_AXIS][target_extruder] = code_value_axis_units(Y_AXIS);

    #if ENABLED(SWITCHING_EXTRUDER)
      if (code_seen('Z')) hotend_offset[Z_AXIS][target_extruder] = code_value_axis_units(Z_AXIS);
    #endif

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
    HOTEND_LOOP() {
      SERIAL_CHAR(' ');
      SERIAL_ECHO(hotend_offset[X_AXIS][e]);
      SERIAL_CHAR(',');
      SERIAL_ECHO(hotend_offset[Y_AXIS][e]);
      #if ENABLED(SWITCHING_EXTRUDER)
        SERIAL_CHAR(',');
        SERIAL_ECHO(hotend_offset[Z_AXIS][e]);
      #endif
    }
    SERIAL_EOL;
  }

#endif // HOTENDS > 1

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (code_seen('S')) feedrate_percentage = code_value_int();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
  if (get_target_extruder_from_command(221)) return;
  if (code_seen('S'))
    extruder_multiplier[target_extruder] = code_value_int();
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (code_seen('P')) {
    int pin_number = code_value_int();

    int pin_state = code_seen('S') ? code_value_int() : -1; // required pin state - default is inverted

    if (pin_state >= -1 && pin_state <= 1) {

      for (uint8_t i = 0; i < COUNT(sensitive_pins); i++) {
        if (sensitive_pins[i] == pin_number) {
          pin_number = -1;
          break;
        }
      }

      if (pin_number > -1) {
        int target = LOW;

        stepper.synchronize();

        pinMode(pin_number, INPUT);

        switch (pin_state) {
          case 1:
            target = HIGH;
            break;

          case 0:
            target = LOW;
            break;

          case -1:
            target = !digitalRead(pin_number);
            break;
        }

        while (digitalRead(pin_number) != target) idle();

      } // pin_number > -1
    } // pin_state -1 0 1
  } // code_seen('P')
}
#if HAS_BUZZER

  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t const frequency = code_seen('S') ? code_value_ushort() : 260;
    uint16_t duration = code_seen('P') ? code_value_ushort() : 1000;

    // Limits the tone duration to 0-5 seconds.
    NOMORE(duration, 5000);

    BUZZ(duration, frequency);
  }

#endif // HAS_BUZZER

#if ENABLED(PIDTEMP)

  /**
   * M301: Set PID parameters P I D (and optionally C, L)
   *
   *   P[float] Kp term
   *   I[float] Ki term (unscaled)
   *   D[float] Kd term (unscaled)
   *
   * With PID_EXTRUSION_SCALING:
   *
   *   C[float] Kc term
   *   L[float] LPQ length
   */
  inline void gcode_M301() {

    // multi-extruder PID patch: M301 updates or prints a single extruder's PID values
    // default behaviour (omitting E parameter) is to update for extruder 0 only
    int e = code_seen('E') ? code_value_int() : 0; // extruder being updated

    if (e < HOTENDS) { // catch bad input value
      if (code_seen('P')) PID_PARAM(Kp, e) = code_value_float();
      if (code_seen('I')) PID_PARAM(Ki, e) = scalePID_i(code_value_float());
      if (code_seen('D')) PID_PARAM(Kd, e) = scalePID_d(code_value_float());
      #if ENABLED(PID_EXTRUSION_SCALING)
        if (code_seen('C')) PID_PARAM(Kc, e) = code_value_float();
        if (code_seen('L')) lpq_len = code_value_float();
        NOMORE(lpq_len, LPQ_MAX_LEN);
      #endif

      thermalManager.updatePID();
      SERIAL_ECHO_START;
      #if ENABLED(PID_PARAMS_PER_HOTEND)
        SERIAL_ECHOPGM(" e:"); // specify extruder in serial output
        SERIAL_ECHO(e);
      #endif // PID_PARAMS_PER_HOTEND
      SERIAL_ECHOPGM(" p:");
      SERIAL_ECHO(PID_PARAM(Kp, e));
      SERIAL_ECHOPGM(" i:");
      SERIAL_ECHO(unscalePID_i(PID_PARAM(Ki, e)));
      SERIAL_ECHOPGM(" d:");
      SERIAL_ECHO(unscalePID_d(PID_PARAM(Kd, e)));
      #if ENABLED(PID_EXTRUSION_SCALING)
        SERIAL_ECHOPGM(" c:");
        //Kc does not have scaling applied above, or in resetting defaults
        SERIAL_ECHO(PID_PARAM(Kc, e));
      #endif
      SERIAL_EOL;
    }
    else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLN(MSG_INVALID_EXTRUDER);
    }
  }

#endif // PIDTEMP

#if HAS_PHOTOGRAPH
  /**
   * M240: Trigger a camera by emulating a Canon RC-1
   *       See http://www.doc-diy.net/photo/rc-1_hacked/
   */
  inline void gcode_M240() {
      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
      delay(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
  }
#endif //  PHOTOGRAPH_PIN

#if HAS_LCD_CONTRAST

  /**
   * M250: Read and optionally set the LCD contrast
   */
  inline void gcode_M250() {
    if (code_seen('C')) set_lcd_contrast(code_value_int());
    SERIAL_PROTOCOLPGM("lcd contrast value: ");
    SERIAL_PROTOCOL(lcd_contrast);
    SERIAL_EOL;
  }

#endif // HAS_LCD_CONTRAST

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)

  /**
   * M302: Allow cold extrudes, or set the minimum extrude temperature
   *
   *       S<temperature> sets the minimum extrude temperature
   *       P<bool> enables (1) or disables (0) cold extrusion
   *
   *  Examples:
   *
   *       M302         ; report current cold extrusion state
   *       M302 P0      ; enable cold extrusion checking
   *       M302 P1      ; disables cold extrusion checking
   *       M302 S0      ; always allow extrusion (disables checking)
   *       M302 S170    ; only allow extrusion above 170
   *       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
   */
  inline void gcode_M302() {
    bool seen_S = code_seen('S');
    if (seen_S) {
      thermalManager.extrude_min_temp = code_value_temp_abs();
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
    }

    if (code_seen('P'))
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0) || code_value_bool();
    else if (!seen_S) {
      // Report current state
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("Cold extrudes are ", (thermalManager.allow_cold_extrude ? "en" : "dis"));
      SERIAL_ECHOPAIR("abled (min temp ", int(thermalManager.extrude_min_temp + 0.5));
      SERIAL_ECHOLNPGM("C)");
    }
  }

#endif // PREVENT_DANGEROUS_EXTRUDE

/**
 * M303: PID relay autotune
 *
 *       S<temperature> sets the target temperature. (default 150C)
 *       E<extruder> (-1 for the bed) (default 0)
 *       C<cycles>
 *       U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303() {
  #if HAS_PID_HEATING
    int e = code_seen('E') ? code_value_int() : 0;
    int c = code_seen('C') ? code_value_int() : 5;
    bool u = code_seen('U') && code_value_bool();

    float temp = code_seen('S') ? code_value_temp_abs() : (e < 0 ? 70.0 : 150.0);

    if (e >= 0 && e < HOTENDS)
      target_extruder = e;

    KEEPALIVE_STATE(NOT_BUSY); // don't send "busy: processing" messages during autotune output

    thermalManager.PID_autotune(temp, e, c, u);

    KEEPALIVE_STATE(IN_HANDLER);
  #else
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_M303_DISABLED);
  #endif
}

#if ENABLED(SCARA)
  bool SCARA_move_to_cal(uint8_t delta_x, uint8_t delta_y) {
    //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
    //SERIAL_ECHOLNPGM(" Soft endstops disabled");
    if (IsRunning()) {
      //gcode_get_destination(); // For X Y Z E F
      delta[X_AXIS] = delta_x;
      delta[Y_AXIS] = delta_y;
      forward_kinematics_SCARA(delta);
      destination[X_AXIS] = delta[X_AXIS] / axis_scaling[X_AXIS];
      destination[Y_AXIS] = delta[Y_AXIS] / axis_scaling[Y_AXIS];
      prepare_move_to_destination();
      //ok_to_send();
      return true;
    }
    return false;
  }

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360() {
    SERIAL_ECHOLNPGM(" Cal: Theta 0");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361() {
    SERIAL_ECHOLNPGM(" Cal: Theta 90");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362() {
    SERIAL_ECHOLNPGM(" Cal: Psi 0");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363() {
    SERIAL_ECHOLNPGM(" Cal: Psi 90");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364() {
    SERIAL_ECHOLNPGM(" Cal: Theta-Psi 90");
    return SCARA_move_to_cal(45, 135);
  }

  /**
   * M365: SCARA calibration: Scaling factor, X, Y, Z axis
   */
  inline void gcode_M365() {
    LOOP_XYZ(i)
      if (code_seen(axis_codes[i]))
        axis_scaling[i] = code_value_float();
  }

#endif // SCARA

#if ENABLED(EXT_SOLENOID)

  void enable_solenoid(uint8_t num) {
    switch (num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS_SOLENOID_1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
      default:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_INVALID_SOLENOID);
        break;
    }
  }

  void enable_solenoid_on_active_extruder() { enable_solenoid(active_extruder); }

  void disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    OUT_WRITE(SOL1_PIN, LOW);
    OUT_WRITE(SOL2_PIN, LOW);
    OUT_WRITE(SOL3_PIN, LOW);
  }

  /**
   * M380: Enable solenoid on the active extruder
   */
  inline void gcode_M380() { enable_solenoid_on_active_extruder(); }

  /**
   * M381: Disable all solenoids
   */
  inline void gcode_M381() { disable_all_solenoids(); }

#endif // EXT_SOLENOID

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { stepper.synchronize(); }

void quickstop_stepper() {
  stepper.quick_stop();
  #if DISABLED(SCARA)
    stepper.synchronize();
    LOOP_XYZ(i) set_current_from_steppers_for_axis((AxisEnum)i);
    SYNC_PLAN_POSITION_KINEMATIC();
  #endif
}

/**
 * M428: Set home_offset based on the distance between the
 *       current_position and the nearest "reference point."
 *       If an axis is past center its endstop position
 *       is the reference-point. Otherwise it uses 0. This allows
 *       the Z offset to be set near the bed when using a max endstop.
 *
 *       M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *       Use M206 to set these values directly.
 */
inline void gcode_M428() {
  bool err = false;
  LOOP_XYZ(i) {
    if (axis_homed[i]) {
      float base = (current_position[i] > (sw_endstop_min[i] + sw_endstop_max[i]) * 0.5) ? base_home_pos(i) : 0,
            diff = current_position[i] - LOGICAL_POSITION(base, i);
      if (diff > -20 && diff < 20) {
        set_home_offset((AxisEnum)i, home_offset[i] - diff);
      }
      else {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_M428_TOO_FAR);
        LCD_ALERTMESSAGEPGM("Err: Too far!");
        BUZZ(200, 40);
        err = true;
        break;
      }
    }
  }

  if (!err) {
    SYNC_PLAN_POSITION_KINEMATIC();
    report_current_position();
    LCD_MESSAGEPGM(MSG_HOME_OFFSETS_APPLIED);
    BUZZ(200, 659);
    BUZZ(200, 698);
  }
}

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  Config_StoreSettings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  Config_RetrieveSettings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  Config_ResetDefault();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
  Config_PrintSettings(code_seen('S') && !code_value_bool());
}

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
   */
  inline void gcode_M540() {
    if (code_seen('S')) stepper.abort_on_endstop_hit = code_value_bool();
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS_DIGIPOTSS
    LOOP_XYZE(i)
      if (code_seen(axis_codes[i])) stepper.digipot_current(i, code_value_int());
    if (code_seen('B')) stepper.digipot_current(4, code_value_int());
    if (code_seen('S')) for (int i = 0; i <= 4; i++) stepper.digipot_current(i, code_value_int());
  #endif
  #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
    if (code_seen('X')) stepper.digipot_current(0, code_value_int());
  #endif
  #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
    if (code_seen('Z')) stepper.digipot_current(1, code_value_int());
  #endif
  #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
    if (code_seen('E')) stepper.digipot_current(2, code_value_int());
  #endif
  #if ENABLED(DAC_STEPPER_CURRENT)
    if (code_seen('S')) {
      float dac_percent = code_value_float();
      for (uint8_t i = 0; i <= 4; i++) dac_current_percent(i, dac_percent);
    }
    LOOP_XYZE(i) if (code_seen(axis_codes[i])) dac_current_percent(i, code_value_float());
  #endif
}

#if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)

  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    #if HAS_DIGIPOTSS
      stepper.digitalPotWrite(
        code_seen('P') ? code_value_int() : 0,
        code_seen('S') ? code_value_int() : 0
      );
    #endif
    #ifdef DAC_STEPPER_CURRENT
      dac_current_raw(
        code_seen('P') ? code_value_byte() : -1,
        code_seen('S') ? code_value_ushort() : 0
      );
    #endif
  }

  #if ENABLED(DAC_STEPPER_CURRENT) // As with Printrbot RevF

    inline void gcode_M909() { dac_print_values(); }

    inline void gcode_M910() { dac_commit_eeprom(); }

  #endif

#endif // HAS_DIGIPOTSS || DAC_STEPPER_CURRENT

#if HAS_MICROSTEPS

  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350() {
    if (code_seen('S')) for (int i = 0; i <= 4; i++) stepper.microstep_mode(i, code_value_byte());
    LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_mode(i, code_value_byte());
    if (code_seen('B')) stepper.microstep_mode(4, code_value_byte());
    stepper.microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (code_seen('S')) switch (code_value_byte()) {
      case 1:
        LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_ms(i, code_value_byte(), -1);
        if (code_seen('B')) stepper.microstep_ms(4, code_value_byte(), -1);
        break;
      case 2:
        LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_ms(i, -1, code_value_byte());
        if (code_seen('B')) stepper.microstep_ms(4, -1, code_value_byte());
        break;
    }
    stepper.microstep_readings();
  }

#endif // HAS_MICROSTEPS
/**
 * M999: Restart after being stopped
 *
 * Default behaviour is to flush the serial buffer and request
 * a resend to the host starting on the last N line received.
 *
 * Sending "M999 S1" will resume printing without flushing the
 * existing command buffer.
 *
 */
inline void gcode_M999() {
  Running = true;
  lcd_reset_alert_level();

  if (code_seen('S') && code_value_bool()) return;

  // gcode_LastN = Stopped_gcode_LastN;
  FlushSerialRequestResend();
}

#if ENABLED(SWITCHING_EXTRUDER)
  inline void move_extruder_servo(uint8_t e) {
    const int angles[2] = SWITCHING_EXTRUDER_SERVO_ANGLES;
    MOVE_SERVO(SWITCHING_EXTRUDER_SERVO_NR, angles[e]);
  }
#endif

inline void invalid_extruder_error(const uint8_t &e) {
  SERIAL_ECHO_START;
  SERIAL_CHAR('T');
  SERIAL_PROTOCOL_F(e, DEC);
  SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
}

void tool_change(const uint8_t tmp_extruder, const float fr_mm_m/*=0.0*/, bool no_move/*=false*/) {
  #if MIXING_VIRTUAL_TOOLS > 1

    if (tmp_extruder >= MIXING_VIRTUAL_TOOLS) {
      invalid_extruder_error(tmp_extruder);
      return;
    }

    // T0-Tnnn: Switch virtual tool by changing the mix
    for (uint8_t j = 0; j < MIXING_STEPPERS; j++)
      mixing_factor[j] = mixing_virtual_tool_mix[tmp_extruder][j];

  #else  MIXING_VIRTUAL_TOOLS <= 1

    #if HOTENDS > 1

      if (tmp_extruder >= EXTRUDERS) {
        invalid_extruder_error(tmp_extruder);
        return;
      }

      float old_feedrate_mm_m = feedrate_mm_m;

      feedrate_mm_m = fr_mm_m > 0.0 ? (old_feedrate_mm_m = fr_mm_m) : XY_PROBE_FEEDRATE_MM_M;

      if (tmp_extruder != active_extruder) {
        if (!no_move && axis_unhomed_error(true, true, true)) {
          SERIAL_ECHOLNPGM("No move on toolchange");
          no_move = true;
        }

        // Save current position to destination, for use later
        set_destination_to_current();

        #if ENABLED(SWITCHING_EXTRUDER)
          // <0 if the new nozzle is higher, >0 if lower. A bigger raise when lower.
          float z_diff = hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder],
                z_raise = 0.3 + (z_diff > 0.0 ? z_diff : 0.0);

          // Always raise by some amount
          planner.buffer_line(
            current_position[X_AXIS],
            current_position[Y_AXIS],
            current_position[Z_AXIS] + z_raise,
            current_position[E_AXIS],
            planner.max_feedrate_mm_s[Z_AXIS],
            active_extruder
          );
          stepper.synchronize();

          move_extruder_servo(active_extruder);
          delay(500);

          // Move back down, if needed
          if (z_raise != z_diff) {
            planner.buffer_line(
              current_position[X_AXIS],
              current_position[Y_AXIS],
              current_position[Z_AXIS] + z_diff,
              current_position[E_AXIS],
              planner.max_feedrate_mm_s[Z_AXIS],
              active_extruder
            );
            stepper.synchronize();
          }
        #endif

          float xydiff[2] = {
            hotend_offset[X_AXIS][tmp_extruder] - hotend_offset[X_AXIS][active_extruder],
            hotend_offset[Y_AXIS][tmp_extruder] - hotend_offset[Y_AXIS][active_extruder]
          };

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOPAIR("Offset Tool XY by { ", xydiff[X_AXIS]);
            SERIAL_ECHOPAIR(", ", xydiff[Y_AXIS]);
            SERIAL_ECHOLNPGM(" }");
          }
        #endif

        // The newly-selected extruder XY is actually at...
        current_position[X_AXIS] += xydiff[X_AXIS];
        current_position[Y_AXIS] += xydiff[Y_AXIS];
        for (uint8_t i = X_AXIS; i <= Y_AXIS; i++) {
          position_shift[i] += xydiff[i];
          update_software_endstops((AxisEnum)i);
        }

        // Set the new active extruder
        active_extruder = tmp_extruder;
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("Sync After Toolchange", current_position);
        #endif

        // Tell the planner the new "current position"
        SYNC_PLAN_POSITION_KINEMATIC();

        // Move to the "old position" (move the extruder into place)
        if (!no_move && IsRunning()) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("Move back", destination);
          #endif
          prepare_move_to_destination();
        }

      } // (tmp_extruder != active_extruder)

      stepper.synchronize();

      #if ENABLED(EXT_SOLENOID)
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif // EXT_SOLENOID

      feedrate_mm_m = old_feedrate_mm_m;

    #else // HOTENDS <= 1

      // Set the new active extruder
      active_extruder = tmp_extruder;

      UNUSED(fr_mm_m);
      UNUSED(no_move);

    #endif // HOTENDS <= 1

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_ACTIVE_EXTRUDER);
    SERIAL_PROTOCOLLN((int)active_extruder);

  #endif //MIXING_VIRTUAL_TOOLS <= 1
}

/**
 * T0-T3: Switch tool, usually switching extruders
 *
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 */
inline void gcode_T(uint8_t tmp_extruder) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> gcode_T(", tmp_extruder);
      SERIAL_ECHOLNPGM(")");
      DEBUG_POS("BEFORE", current_position);
    }
  #endif

  #if HOTENDS == 1 || ( MIXING_VIRTUAL_TOOLS > 1)

    tool_change(tmp_extruder);

  #elif HOTENDS > 1

    tool_change(
      tmp_extruder,
      code_seen('F') ? code_value_axis_units(X_AXIS) : 0.0,
      (tmp_extruder == active_extruder) || (code_seen('S') && code_value_bool())
    );

  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      DEBUG_POS("AFTER", current_position);
      SERIAL_ECHOLNPGM("<<< gcode_T");
    }
  #endif
}

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
  current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN(current_command);
  }

  // Sanitize the current command:
  //  - Skip leading spaces
  //  - Bypass N[-0-9][0-9]*[ ]*
  //  - Overwrite * with nul to mark the end
  while (*current_command == ' ') ++current_command;
  if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
    current_command += 2; // skip N[-0-9]
    while (NUMERIC(*current_command)) ++current_command; // skip [0-9]*
    while (*current_command == ' ') ++current_command; // skip [ ]*
  }
  char* starpos = strchr(current_command, '*');  // * should always be the last parameter
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

  char *cmd_ptr = current_command;

  // Get the command code, which must be G, M, or T
  char command_code = *cmd_ptr++;

  // Skip spaces to get the numeric part
  while (*cmd_ptr == ' ') cmd_ptr++;

  uint16_t codenum = 0; // define ahead of goto

  // Bail early if there's no code
  bool code_is_good = NUMERIC(*cmd_ptr);
  if (!code_is_good) goto ExitUnknownCommand;

  // Get and skip the code number
  do {
    codenum = (codenum * 10) + (*cmd_ptr - '0');
    cmd_ptr++;
  } while (NUMERIC(*cmd_ptr));

  // Skip all spaces to get to the first argument, or nul
  while (*cmd_ptr == ' ') cmd_ptr++;

  // The command's arguments (if any) start here, for sure!
  current_command_args = cmd_ptr;

  KEEPALIVE_STATE(IN_HANDLER);

  // Handle a known G, M, or T
  switch (command_code) {
    case 'G': switch (codenum) {

      // G0, G1
      case 0:
      case 1:
        gcode_G0_G1();
        break;

      // G2, G3
      #if ENABLED(ARC_SUPPORT) && DISABLED(SCARA)
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(codenum == 2);
          break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4();
        break;
      #if ENABLED(NOZZLE_CLEAN_FEATURE)
        case 12:
          gcode_G12(); // G12: Nozzle Clean
          break;
      #endif // NOZZLE_CLEAN_FEATURE

      #if ENABLED(INCH_MODE_SUPPORT)
        case 20: //G20: Inch Mode
          gcode_G20();
          break;

        case 21: //G21: MM Mode
          gcode_G21();
          break;
      #endif // INCH_MODE_SUPPORT

      #if ENABLED(NOZZLE_PARK_FEATURE)
        case 27: // G27: Nozzle Park
          gcode_G27();
          break;
      #endif // NOZZLE_PARK_FEATURE

      case 28: // G28: Home all axes, one at a time
        gcode_G28();
        break;
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;

      case 92: // G92
        gcode_G92();
        break;
    }
    break;

    case 'M': switch (codenum) {
      #if ENABLED(ULTIPANEL)
        case 0: // M0 - Unconditional stop - Wait for user button press on LCD
        case 1: // M1 - Conditional stop - Wait for user button press on LCD
          gcode_M0_M1();
          break;
      #endif // ULTIPANEL

      case 17:
        gcode_M17();
        break;
       case 31: //M31 take time since the start of the SD print or an M109 command
        gcode_M31();
        break;

      case 42: //M42 -Change pin status via gcode
        gcode_M42();
        break;

      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: // M48 Z probe repeatability
          gcode_M48();
          break;
      #endif // Z_MIN_PROBE_REPEATABILITY_TEST

      case 75: // Start print timer
        gcode_M75();
        break;

      case 76: // Pause print timer
        gcode_M76();
        break;

      case 77: // Stop print timer
        gcode_M77();
        break;

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100:
          gcode_M100();
          break;
      #endif

      case 104: // M104
        gcode_M104();
        break;

      case 110: // M110: Set Current Line Number
        gcode_M110();
        break;

      case 111: // M111: Set debug level
        gcode_M111();
        break;
      case 108: // M108: Cancel Waiting
        gcode_M108();
        break;

      case 112: // M112: Emergency Stop
        gcode_M112();
        break;

      case 410: // M410 quickstop - Abort all the planned moves.
        gcode_M410();
        break;
      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: // M113: Set Host Keepalive interval
          gcode_M113();
          break;
      #endif

      case 140: // M140: Set bed temp
        gcode_M140();
        break;

      case 105: // M105: Read current temperature
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; // "ok" already printed

      case 109: // M109: Wait for temperature
        gcode_M109();
        break;

      #if HAS_TEMP_BED
        case 190: // M190: Wait for bed heater to reach target
          gcode_M190();
          break;
      #endif // HAS_TEMP_BED

      #if FAN_COUNT > 0
        case 106: // M106: Fan On
          gcode_M106();
          break;
        case 107: // M107: Fan Off
          gcode_M107();
          break;
      #endif // FAN_COUNT > 0

      #if HAS_POWER_SWITCH

        case 80: // M80: Turn on Power Supply
          gcode_M80();
          break;

      #endif // HAS_POWER_SWITCH

      case 81: // M81: Turn off Power, including Power Supply, if possible
        gcode_M81();
        break;

      case 82:
        gcode_M82();
        break;
      case 83:
        gcode_M83();
        break;
      case 18: // (for compatibility)
      case 84: // M84
        gcode_M18_M84();
        break;
      case 85: // M85
        gcode_M85();
        break;
      case 92: // M92: Set the steps-per-unit for one or more axes
        gcode_M92();
        break;
      case 115: // M115: Report capabilities
        gcode_M115();
        break;
      case 117: // M117: Set LCD message text, if possible
        gcode_M117();
        break;
      case 114: // M114: Report current position
        gcode_M114();
        break;
      case 120: // M120: Enable endstops
        gcode_M120();
        break;
      case 121: // M121: Disable endstops
        gcode_M121();
        break;
      case 119: // M119: Report endstop states
        gcode_M119();
        break;

      #if ENABLED(ULTIPANEL)

        case 145: // M145: Set material heatup parameters
          gcode_M145();
          break;

      #endif

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        case 149:
          gcode_M149();
          break;
      #endif
      case 200: // M200 D<diameter> Set filament diameter and set E axis units to cubic. (Use S0 to revert to linear units.)
        gcode_M200();
        break;
      case 201: // M201
        gcode_M201();
        break;
      #if 0 // Not used for Sprinter/grbl gen6
        case 202: // M202
          gcode_M202();
          break;
      #endif
      case 203: // M203 max feedrate units/sec
        gcode_M203();
        break;
      case 204: // M204 acclereration S normal moves T filmanent only moves
        gcode_M204();
        break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
        gcode_M205();
        break;
      case 206: // M206 additional homing offset
        gcode_M206();
        break;
      #if HOTENDS > 1
        case 218: // M218 - Set a tool offset: T<index> X<offset> Y<offset>
          gcode_M218();
          break;
      #endif

      case 220: // M220 - Set Feedrate Percentage: S<percent> ("FR" on your LCD)
        gcode_M220();
        break;

      case 221: // M221 - Set Flow Percentage: S<percent>
        gcode_M221();
        break;

      case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
        gcode_M226();
        break;

      #if HAS_BUZZER
        case 300: // M300 - Play beep tone
          gcode_M300();
          break;
      #endif // HAS_BUZZER

      #if ENABLED(PIDTEMP)
        case 301: // M301
          gcode_M301();
          break;
      #endif // PIDTEMP

      #if HAS_PHOTOGRAPH
        case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240();
          break;
      #endif // PHOTOGRAPH_PIN

      #if HAS_LCD_CONTRAST
        case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
          gcode_M250();
          break;
      #endif // HAS_LCD_CONTRAST

      #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
        case 302: // allow cold extrudes, or set the minimum extrude temperature
          gcode_M302();
          break;
      #endif // PREVENT_DANGEROUS_EXTRUDE

      case 303: // M303 PID autotune
        gcode_M303();
        break;

      #if ENABLED(SCARA)
        case 360:  // M360 SCARA Theta pos1
          if (gcode_M360()) return;
          break;
        case 361:  // M361 SCARA Theta pos2
          if (gcode_M361()) return;
          break;
        case 362:  // M362 SCARA Psi pos1
          if (gcode_M362()) return;
          break;
        case 363:  // M363 SCARA Psi pos2
          if (gcode_M363()) return;
          break;
        case 364:  // M364 SCARA Psi pos3 (90 deg to Theta)
          if (gcode_M364()) return;
          break;
        case 365: // M365 Set SCARA scaling for X Y Z
          gcode_M365();
          break;
      #endif // SCARA

      case 400: // M400 finish all moves
        gcode_M400();
        break;
      case 428: // M428 Apply current_position to home_offset
        gcode_M428();
        break;

      case 500: // M500 Store settings in EEPROM
        gcode_M500();
        break;
      case 501: // M501 Read settings from EEPROM
        gcode_M501();
        break;
      case 502: // M502 Revert to default settings
        gcode_M502();
        break;
      case 503: // M503 print settings currently in memory
        gcode_M503();
        break;

      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540:
          gcode_M540();
          break;
      #endif
      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907();
        break;

      #if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)

        case 908: // M908 Control digital trimpot directly.
          gcode_M908();
          break;

        #if ENABLED(DAC_STEPPER_CURRENT) // As with Printrbot RevF

          case 909: // M909 Print digipot/DAC current value
            gcode_M909();
            break;

          case 910: // M910 Commit digipot/DAC value to external EEPROM
            gcode_M910();
            break;

        #endif

      #endif // HAS_DIGIPOTSS || DAC_STEPPER_CURRENT

      #if HAS_MICROSTEPS

        case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350();
          break;

        case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351();
          break;

      #endif // HAS_MICROSTEPS

      case 999: // M999: Restart after being Stopped
        gcode_M999();
        break;
    }
    break;

    case 'T':
      gcode_T(codenum);
      break;

    default: code_is_good = false;
  }

  KEEPALIVE_STATE(NOT_BUSY);

ExitUnknownCommand:

  // Still unknown command? Throw an error
  if (!code_is_good) unknown_command_error();

  ok_to_send();
}

void FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ok_to_send();
}

void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_PROTOCOLPGM(MSG_OK);
  SERIAL_EOL;
}

void clamp_to_software_endstops(float target[3]) {
  if (min_software_endstops) {
    NOLESS(target[X_AXIS], sw_endstop_min[X_AXIS]);
    NOLESS(target[Y_AXIS], sw_endstop_min[Y_AXIS]);
    NOLESS(target[Z_AXIS], sw_endstop_min[Z_AXIS]);
  }
  if (max_software_endstops) {
    NOMORE(target[X_AXIS], sw_endstop_max[X_AXIS]);
    NOMORE(target[Y_AXIS], sw_endstop_max[Y_AXIS]);
    NOMORE(target[Z_AXIS], sw_endstop_max[Z_AXIS]);
  }
}

void set_current_from_steppers_for_axis(AxisEnum axis) {
  current_position[axis] = stepper.get_axis_position_mm(axis); // CORE handled transparently
}
#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)

  inline void prevent_dangerous_extrude(float& curr_e, float& dest_e) {
    if (DEBUGGING(DRYRUN)) return;
    float de = dest_e - curr_e;
    if (de) {
      if (thermalManager.tooColdToExtrude(active_extruder)) {
        curr_e = dest_e; // Behave as if the move really took place, but ignore E part
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
      }
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        if (labs(de) > EXTRUDE_MAXLENGTH) {
          curr_e = dest_e; // Behave as if the move really took place, but ignore E part
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
        }
      #endif
    }
  }

#endif // PREVENT_DANGEROUS_EXTRUDE

/**
 * Prepare a single move and get ready for the next one
 *
 */
void prepare_move_to_destination() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

  #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
    prevent_dangerous_extrude(current_position[E_AXIS], destination[E_AXIS]);
  #endif
  set_current_to_destination();
}

#if ENABLED(ARC_SUPPORT)
  /**
   * Plan an arc in 2 dimensions
   *
   * The arc is approximated by generating many small linear segments.
   * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
   * Arcs should only be made relatively large (over 5mm), as larger arcs with
   * larger segments will tend to be more efficient. Your slicer should have
   * options for G2/G3 arc generation. In future these options may be GCode tunable.
   */
  void plan_arc(
    float target[NUM_AXIS], // Destination position
    float* offset,          // Center of rotation relative to current_position
    uint8_t clockwise       // Clockwise?
  ) {

    float radius = HYPOT(offset[X_AXIS], offset[Y_AXIS]),
          center_X = current_position[X_AXIS] + offset[X_AXIS],
          center_Y = current_position[Y_AXIS] + offset[Y_AXIS],
          linear_travel = target[Z_AXIS] - current_position[Z_AXIS],
          extruder_travel = target[E_AXIS] - current_position[E_AXIS],
          r_X = -offset[X_AXIS],  // Radius vector from center to current location
          r_Y = -offset[Y_AXIS],
          rt_X = target[X_AXIS] - center_X,
          rt_Y = target[Y_AXIS] - center_Y;

    // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_X * rt_Y - r_Y * rt_X, r_X * rt_X + r_Y * rt_Y);
    if (angular_travel < 0) angular_travel += RADIANS(360);
    if (clockwise) angular_travel -= RADIANS(360);

    // Make a circle if the angular rotation is 0
    if (angular_travel == 0 && current_position[X_AXIS] == target[X_AXIS] && current_position[Y_AXIS] == target[Y_AXIS])
      angular_travel += RADIANS(360);

    float mm_of_travel = HYPOT(angular_travel * radius, fabs(linear_travel));
    if (mm_of_travel < 0.001) return;
    uint16_t segments = floor(mm_of_travel / (MM_PER_ARC_SEGMENT));
    if (segments == 0) segments = 1;

    float theta_per_segment = angular_travel / segments;
    float linear_per_segment = linear_travel / segments;
    float extruder_per_segment = extruder_travel / segments;

    /**
     * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
     *     r_T = [cos(phi) -sin(phi);
     *            sin(phi)  cos(phi] * r ;
     *
     * For arc generation, the center of the circle is the axis of rotation and the radius vector is
     * defined from the circle center to the initial position. Each line segment is formed by successive
     * vector rotations. This requires only two cos() and sin() computations to form the rotation
     * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     * all double numbers are single precision on the Arduino. (True double precision will not have
     * round off issues for CNC applications.) Single precision error can accumulate to be greater than
     * tool precision in some cases. Therefore, arc path correction is implemented.
     *
     * Small angle approximation may be used to reduce computation overhead further. This approximation
     * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
     * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
     * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     * issue for CNC machines with the single precision Arduino calculations.
     *
     * This approximation also allows plan_arc to immediately insert a line segment into the planner
     * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
     * This is important when there are successive arc motions.
     */
    // Vector rotation matrix values
    float cos_T = 1 - 0.5 * sq(theta_per_segment); // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[NUM_AXIS];
    float sin_Ti, cos_Ti, r_new_Y;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    arc_target[Z_AXIS] = current_position[Z_AXIS];

    // Initialize the extruder axis
    arc_target[E_AXIS] = current_position[E_AXIS];

    float fr_mm_s = MMM_TO_MMS_SCALED(feedrate_mm_m);

    millis_t next_idle_ms = millis() + 200UL;

    for (i = 1; i < segments; i++) { // Iterate (segments-1) times

      thermalManager.manage_heater();
      millis_t now = millis();
      if (ELAPSED(now, next_idle_ms)) {
        next_idle_ms = now + 200UL;
        idle();
      }

      if (++count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix to previous r_X / 1
        r_new_Y = r_X * sin_T + r_Y * cos_T;
        r_X = r_X * cos_T - r_Y * sin_T;
        r_Y = r_new_Y;
      }
      else {
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        // To reduce stuttering, the sin and cos could be computed at different times.
        // For now, compute both at the same time.
        cos_Ti = cos(i * theta_per_segment);
        sin_Ti = sin(i * theta_per_segment);
        r_X = -offset[X_AXIS] * cos_Ti + offset[Y_AXIS] * sin_Ti;
        r_Y = -offset[X_AXIS] * sin_Ti - offset[Y_AXIS] * cos_Ti;
        count = 0;
      }

      // Update arc_target location
      arc_target[X_AXIS] = center_X + r_X;
      arc_target[Y_AXIS] = center_Y + r_Y;
      arc_target[Z_AXIS] += linear_per_segment;
      arc_target[E_AXIS] += extruder_per_segment;
      clamp_to_software_endstops(arc_target);
      planner.buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], arc_target[E_AXIS], fr_mm_s, active_extruder);
    }

    // Ensure last segment arrives at target location.
    planner.buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], fr_mm_s, active_extruder);

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
  }
#endif
#if HAS_CONTROLLERFAN

  void controllerFan() {
    static millis_t lastMotorOn = 0; // Last time a motor was turned on
    static millis_t nextMotorCheck = 0; // Last time the state was checked
    millis_t ms = millis();
    if (ELAPSED(ms, nextMotorCheck)) {
      nextMotorCheck = ms + 2500UL; // Not a time critical function, so only check every 2.5s
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || thermalManager.soft_pwm_bed > 0
          || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
          #if E_STEPPERS > 1
            || E1_ENABLE_READ == E_ENABLE_ON
            #if HAS_X2_ENABLE
              || X2_ENABLE_READ == X_ENABLE_ON
            #endif
            #if E_STEPPERS > 2
              || E2_ENABLE_READ == E_ENABLE_ON
              #if E_STEPPERS > 3
                || E3_ENABLE_READ == E_ENABLE_ON
              #endif
            #endif
          #endif
      ) {
        lastMotorOn = ms; //... set time to NOW so the fan will turn on
      }

      // Fan off if no steppers have been enabled for CONTROLLERFAN_SECS seconds
      uint8_t speed = (!lastMotorOn || ELAPSED(ms, lastMotorOn + (CONTROLLERFAN_SECS) * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;

      // allows digital or PWM fan output to be used (see M42 handling)
      digitalWrite(CONTROLLERFAN_PIN, speed);
      analogWrite(CONTROLLERFAN_PIN, speed);
    }
  }

#endif // HAS_CONTROLLERFAN

#if ENABLED(SCARA)

  void forward_kinematics_SCARA(float f_scara[3]) {
    // Perform forward kinematics, and place results in delta[3]
    // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014

    float x_sin, x_cos, y_sin, y_cos;

    //SERIAL_ECHOPGM("f_delta x="); SERIAL_ECHO(f_scara[X_AXIS]);
    //SERIAL_ECHOPGM(" y="); SERIAL_ECHO(f_scara[Y_AXIS]);

    x_sin = sin(f_scara[X_AXIS] / SCARA_RAD2DEG) * Linkage_1;
    x_cos = cos(f_scara[X_AXIS] / SCARA_RAD2DEG) * Linkage_1;
    y_sin = sin(f_scara[Y_AXIS] / SCARA_RAD2DEG) * Linkage_2;
    y_cos = cos(f_scara[Y_AXIS] / SCARA_RAD2DEG) * Linkage_2;

    //SERIAL_ECHOPGM(" x_sin="); SERIAL_ECHO(x_sin);
    //SERIAL_ECHOPGM(" x_cos="); SERIAL_ECHO(x_cos);
    //SERIAL_ECHOPGM(" y_sin="); SERIAL_ECHO(y_sin);
    //SERIAL_ECHOPGM(" y_cos="); SERIAL_ECHOLN(y_cos);

    delta[X_AXIS] = x_cos + y_cos + SCARA_offset_x;  //theta
    delta[Y_AXIS] = x_sin + y_sin + SCARA_offset_y;  //theta+phi

    //SERIAL_ECHOPGM(" delta[X_AXIS]="); SERIAL_ECHO(delta[X_AXIS]);
    //SERIAL_ECHOPGM(" delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
  }

  void inverse_kinematics(const float cartesian[3]) {
    // Inverse kinematics.
    // Perform SCARA IK and place results in delta[3].
    // The maths and first version were done by QHARLEY.
    // Integrated, tweaked by Joachim Cerny in June 2014.

    float SCARA_pos[2];
    static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi;

    SCARA_pos[X_AXIS] = RAW_X_POSITION(cartesian[X_AXIS]) * axis_scaling[X_AXIS] - SCARA_offset_x;  //Translate SCARA to standard X Y
    SCARA_pos[Y_AXIS] = RAW_Y_POSITION(cartesian[Y_AXIS]) * axis_scaling[Y_AXIS] - SCARA_offset_y;  // With scaling factor.

    #if (Linkage_1 == Linkage_2)
      SCARA_C2 = ((sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS])) / (2 * (float)L1_2)) - 1;
    #else
      SCARA_C2 = (sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) - (float)L1_2 - (float)L2_2) / 45000;
    #endif

    SCARA_S2 = sqrt(1 - sq(SCARA_C2));

    SCARA_K1 = Linkage_1 + Linkage_2 * SCARA_C2;
    SCARA_K2 = Linkage_2 * SCARA_S2;

    SCARA_theta = (atan2(SCARA_pos[X_AXIS], SCARA_pos[Y_AXIS]) - atan2(SCARA_K1, SCARA_K2)) * -1;
    SCARA_psi = atan2(SCARA_S2, SCARA_C2);

    delta[X_AXIS] = SCARA_theta * SCARA_RAD2DEG;  // Multiply by 180/Pi  -  theta is support arm angle
    delta[Y_AXIS] = (SCARA_theta + SCARA_psi) * SCARA_RAD2DEG;  //       -  equal to sub arm angle (inverted motor)
    delta[Z_AXIS] = RAW_Z_POSITION(cartesian[Z_AXIS]);

    /**
    SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
    SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

    SERIAL_ECHOPGM("scara x="); SERIAL_ECHO(SCARA_pos[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHOLN(SCARA_pos[Y_AXIS]);

    SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
    SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);

    SERIAL_ECHOPGM("C2="); SERIAL_ECHO(SCARA_C2);
    SERIAL_ECHOPGM(" S2="); SERIAL_ECHO(SCARA_S2);
    SERIAL_ECHOPGM(" Theta="); SERIAL_ECHO(SCARA_theta);
    SERIAL_ECHOPGM(" Psi="); SERIAL_ECHOLN(SCARA_psi);
    SERIAL_EOL;
    */
  }

#endif // SCARA
void enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
  enable_e0();
  enable_e1();
  enable_e2();
  enable_e3();
}

void disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle() {
  lcd_update();
  host_keepalive();
  manage_inactivity();
  thermalManager.manage_heater();
  #if HAS_BUZZER && PIN_EXISTS(BEEPER)
    buzzer.tick();
  #endif
}

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

  if (commands_in_queue < BUFSIZE) get_available_commands();

  millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) kill(PSTR(MSG_KILLED));

  if (stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
      && !ignore_stepper_queue && !planner.blocks_queued()) {
    #if ENABLED(DISABLE_INACTIVE_X)
      disable_x();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Y)
      disable_y();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Z)
      disable_z();
    #endif
    #if ENABLED(DISABLE_INACTIVE_E)
      disable_e0();
      disable_e1();
      disable_e2();
      disable_e3();
    #endif
  }
  #if HAS_KILL

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
      killCount++;
    else if (killCount > 0)
      killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) kill(PSTR(MSG_KILLED));
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 2500;
    if (!READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enqueue_and_echo_commands_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif

  #if HAS_CONTROLLERFAN
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (ELAPSED(ms, previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
      && thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      #if ENABLED(SWITCHING_EXTRUDER)
        bool oldstatus = E0_ENABLE_READ;
        enable_e0();
      #else // !SWITCHING_EXTRUDER
        bool oldstatus;
        switch (active_extruder) {
          case 0:
            oldstatus = E0_ENABLE_READ;
            enable_e0();
            break;
          #if E_STEPPERS > 1
            case 1:
              oldstatus = E1_ENABLE_READ;
              enable_e1();
              break;
            #if E_STEPPERS > 2
              case 2:
                oldstatus = E2_ENABLE_READ;
                enable_e2();
                break;
              #if E_STEPPERS > 3
                case 3:
                  oldstatus = E3_ENABLE_READ;
                  enable_e3();
                  break;
              #endif
            #endif
          #endif
        }
      #endif // !SWITCHING_EXTRUDER

      float oldepos = current_position[E_AXIS], oldedes = destination[E_AXIS];
      planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                       destination[E_AXIS] + (EXTRUDER_RUNOUT_EXTRUDE) * (EXTRUDER_RUNOUT_ESTEPS) / planner.axis_steps_per_mm[E_AXIS],
                       MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED) * (EXTRUDER_RUNOUT_ESTEPS) / planner.axis_steps_per_mm[E_AXIS], active_extruder);
      current_position[E_AXIS] = oldepos;
      destination[E_AXIS] = oldedes;
      planner.set_e_position_mm(oldepos);
      previous_cmd_ms = ms; // refresh_cmd_timeout()
      stepper.synchronize();
      #if ENABLED(SWITCHING_EXTRUDER)
        E0_ENABLE_WRITE(oldstatus);
      #else
        switch (active_extruder) {
          case 0:
            E0_ENABLE_WRITE(oldstatus);
            break;
          #if E_STEPPERS > 1
            case 1:
              E1_ENABLE_WRITE(oldstatus);
              break;
            #if E_STEPPERS > 2
              case 2:
                E2_ENABLE_WRITE(oldstatus);
                break;
              #if E_STEPPERS > 3
                case 3:
                  E3_ENABLE_WRITE(oldstatus);
                  break;
              #endif
            #endif
          #endif
        }
      #endif // !SWITCHING_EXTRUDER
    }
  #endif // EXTRUDER_RUNOUT_PREVENT
  planner.check_axes_activity();
}

void kill(const char* lcd_msg) {
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

  #if ENABLED(ULTRA_LCD)
    kill_screen(lcd_msg);
  #else
    UNUSED(lcd_msg);
  #endif

  for (int i = 5; i--;) delay(100); // Wait a short time

  cli(); // Stop interrupts
  thermalManager.disable_all_heaters();
  disable_all_steppers();

  #if HAS_POWER_SWITCH
    pinMode(PS_ON_PIN, INPUT);
  #endif

  suicide();
  while (1) {
    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif
  } // Wait for reset
}

void stop() {
  thermalManager.disable_all_heaters();
  if (IsRunning()) {
    Running = false;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  float d2 = diameter * 0.5;
  return 1.0 / (M_PI * d2 * d2);
}

void calculate_volumetric_multipliers() {
  for (uint8_t i = 0; i < COUNT(filament_size); i++)
    volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}
