#include "Marlin.h"
#include "ultralcd.h"
#include "temperature.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "duration_t.h"
#include "types.h"

#if ENABLED(USE_WATCHDOG)
  #include "watchdog.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif
bool Running = true;
uint8_t marlin_debug_flags = DEBUG_NONE;
static char command_queue[BUFSIZE][MAX_CMD_SIZE];
static char* current_command, *current_command_args;
static uint8_t cmd_queue_index_r = 0,
               cmd_queue_index_w = 0,
               commands_in_queue = 0;
// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder = 0;
volatile bool wait_for_heatup = true;
const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";
static int serial_count = 0;
// GCode parameter pointer used by code_seen(), code_value_float(), etc.
static char* seen_pointer;

// Next Immediate GCode Command pointer. NULL if none.
const char* queued_commands_P = NULL;

const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42

// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
Stopwatch print_job_timer = Stopwatch();

// Buzzer - I2C on the LCD or a BEEPER_PIN
#if HAS_BUZZER
  Buzzer buzzer;
  #define BUZZ(d,f) buzzer.tone(d, f)
#else
  #define BUZZ(d,f) NOOP
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
#if ENABLED(ULTIPANEL) && HAS_POWER_SWITCH
  bool powersupply = true;
#endif
static bool send_ok[BUFSIZE];
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
void serial_echopair_P(const char* s_P, char v)          { serialprintPGM(s_P); SERIAL_CHAR(v); }
void serial_echopair_P(const char* s_P, int v)           { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, long v)          { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, float v)         { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, double v)        { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, unsigned long v) { serialprintPGM(s_P); SERIAL_ECHO(v); }

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
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN, HIGH);
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

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
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
  setup_killpin();
  setup_powerhold();
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
  // Send "ok" after commands by default
  for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

  // Load data from EEPROM if available (or use defaults)
  Config_RetrieveSettings();
  thermalManager.init();    // Initialize temperature loop
  watchdog_init();
  lcd_init();
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD)
      safe_delay(BOOTSCREEN_TIMEOUT);
    #elif ENABLED(ULTRA_LCD)
      bootscreen();
      lcd_init();
    #endif
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
  idle();
}

inline void get_serial_commands() {
 
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
inline float code_value_linear_units() { return code_value_float(); }
inline float code_value_axis_units(int axis) { UNUSED(axis); return code_value_float(); }
inline float code_value_per_axis_unit(int axis) { UNUSED(axis); return code_value_float(); }
float code_value_temp_abs() { return code_value_float(); }
float code_value_temp_diff() { return code_value_float(); }
FORCE_INLINE millis_t code_value_millis() { return code_value_ulong(); }
inline millis_t code_value_millis_from_seconds() { return code_value_float() * 1000; }
bool code_seen(char code) {
  seen_pointer = strchr(current_command_args, code);
  return (seen_pointer != NULL); // Return TRUE if the code-letter was found
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
    }

    lcd_ignore_click();
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
    LCD_MESSAGEPGM(WELCOME_MSG);
  }

#endif // ULTIPANEL
#if HAS_TEMP_HOTEND || HAS_TEMP_BED

  void print_heaterstates() {
    uint8_t target_extruder = 1;
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

#ifndef MIN_COOLING_SLOPE_DEG
  #define MIN_COOLING_SLOPE_DEG 1.50
#endif
#ifndef MIN_COOLING_SLOPE_TIME
  #define MIN_COOLING_SLOPE_TIME 60
#endif

#if HAS_TEMP_BED
  #ifndef MIN_COOLING_SLOPE_DEG_BED
    #define MIN_COOLING_SLOPE_DEG_BED 1.50
  #endif
  #ifndef MIN_COOLING_SLOPE_TIME_BED
    #define MIN_COOLING_SLOPE_TIME_BED 60
  #endif
#endif // HAS_TEMP_BED
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
      SERIAL_EOL;
    }
    else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLN(MSG_INVALID_EXTRUDER);
    }
  }

#endif // PIDTEMP
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

inline void invalid_extruder_error(const uint8_t &e) {
  SERIAL_ECHO_START;
  SERIAL_CHAR('T');
  SERIAL_PROTOCOL_F(e, DEC);
  SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
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
  ok_to_send();
}

void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_PROTOCOLPGM(MSG_OK);
  SERIAL_EOL;
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
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {
  if (commands_in_queue < BUFSIZE) get_available_commands();
  millis_t ms = millis();
  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) kill(PSTR(MSG_KILLED));
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
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}
