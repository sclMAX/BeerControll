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

#if ENABLED(SDSUPPORT)
  CardReader card;
#endif
bool Running = true;

uint8_t marlin_debug_flags = DEBUG_NONE;
static char command_queue[BUFSIZE][MAX_CMD_SIZE];
static char* current_command, *current_command_args;
static uint8_t cmd_queue_index_r = 0,
               cmd_queue_index_w = 0,
               commands_in_queue = 0;
TempUnit input_temp_units = TEMPUNIT_C;

volatile bool wait_for_heatup = true;

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";

static int serial_count = 0;

const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42
static millis_t max_inactive_time = 0;

// Print Job Timer
Stopwatch print_job_timer = Stopwatch();
// Buzzer - I2C on the LCD or a BEEPER_PIN
#if ENABLED(LCD_USE_I2C_BUZZER)
  #define BUZZ(d,f) lcd_buzz(d, f)
#elif HAS_BUZZER
  Buzzer buzzer;
  #define BUZZ(d,f) buzzer.tone(d, f)
#else
  #define BUZZ(d,f) NOOP
#endif

static uint8_t target_extruder;
static bool send_ok[BUFSIZE];

#ifdef CHDK
  millis_t chdkHigh = 0;
  boolean chdkActive = false;
#endif
#define host_keepalive() ;
#define KEEPALIVE_STATE(n) ;

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

#if ENABLED(SDSUPPORT)
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
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
#endif //!SDSUPPORT
void setup_powerhold() {
  #if HAS_POWER_SWITCH
    #if ENABLED(PS_DEFAULT_OFF)
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  #endif
}

void setup() {
  setup_powerhold();
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
void loop() {
  idle();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle() {
  lcd_update();
  host_keepalive();
  thermalManager.manage_heater();
  #if HAS_BUZZER && PIN_EXISTS(BEEPER)
    buzzer.tick();
  #endif
}
