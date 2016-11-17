#ifndef MARLIN_H
#define MARLIN_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "MarlinConfig.h"
#include "enum.h"
#include "types.h"
#include "fastio.h"
#include "utility.h"
#include "WString.h"
#include "stopwatch.h"
void idle();
extern uint8_t marlin_debug_flags;
#define DEBUGGING(F) (marlin_debug_flags & (DEBUG_## F))

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }
extern volatile bool wait_for_heatup;

// Print job timer
extern Stopwatch print_job_timer;

// Handling multiple extruders pins
extern uint8_t active_extruder;
// Buzzer
#if HAS_BUZZER && PIN_EXISTS(BEEPER)
  #include "buzzer.h"
#endif
#endif //MARLIN_H
