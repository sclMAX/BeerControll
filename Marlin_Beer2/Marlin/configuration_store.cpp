/**
 * configuration_store.cpp
 *
 * Configuration and EEPROM storage
 *
 * IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
 * in the functions below, also increment the version number. This makes sure that
 * the default values are used whenever there is a change to the data, to prevent
 * wrong data being written to the variables.
 *
 * ALSO: Variables in the Store and Retrieve sections must be in the same order.
 *       If a feature is disabled, some data must still be written that, when read,
 *       either sets a Sane Default, or results in No Change to the existing value.
 *
 */

#define EEPROM_VERSION "V24"

// Change EEPROM version if these are changed:
#define EEPROM_OFFSET 100
#define MAX_EXTRUDERS 4
#include "Marlin.h"
#include "language.h"
#include "temperature.h"
#include "ultralcd.h"
#include "configuration_store.h"

uint16_t eeprom_checksum;
const char version[4] = EEPROM_VERSION;

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size) {
  uint8_t c;
  while (size--) {
    eeprom_write_byte((unsigned char*)pos, *value);
    c = eeprom_read_byte((unsigned char*)pos);
    if (c != *value) {
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ERR_EEPROM_WRITE);
    }
    eeprom_checksum += c;
    pos++;
    value++;
  };
}
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size) {
  do {
    uint8_t c = eeprom_read_byte((unsigned char*)pos);
    *value = c;
    eeprom_checksum += c;
    pos++;
    value++;
  } while (--size);
}

/**
 * Post-process after Retrieve or Reset
 */
void Config_Postprocess() {
  #if ENABLED(PIDTEMP)
    thermalManager.updatePID();
  #endif
}

#if ENABLED(EEPROM_SETTINGS)
  #define DUMMY_PID_VALUE 3000.0f
  #define EEPROM_START() int eeprom_index = EEPROM_OFFSET
  #define EEPROM_SKIP(VAR) eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR) _EEPROM_writeData(eeprom_index, (uint8_t*)&VAR, sizeof(VAR))
  #define EEPROM_READ(VAR) _EEPROM_readData(eeprom_index, (uint8_t*)&VAR, sizeof(VAR))

  /**
  * M500 - Store Configuration
  */
void Config_StoreSettings()  {
  float dummy = 0.0f;
  char ver[4] = "000";
  EEPROM_START();
  EEPROM_WRITE(ver);     // invalidate data first
  EEPROM_SKIP(eeprom_checksum); // Skip the checksum slot
  eeprom_checksum = 0; // clear before first "real data"
  EEPROM_WRITE(tempObjMacerador);
  EEPROM_WRITE(tempObjLicor);
  EEPROM_WRITE(tempObjHervido);
  for(int idx = 0; idx < etapas; idx++){
    EEPROM_WRITE(MaceradorTemp[idx]);
    EEPROM_WRITE(LicorTemp[idx]);
    EEPROM_WRITE(Inicio[idx]);
    EEPROM_WRITE(Duracion[idx]);
  }
  EEPROM_WRITE(E1Recircula);
  EEPROM_WRITE(E2Recircula);
  EEPROM_WRITE(E3Recircula);
  EEPROM_WRITE(E4Recircula);
  for (uint8_t e = 0; e < MAX_EXTRUDERS; e++) {
    #if ENABLED(PIDTEMP)
      if (e < HOTENDS) {
        EEPROM_WRITE(PID_PARAM(Kp, e));
        EEPROM_WRITE(PID_PARAM(Ki, e));
        EEPROM_WRITE(PID_PARAM(Kd, e));
        dummy = 1.0f; // 1.0 = default kc
        EEPROM_WRITE(dummy);
      }
      else
    #endif // !PIDTEMP
      {
        dummy = DUMMY_PID_VALUE; // When read, will not change the existing value
        EEPROM_WRITE(dummy); // Kp
        dummy = 0.0f;
        for (uint8_t q = 3; q--;) EEPROM_WRITE(dummy); // Ki, Kd, Kc
      }

  } // Hotends Loop
  int lpq_len = 20;
  EEPROM_WRITE(lpq_len);

  #if DISABLED(PIDTEMPBED)
    dummy = DUMMY_PID_VALUE;
    for (uint8_t q = 3; q--;) EEPROM_WRITE(dummy);
  #else
    EEPROM_WRITE(thermalManager.bedKp);
    EEPROM_WRITE(thermalManager.bedKi);
    EEPROM_WRITE(thermalManager.bedKd);
  #endif
  EEPROM_WRITE(lcd_contrast);
  dummy = 1.0f;
  EEPROM_WRITE(dummy);
  uint16_t final_checksum = eeprom_checksum,
           eeprom_size = eeprom_index;

  eeprom_index = EEPROM_OFFSET;
  EEPROM_WRITE(version);
  EEPROM_WRITE(final_checksum);

  // Report storage size
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("Settings Stored (", eeprom_size);
  SERIAL_ECHOLNPGM(" bytes)");
}

/**
 * M501 - Retrieve Configuration
 */
void Config_RetrieveSettings() {
  EEPROM_START();
  char stored_ver[4];
  EEPROM_READ(stored_ver);
  uint16_t stored_checksum;
  EEPROM_READ(stored_checksum);
  //  SERIAL_ECHOPAIR("Version: [", ver);
  //  SERIAL_ECHOPAIR("] Stored version: [", stored_ver);
  //  SERIAL_ECHOLNPGM("]");
  if (strncmp(version, stored_ver, 3) != 0) {
    Config_ResetDefault();
  }
  else {
    float dummy = 0;
    eeprom_checksum = 0; // clear before reading first "real data"
    EEPROM_READ(tempObjMacerador);
    EEPROM_READ(tempObjLicor);
    EEPROM_READ(tempObjHervido);
    for(int idx = 0; idx < etapas; idx++){
      EEPROM_READ(MaceradorTemp[idx]);
      EEPROM_READ(LicorTemp[idx]);
      EEPROM_READ(Inicio[idx]);
      EEPROM_READ(Duracion[idx]);
    }
    EEPROM_READ(E1Recircula);
    EEPROM_READ(E2Recircula);
    EEPROM_READ(E3Recircula);
    EEPROM_READ(E4Recircula);
    #if ENABLED(PIDTEMP)
      for (uint8_t e = 0; e < MAX_EXTRUDERS; e++) {
        EEPROM_READ(dummy); // Kp
        if (e < HOTENDS && dummy != DUMMY_PID_VALUE) {
          // do not need to scale PID values as the values in EEPROM are already scaled
          PID_PARAM(Kp, e) = dummy;
          EEPROM_READ(PID_PARAM(Ki, e));
          EEPROM_READ(PID_PARAM(Kd, e));
          EEPROM_READ(dummy);
        }
        else {
          for (uint8_t q=3; q--;) EEPROM_READ(dummy); // Ki, Kd, Kc
        }
      }
    #else // !PIDTEMP
      // 4 x 4 = 16 slots for PID parameters
      for (uint8_t q = MAX_EXTRUDERS * 4; q--;) EEPROM_READ(dummy);  // Kp, Ki, Kd, Kc
    #endif // !PIDTEMP
    int lpq_len;
    EEPROM_READ(lpq_len);

    #if ENABLED(PIDTEMPBED)
      EEPROM_READ(dummy); // bedKp
      if (dummy != DUMMY_PID_VALUE) {
        thermalManager.bedKp = dummy;
        EEPROM_READ(thermalManager.bedKi);
        EEPROM_READ(thermalManager.bedKd);
      }
    #else
      for (uint8_t q=3; q--;) EEPROM_READ(dummy); // bedKp, bedKi, bedKd
    #endif
    EEPROM_READ(lcd_contrast);   
    EEPROM_READ(dummy);
    if (eeprom_checksum == stored_checksum) {
      Config_Postprocess();
      SERIAL_ECHO_START;
      SERIAL_ECHO(version);
      SERIAL_ECHOPAIR(" stored settings retrieved (", eeprom_index);
      SERIAL_ECHOLNPGM(" bytes)");
    }
    else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("EEPROM checksum mismatch");
      Config_ResetDefault();
    }
 }

  #if ENABLED(EEPROM_CHITCHAT)
    Config_PrintSettings();
  #endif
}

#endif // EEPROM_SETTINGS

/**
 * M502 - Reset Configuration
 */
void Config_ResetDefault() {
  tempObjMacerador = 64;
  tempObjLicor = 75;
  tempObjHervido = 110;
  #if ENABLED(PIDTEMP)
    #if ENABLED(PID_PARAMS_PER_HOTEND) && HOTENDS > 1
      HOTEND_LOOP()
    #else
      int e = 0; UNUSED(e); // only need to write once
    #endif
    {
      PID_PARAM(Kp, e) = DEFAULT_Kp;
      PID_PARAM(Ki, e) = scalePID_i(DEFAULT_Ki);
      PID_PARAM(Kd, e) = scalePID_d(DEFAULT_Kd);
    }
  #endif // PIDTEMP

  #if ENABLED(PIDTEMPBED)
    thermalManager.bedKp = DEFAULT_bedKp;
    thermalManager.bedKi = scalePID_i(DEFAULT_bedKi);
    thermalManager.bedKd = scalePID_d(DEFAULT_bedKd);
  #endif
  Config_Postprocess();
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
}

#if DISABLED(DISABLE_M503)

#define CONFIG_ECHO_START do{ if (!forReplay) SERIAL_ECHO_START; }while(0)

/**
 * M503 - Print Configuration
 */
void Config_PrintSettings(bool forReplay) {
  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown

  CONFIG_ECHO_START;

  if (!forReplay) {
    SERIAL_ECHOLNPGM("Steps per unit:");
    CONFIG_ECHO_START;
  }
  SERIAL_EOL;
  #if ENABLED(ULTIPANEL)
    CONFIG_ECHO_START;
    if (!forReplay) {
      SERIAL_ECHOLNPGM("Material heatup parameters:");
      CONFIG_ECHO_START;
    }
    SERIAL_ECHOPAIR("  M145 S0 H", 0);
    SERIAL_ECHOPAIR(" B", 0);
    SERIAL_ECHOPAIR(" F", 0);
    SERIAL_EOL;
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M145 S1 H", 0);
    SERIAL_ECHOPAIR(" B", 0);
    SERIAL_ECHOPAIR(" F", 0);
    SERIAL_EOL;
  #endif // ULTIPANEL

  #if HAS_PID_HEATING

    CONFIG_ECHO_START;
    if (!forReplay) {
      SERIAL_ECHOLNPGM("PID settings:");
    }
    #if ENABLED(PIDTEMP)
      #if HOTENDS > 1
        if (forReplay) {
          HOTEND_LOOP() {
            CONFIG_ECHO_START;
            SERIAL_ECHOPAIR("  M301 E", e);
            SERIAL_ECHOPAIR(" P", PID_PARAM(Kp, e));
            SERIAL_ECHOPAIR(" I", unscalePID_i(PID_PARAM(Ki, e)));
            SERIAL_ECHOPAIR(" D", unscalePID_d(PID_PARAM(Kd, e)));
            SERIAL_EOL;
          }
        }
        else
      #endif // HOTENDS > 1
      // !forReplay || HOTENDS == 1
      {
        CONFIG_ECHO_START;
        SERIAL_ECHOPAIR("  M301 P", PID_PARAM(Kp, 0)); // for compatibility with hosts, only echo values for E0
        SERIAL_ECHOPAIR(" I", unscalePID_i(PID_PARAM(Ki, 0)));
        SERIAL_ECHOPAIR(" D", unscalePID_d(PID_PARAM(Kd, 0)));
        SERIAL_EOL;
      }
    #endif // PIDTEMP

    #if ENABLED(PIDTEMPBED)
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M304 P", thermalManager.bedKp);
      SERIAL_ECHOPAIR(" I", unscalePID_i(thermalManager.bedKi));
      SERIAL_ECHOPAIR(" D", unscalePID_d(thermalManager.bedKd));
      SERIAL_EOL;
    #endif

  #endif // PIDTEMP || PIDTEMPBED
}

#endif // !DISABLE_M503
