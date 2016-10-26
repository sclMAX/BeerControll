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

/**
 * V24 EEPROM Layout:
 *
 *  100  Version (char x4)
 *  104  EEPROM Checksum (uint16_t)
 *
 * Mesh bed leveling:
 *  202  M420 S    status (uint8)
 *  203            z_offset (float)
 *  207            mesh_num_x (uint8 as set in firmware)
 *  208            mesh_num_y (uint8 as set in firmware)
 *  209 G29 S3 XYZ z_values[][] (float x9, by default, up to float x 81)
 *
 * AUTO BED LEVELING
 *  245  M851      zprobe_zoffset (float)
 * *
 * ULTIPANEL:
 *  289  M145 S0 H preheatHotendTemp1 (int)
 *  291  M145 S0 B preheatBedTemp1 (int)
 *  293  M145 S0 F preheatFanSpeed1 (int)
 *  295  M145 S1 H preheatHotendTemp2 (int)
 *  297  M145 S1 B preheatBedTemp2 (int)
 *  299  M145 S1 F preheatFanSpeed2 (int)
 *
 * PIDTEMP:
 *  301  M301 E0 PIDC  Kp[0], Ki[0], Kd[0], Kc[0] (float x4)
 *  317  M301 E1 PIDC  Kp[1], Ki[1], Kd[1], Kc[1] (float x4)
 *  333  M301 E2 PIDC  Kp[2], Ki[2], Kd[2], Kc[2] (float x4)
 *  349  M301 E3 PIDC  Kp[3], Ki[3], Kd[3], Kc[3] (float x4)
 *  365  M301 L        lpq_len (int)
 *
 *  367  M304 PID  thermalManager.bedKp, thermalManager.bedKi, thermalManager.bedKd (float x3)
 *
 * DOGLCD:
 *  379  M250 C    lcd_contrast (int)
 *  393  M209 S    autoretract_enabled (bool)
 *  394  M207 S    retract_length (float)
 *  398  M207 W    retract_length_swap (float)
 *  402  M207 F    retract_feedrate_mm_s (float)
 *  406  M207 Z    retract_zlift (float)
 *  410  M208 S    retract_recover_length (float)
 *  414  M208 W    retract_recover_length_swap (float)
 *  418  M208 F    retract_recover_feedrate_mm_s (float)
 *
 */
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
 * M502 - Reset Configuration
 */
void Config_ResetDefault() {
  float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  float tmp2[] = DEFAULT_MAX_FEEDRATE;
  long tmp3[] = DEFAULT_MAX_ACCELERATION;
  #if ENABLED(ULTIPANEL)
    preheatHotendTemp1 = PREHEAT_1_TEMP_HOTEND;
    preheatBedTemp1 = PREHEAT_1_TEMP_BED;
    preheatFanSpeed1 = PREHEAT_1_FAN_SPEED;
    preheatHotendTemp2 = PREHEAT_2_TEMP_HOTEND;
    preheatBedTemp2 = PREHEAT_2_TEMP_BED;
    preheatFanSpeed2 = PREHEAT_2_FAN_SPEED;
  #endif
  #if HAS_LCD_CONTRAST
    lcd_contrast = DEFAULT_LCD_CONTRAST;
  #endif
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
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
}
