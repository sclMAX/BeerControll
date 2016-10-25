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
 * Conditionals_LCD.h
 * LCD Defines that depend on configuration but are not editable.
 */

#ifndef CONDITIONALS_LCD_H // Get the LCD defines which are needed first
  #define CONDITIONALS_LCD_H

  #define LCD_HAS_DIRECTIONAL_BUTTONS (BUTTON_EXISTS(UP) || BUTTON_EXISTS(DWN) || BUTTON_EXISTS(LFT) || BUTTON_EXISTS(RT))

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define DOGLCD
    #define U8GLIB_ST7920
    #define REPRAP_DISCOUNT_SMART_CONTROLLER
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  #if ENABLED(DOGLCD) // Change number of lines to match the DOG graphic display
    #ifndef LCD_WIDTH
      #define LCD_WIDTH 22
    #endif
    #ifndef LCD_HEIGHT
      #define LCD_HEIGHT 5
    #endif
  #endif

  #if ENABLED(ULTIPANEL)
    #define NEWPANEL  //enable this if you have a click-encoder panel
    #define ULTRA_LCD
    #ifndef LCD_WIDTH
      #define LCD_WIDTH 20
    #endif
    #ifndef LCD_HEIGHT
      #define LCD_HEIGHT 4
    #endif
  #else //no panel but just LCD
    #if ENABLED(ULTRA_LCD)
      #ifndef LCD_WIDTH
        #define LCD_WIDTH 16
      #endif
      #ifndef LCD_HEIGHT
        #define LCD_HEIGHT 2
      #endif
    #endif
  #endif

  #if ENABLED(DOGLCD)
    /* Custom characters defined in font dogm_font_data_Marlin_symbols.h / Marlin_symbols.fon */
    // \x00 intentionally skipped to avoid problems in strings
    #define LCD_STR_REFRESH     "\x01"
    #define LCD_STR_FOLDER      "\x02"
    #define LCD_STR_ARROW_RIGHT "\x03"
    #define LCD_STR_UPLEVEL     "\x04"
    #define LCD_STR_CLOCK       "\x05"
    #define LCD_STR_FEEDRATE    "\x06"
    #define LCD_STR_BEDTEMP     "\x07"
    #define LCD_STR_THERMOMETER "\x08"
    #define LCD_STR_DEGREE      "\x09"

    #define LCD_STR_SPECIAL_MAX '\x09'
    // Maximum here is 0x1f because 0x20 is ' ' (space) and the normal charsets begin.
    // Better stay below 0x10 because DISPLAY_CHARSET_HD44780_WESTERN begins here.
  #else
    /* Custom characters defined in the first 8 characters of the LCD */
    #define LCD_STR_BEDTEMP     "\x00"  // Print only as a char. This will have 'unexpected' results when used in a string!
    #define LCD_STR_DEGREE      "\x01"
    #define LCD_STR_THERMOMETER "\x02"
    #define LCD_STR_UPLEVEL     "\x03"
    #define LCD_STR_REFRESH     "\x04"
    #define LCD_STR_FOLDER      "\x05"
    #define LCD_STR_FEEDRATE    "\x06"
    #define LCD_STR_CLOCK       "\x07"
    #define LCD_STR_ARROW_RIGHT ">"  /* from the default character set */
  #endif

  /**
   * Default LCD contrast for dogm-like LCD displays
   */
  #if ENABLED(DOGLCD)
    #define HAS_LCD_CONTRAST false
    #if HAS_LCD_CONTRAST
      #ifndef LCD_CONTRAST_MIN
        #define LCD_CONTRAST_MIN 0
      #endif
      #ifndef LCD_CONTRAST_MAX
        #define LCD_CONTRAST_MAX 63
      #endif
      #ifndef DEFAULT_LCD_CONTRAST
        #define DEFAULT_LCD_CONTRAST 32
      #endif
    #endif
  #endif

  #ifndef BOOTSCREEN_TIMEOUT
    #define BOOTSCREEN_TIMEOUT 2500
  #endif
  #define HOTENDS      EXTRUDERS
  #define E_STEPPERS   EXTRUDERS
  #define E_MANUAL     EXTRUDERS
  #define TOOL_E_INDEX current_block->active_extruder
#endif //CONDITIONALS_LCD_H
