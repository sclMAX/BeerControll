/**
 * Conditionals_LCD.h
 * LCD Defines that depend on configuration but are not editable.
 */

#ifndef CONDITIONALS_LCD_H // Get the LCD defines which are needed first
#define CONDITIONALS_LCD_H

  #define LCD_HAS_DIRECTIONAL_BUTTONS (BUTTON_EXISTS(UP) || BUTTON_EXISTS(DWN) || BUTTON_EXISTS(LFT) || BUTTON_EXISTS(RT))
  #define DOGLCD
  #define U8GLIB_ST7920
  #define REPRAP_DISCOUNT_SMART_CONTROLLER
  #define ULTIPANEL
  #define NEWPANEL
  #ifndef LCD_WIDTH
    #define LCD_WIDTH 22
  #endif
  #ifndef LCD_HEIGHT
    #define LCD_HEIGHT 5
  #endif
  #define NEWPANEL  //enable this if you have a click-encoder panel
  #define ULTRA_LCD
  #ifndef LCD_WIDTH
    #define LCD_WIDTH 20
  #endif
  #ifndef LCD_HEIGHT
    #define LCD_HEIGHT 4
  #endif
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
  #ifndef BOOTSCREEN_TIMEOUT
    #define BOOTSCREEN_TIMEOUT 2500
  #endif
  #define HOTENDS      EXTRUDERS
  #define E_STEPPERS   EXTRUDERS
  #define E_MANUAL     EXTRUDERS
  #define TOOL_E_INDEX current_block->active_extruder

#endif //CONDITIONALS_LCD_H
