#ifndef ULTRALCD_IMPL_HD44780_H
#define ULTRALCD_IMPL_HD44780_H

/**
* Implementation of the LCD display routines for a Hitachi HD44780 display. These are common LCD character displays.
**/

#include "duration_t.h"

extern volatile uint8_t buttons;  //an extended version of the last checked buttons in a bit array.

////////////////////////////////////
// Create LCD class instance and chipset-specific information
#if ENABLED(LCD_I2C_TYPE_PCA8574)
  #include <LiquidCrystal_I2C.h>
  #define LCD_CLASS LiquidCrystal_I2C
  LCD_CLASS lcd(LCD_I2C_ADDRESS, LCD_WIDTH, LCD_HEIGHT);

// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection
#elif ENABLED(SR_LCD_2W_NL)
  extern "C" void __cxa_pure_virtual() { while (1); }
  #include <LCD.h>
  #include <LiquidCrystal_SR.h>
  #define LCD_CLASS LiquidCrystal_SR
  LCD_CLASS lcd(SR_DATA_PIN, SR_CLK_PIN);
#else
  // Standard directly connected LCD implementations
  #include <LiquidCrystal.h>
  #define LCD_CLASS LiquidCrystal
  LCD_CLASS lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7); //RS,Enable,D4,D5,D6,D7
#endif

#include "utf_mapper.h"

#if ENABLED(LCD_HAS_STATUS_INDICATORS)
  static void lcd_implementation_update_indicators();
#endif

static void lcd_set_custom_characters() {
  byte bedTemp[8] = {
    B00000,
    B11111,
    B10101,
    B10001,
    B10101,
    B11111,
    B00000,
    B00000
  }; //thanks Sonny Mounicou
  byte degree[8] = {
    B01100,
    B10010,
    B10010,
    B01100,
    B00000,
    B00000,
    B00000,
    B00000
  };
  byte thermometer[8] = {
    B00100,
    B01010,
    B01010,
    B01010,
    B01010,
    B10001,
    B10001,
    B01110
  };
  byte uplevel[8] = {
    B00100,
    B01110,
    B11111,
    B00100,
    B11100,
    B00000,
    B00000,
    B00000
  }; //thanks joris
  byte refresh[8] = {
    B00000,
    B00110,
    B11001,
    B11000,
    B00011,
    B10011,
    B01100,
    B00000,
  }; //thanks joris
  byte folder[8] = {
    B00000,
    B11100,
    B11111,
    B10001,
    B10001,
    B11111,
    B00000,
    B00000
  }; //thanks joris
  byte feedrate[8] = {
    B11100,
    B10000,
    B11000,
    B10111,
    B00101,
    B00110,
    B00101,
    B00000
  }; //thanks Sonny Mounicou
  byte clock[8] = {
    B00000,
    B01110,
    B10011,
    B10101,
    B10001,
    B01110,
    B00000,
    B00000
  }; //thanks Sonny Mounicou
  lcd.createChar(LCD_STR_BEDTEMP[0], bedTemp);
  lcd.createChar(LCD_STR_DEGREE[0], degree);
  lcd.createChar(LCD_STR_THERMOMETER[0], thermometer);
  lcd.createChar(LCD_STR_UPLEVEL[0], uplevel);
  lcd.createChar(LCD_STR_REFRESH[0], refresh);
  lcd.createChar(LCD_STR_FOLDER[0], folder);
  lcd.createChar(LCD_STR_FEEDRATE[0], feedrate);
  lcd.createChar(LCD_STR_CLOCK[0], clock);
}

static void lcd_implementation_init() {
  #if ENABLED(LCD_I2C_TYPE_PCA8574)
    lcd.init();
    lcd.backlight();
  #else
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  #endif
  lcd_set_custom_characters();
  lcd.clear();
}

static void lcd_implementation_clear() { lcd.clear(); }

/* Arduino < 1.0.0 is missing a function to print PROGMEM strings, so we need to implement our own */
char lcd_printPGM(const char* str) {
  char c, n = 0;
  while ((c = pgm_read_byte(str++))) n += charset_mapper(c);
  return n;
}

char lcd_print(const char* str) {
  char c, n = 0;
  unsigned char i = 0;
  while ((c = str[i++])) n += charset_mapper(c);
  return n;
}

unsigned lcd_print(char c) { return charset_mapper(c); }

#if ENABLED(SHOW_BOOTSCREEN)

  void lcd_erase_line(int line) {
    lcd.setCursor(0, line);
    for (int i = 0; i < LCD_WIDTH; i++)
      lcd_print(' ');
  }

  // Scroll the PSTR 'text' in a 'len' wide field for 'time' milliseconds at position col,line
  void lcd_scroll(int col, int line, const char* text, int len, int time) {
    char tmp[LCD_WIDTH + 1] = {0};
    int n = max(lcd_strlen_P(text) - len, 0);
    for (int i = 0; i <= n; i++) {
      strncpy_P(tmp, text + i, min(len, LCD_WIDTH));
      lcd.setCursor(col, line);
      lcd_print(tmp);
      delay(time / max(n, 1));
    }
  }

  static void logo_lines(const char *extra) {
    int indent = (LCD_WIDTH - 8 - lcd_strlen_P(extra)) / 2;
    lcd.setCursor(indent, 0); lcd.print('\x00'); lcd_printPGM(PSTR( "------" ));  lcd.print('\x01');
    lcd.setCursor(indent, 1);                    lcd_printPGM(PSTR("|Marlin|"));  lcd_printPGM(extra);
    lcd.setCursor(indent, 2); lcd.print('\x02'); lcd_printPGM(PSTR( "------" ));  lcd.print('\x03');
  }

  void bootscreen() {
    byte top_left[8] = {
      B00000,
      B00000,
      B00000,
      B00000,
      B00001,
      B00010,
      B00100,
      B00100
    };
    byte top_right[8] = {
      B00000,
      B00000,
      B00000,
      B11100,
      B11100,
      B01100,
      B00100,
      B00100
    };
    byte botom_left[8] = {
      B00100,
      B00010,
      B00001,
      B00000,
      B00000,
      B00000,
      B00000,
      B00000
    };
    byte botom_right[8] = {
      B00100,
      B01000,
      B10000,
      B00000,
      B00000,
      B00000,
      B00000,
      B00000
    };
    lcd.createChar(0, top_left);
    lcd.createChar(1, top_right);
    lcd.createChar(2, botom_left);
    lcd.createChar(3, botom_right);

    lcd.clear();

    #define LCD_EXTRA_SPACE (LCD_WIDTH-8)

    #define CENTER_OR_SCROLL(STRING,DELAY) \
      lcd_erase_line(3); \
      if (strlen(STRING) <= LCD_WIDTH) { \
        lcd.setCursor((LCD_WIDTH - lcd_strlen_P(PSTR(STRING))) / 2, 3); \
        lcd_printPGM(PSTR(STRING)); \
        safe_delay(DELAY); \
      } \
      else { \
        lcd_scroll(0, 3, PSTR(STRING), LCD_WIDTH, DELAY); \
      }

    #ifdef STRING_SPLASH_LINE1
      //
      // Show the Marlin logo with splash line 1
      //
      if (LCD_EXTRA_SPACE >= strlen(STRING_SPLASH_LINE1) + 1) {
        //
        // Show the Marlin logo, splash line1, and splash line 2
        //
        logo_lines(PSTR(" " STRING_SPLASH_LINE1));
        #ifdef STRING_SPLASH_LINE2
          CENTER_OR_SCROLL(STRING_SPLASH_LINE2, 2000);
        #else
          safe_delay(2000);
        #endif
      }
      else {
        //
        // Show the Marlin logo with splash line 1
        // After a delay show splash line 2, if it exists
        //
        #ifdef STRING_SPLASH_LINE2
          #define _SPLASH_WAIT_1 1500
        #else
          #define _SPLASH_WAIT_1 2000
        #endif
        logo_lines(PSTR(""));
        CENTER_OR_SCROLL(STRING_SPLASH_LINE1, _SPLASH_WAIT_1);
        #ifdef STRING_SPLASH_LINE2
          CENTER_OR_SCROLL(STRING_SPLASH_LINE2, 1500);
        #endif
      }
    #elif defined(STRING_SPLASH_LINE2)
      //
      // Show splash line 2 only, alongside the logo if possible
      //
      if (LCD_EXTRA_SPACE >= strlen(STRING_SPLASH_LINE2) + 1) {
        logo_lines(PSTR(" " STRING_SPLASH_LINE2));
        safe_delay(2000);
      }
      else {
        logo_lines(PSTR(""));
        CENTER_OR_SCROLL(STRING_SPLASH_LINE2, 2000);
      }
    #else
      //
      // Show only the Marlin logo
      //
      logo_lines(PSTR(""));
      safe_delay(2000);
    #endif
    lcd_set_custom_characters();
  }

#endif // SHOW_BOOTSCREEN

void lcd_kill_screen() {
  lcd.setCursor(0, 0);
  lcd_print(lcd_status_message);
  #if LCD_HEIGHT < 4
    lcd.setCursor(0, 2);
  #else
    lcd.setCursor(0, 2);
    lcd_printPGM(PSTR(MSG_HALTED));
    lcd.setCursor(0, 3);
  #endif
  lcd_printPGM(PSTR(MSG_PLEASE_RESET));
}

/**
Possible status screens:
16x2   |000/000 B000/000|
       |0123456789012345|

16x4   |000/000 B000/000|
       |SD100%  Z 000.00|
       |F100%     T--:--|
       |0123456789012345|

20x2   |T000/000D B000/000D |
       |01234567890123456789|

20x4   |T000/000D B000/000D |
       |X 000 Y 000 Z 000.00|
       |F100%  SD100% T--:--|
       |01234567890123456789|

20x4   |T000/000D B000/000D |
       |T000/000D   Z 000.00|
       |F100%  SD100% T--:--|
       |01234567890123456789|
*/
static void lcd_implementation_status_screen() {

  #define LCD_TEMP_ONLY(T1,T2) \
    lcd.print(itostr3(T1 + 0.5)); \
    lcd.print('/'); \
    lcd.print(itostr3left(T2 + 0.5))

  #define LCD_TEMP(T1,T2,PREFIX) \
    lcd.print(PREFIX); \
    LCD_TEMP_ONLY(T1,T2); \
    lcd_printPGM(PSTR(LCD_STR_DEGREE " ")); \
    if (T2 < 10) lcd.print(' ')

  //
  // Line 1
  //

  lcd.setCursor(0, 0);

  #if LCD_WIDTH < 20

    //
    // Hotend 0 Temperature
    //
    LCD_TEMP_ONLY(thermalManager.degHotend(0), thermalManager.degTargetHotend(0));

    //
    // Hotend 1 or Bed Temperature
    //
    #if HOTENDS > 1 || TEMP_SENSOR_BED != 0

      lcd.setCursor(8, 0);
      #if HOTENDS > 1
        lcd.print(LCD_STR_THERMOMETER[0]);
        LCD_TEMP_ONLY(thermalManager.degHotend(1), thermalManager.degTargetHotend(1));
      #else
        lcd.print(LCD_STR_BEDTEMP[0]);
        LCD_TEMP_ONLY(thermalManager.degBed(), thermalManager.degTargetBed());
      #endif

    #endif // HOTENDS > 1 || TEMP_SENSOR_BED != 0

  #else // LCD_WIDTH >= 20

    //
    // Hotend 0 Temperature
    //
    LCD_TEMP(thermalManager.degHotend(0), thermalManager.degTargetHotend(0), LCD_STR_THERMOMETER[0]);

    //
    // Hotend 1 or Bed Temperature
    //
    #if HOTENDS > 1 || TEMP_SENSOR_BED != 0
      lcd.setCursor(10, 0);
      #if HOTENDS > 1
        LCD_TEMP(thermalManager.degHotend(1), thermalManager.degTargetHotend(1), LCD_STR_THERMOMETER[0]);
      #else
        LCD_TEMP(thermalManager.degBed(), thermalManager.degTargetBed(), LCD_STR_BEDTEMP[0]);
      #endif

    #endif  // HOTENDS > 1 || TEMP_SENSOR_BED != 0

  #endif // LCD_WIDTH >= 20

  //
  // Line 2
  //

  #if LCD_HEIGHT > 2

    bool blink = lcd_blink();

    #if LCD_WIDTH < 20

      #if ENABLED(SDSUPPORT)
        lcd.setCursor(0, 2);
        lcd_printPGM(PSTR("SD"));
        if (IS_SD_PRINTING)
          lcd.print(itostr3(card.percentDone()));
        else
          lcd_printPGM(PSTR("---"));
          lcd.print('%');
      #endif // SDSUPPORT

    #else // LCD_WIDTH >= 20

      lcd.setCursor(0, 1);

      #if HOTENDS > 1 && TEMP_SENSOR_BED != 0

        // If we both have a 2nd extruder and a heated bed,
        // show the heated bed temp on the left,
        // since the first line is filled with extruder temps
        LCD_TEMP(thermalManager.degBed(), thermalManager.degTargetBed(), LCD_STR_BEDTEMP[0]);

      #else
        // Before homing the axis letters are blinking 'X' <-> '?'.
        // When axis is homed but axis_known_position is false the axis letters are blinking 'X' <-> ' '.
        // When everything is ok you see a constant 'X'.

        _draw_axis_label(X_AXIS, PSTR(MSG_X), blink);
        lcd.print(ftostr4sign(current_position[X_AXIS]));

        lcd_printPGM(PSTR(" "));

        _draw_axis_label(Y_AXIS, PSTR(MSG_Y), blink);
        lcd.print(ftostr4sign(current_position[Y_AXIS]));

      #endif // HOTENDS > 1 || TEMP_SENSOR_BED != 0

    #endif // LCD_WIDTH >= 20

    lcd.setCursor(LCD_WIDTH - 8, 1);
    _draw_axis_label(Z_AXIS, PSTR(MSG_Z), blink);
    lcd.print(ftostr52sp(current_position[Z_AXIS] + 0.00001));

  #endif // LCD_HEIGHT > 2

  //
  // Line 3
  //

  #if LCD_HEIGHT > 3

    lcd.setCursor(0, 2);
    lcd.print(LCD_STR_FEEDRATE[0]);
    lcd.print('%');

    #if LCD_WIDTH > 19 && ENABLED(SDSUPPORT)

      lcd.setCursor(7, 2);
      lcd_printPGM(PSTR("SD"));
      if (IS_SD_PRINTING)
        lcd.print(itostr3(card.percentDone()));
      else
        lcd_printPGM(PSTR("---"));
      lcd.print('%');

    #endif // LCD_WIDTH > 19 && SDSUPPORT

    lcd.setCursor(LCD_WIDTH - 6, 2);
    lcd.print(LCD_STR_CLOCK[0]);

    char buffer[10];
    duration_t elapsed = print_job_timer.duration();
    elapsed.toDigital(buffer);
    lcd_print(buffer);

  #endif // LCD_HEIGHT > 3

  //
  // Last Line
  // Status Message (which may be a Progress Bar or Filament display)
  //

  lcd.setCursor(0, LCD_HEIGHT - 1);
  lcd_print(lcd_status_message);
}

static void lcd_implementation_drawmenu_generic(bool sel, uint8_t row, const char* pstr, char pre_char, char post_char) {
  char c;
  uint8_t n = LCD_WIDTH - 2;
  lcd.setCursor(0, row);
  lcd.print(sel ? pre_char : ' ');
  while ((c = pgm_read_byte(pstr)) && n > 0) {
    n -= lcd_print(c);
    pstr++;
  }
  while (n--) lcd.print(' ');
  lcd.print(post_char);
}

static void lcd_implementation_drawmenu_setting_edit_generic(bool sel, uint8_t row, const char* pstr, char pre_char, char* data) {
  char c;
  uint8_t n = LCD_WIDTH - 2 - lcd_strlen(data);
  lcd.setCursor(0, row);
  lcd.print(sel ? pre_char : ' ');
  while ((c = pgm_read_byte(pstr)) && n > 0) {
    n -= lcd_print(c);
    pstr++;
  }
  lcd.print(':');
  while (n--) lcd.print(' ');
  lcd_print(data);
}
static void lcd_implementation_drawmenu_setting_edit_generic_P(bool sel, uint8_t row, const char* pstr, char pre_char, const char* data) {
  char c;
  uint8_t n = LCD_WIDTH - 2 - lcd_strlen_P(data);
  lcd.setCursor(0, row);
  lcd.print(sel ? pre_char : ' ');
  while ((c = pgm_read_byte(pstr)) && n > 0) {
    n -= lcd_print(c);
    pstr++;
  }
  lcd.print(':');
  while (n--) lcd.print(' ');
  lcd_printPGM(data);
}

#define lcd_implementation_drawmenu_setting_edit_int3(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float3(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float32(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float43(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr43sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float5(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr5rj(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float52(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr52sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float51(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr51sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_long5(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr5rj(*(data)))
#define lcd_implementation_drawmenu_setting_edit_bool(sel, row, pstr, pstr2, data) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, '>', (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))

//Add version for callback functions
#define lcd_implementation_drawmenu_setting_edit_callback_int3(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float3(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float32(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float43(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr43sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float5(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr5rj(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float52(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr52sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float51(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr51sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_long5(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, '>', ftostr5rj(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_bool(sel, row, pstr, pstr2, data, callback) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, '>', (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))

void lcd_implementation_drawedit(const char* pstr, const char* value=NULL) {
  lcd.setCursor(1, 1);
  lcd_printPGM(pstr);
  if (value != NULL) {
    lcd.print(':');
    lcd.setCursor(LCD_WIDTH - lcd_strlen(value), 1);
    lcd_print(value);
  }
}

#if ENABLED(SDSUPPORT)

  static void lcd_implementation_drawmenu_sd(bool sel, uint8_t row, const char* pstr, const char* filename, char* longFilename, uint8_t concat, char post_char) {
    UNUSED(pstr);
    char c;
    uint8_t n = LCD_WIDTH - concat;
    lcd.setCursor(0, row);
    lcd.print(sel ? '>' : ' ');
    if (longFilename[0]) {
      filename = longFilename;
      longFilename[n] = '\0';
    }
    while ((c = *filename) && n > 0) {
      n -= lcd_print(c);
      filename++;
    }
    while (n--) lcd.print(' ');
    lcd.print(post_char);
  }

  static void lcd_implementation_drawmenu_sdfile(bool sel, uint8_t row, const char* pstr, const char* filename, char* longFilename) {
    lcd_implementation_drawmenu_sd(sel, row, pstr, filename, longFilename, 2, ' ');
  }

  static void lcd_implementation_drawmenu_sddirectory(bool sel, uint8_t row, const char* pstr, const char* filename, char* longFilename) {
    lcd_implementation_drawmenu_sd(sel, row, pstr, filename, longFilename, 2, LCD_STR_FOLDER[0]);
  }

#endif //SDSUPPORT

#define lcd_implementation_drawmenu_back(sel, row, pstr) lcd_implementation_drawmenu_generic(sel, row, pstr, LCD_STR_UPLEVEL[0], LCD_STR_UPLEVEL[0])
#define lcd_implementation_drawmenu_submenu(sel, row, pstr, data) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', LCD_STR_ARROW_RIGHT[0])
#define lcd_implementation_drawmenu_gcode(sel, row, pstr, gcode) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', ' ')
#define lcd_implementation_drawmenu_function(sel, row, pstr, data) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', ' ')

#if ENABLED(LCD_HAS_STATUS_INDICATORS)

  static void lcd_implementation_update_indicators() {
    // Set the LEDS - referred to as backlights by the LiquidTWI2 library
    static uint8_t ledsprev = 0;
    uint8_t leds = 0;

    if (thermalManager.degTargetBed() > 0) leds |= LED_A;

    if (thermalManager.degTargetHotend(0) > 0) leds |= LED_B;
    #if HOTENDS > 1
      if (thermalManager.degTargetHotend(1) > 0) leds |= LED_C;
    #endif

    if (leds != ledsprev) {
      lcd.setBacklight(leds);
      ledsprev = leds;
    }

  }

#endif // LCD_HAS_STATUS_INDICATORS
#endif // ULTRALCD_IMPL_HD44780_H
