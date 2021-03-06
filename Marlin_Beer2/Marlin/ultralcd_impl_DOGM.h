/**
 * ultralcd_impl_DOGM.h
 *
 * Graphics LCD implementation for 128x64 pixel LCDs by STB for ErikZalm/Marlin
 * Demonstrator: http://www.reprap.org/wiki/STB_Electronics
 * License: http://opensource.org/licenses/BSD-3-Clause
 *
 * With the use of:
 * u8glib by Oliver Kraus
 * https://github.com/olikraus/U8glib_Arduino
 * License: http://opensource.org/licenses/BSD-3-Clause
 */

#ifndef ULTRALCD_IMPL_DOGM_H
#define ULTRALCD_IMPL_DOGM_H

#include "MarlinConfig.h"

/**
 * Implementation of the LCD display routines for a DOGM128 graphic display.
 * These are common LCD 128x64 pixel graphic displays.
 */
#include "ultralcd.h"
#include "ultralcd_st7920_u8glib_rrd.h"
#include "dogm_bitmaps.h"
#include "duration_t.h"

#include <U8glib.h>

#if ENABLED(SHOW_BOOTSCREEN) && ENABLED(SHOW_CUSTOM_BOOTSCREEN)
  #include "_Bootscreen.h"
#endif

#if DISABLED(MAPPER_C2C3) && DISABLED(MAPPER_NON) && ENABLED(USE_BIG_EDIT_FONT)
  #undef USE_BIG_EDIT_FONT
#endif
#define FONT_STATUSMENU_NAME FONT_MENU_NAME
#include "dogm_font_data_Marlin_symbols.h"   // The Marlin special symbols
#define FONT_SPECIAL_NAME Marlin_symbols
#include "dogm_font_data_HD44780_J.h"
#define FONT_MENU_NAME HD44780_J_5x7


//#define FONT_STATUSMENU_NAME FONT_MENU_NAME

#define FONT_STATUSMENU 1
#define FONT_SPECIAL 2
#define FONT_MENU_EDIT 3
#define FONT_MENU 4

// DOGM parameters (size in pixels)
#define DOG_CHAR_WIDTH         6
#define DOG_CHAR_HEIGHT        12
#if ENABLED(USE_BIG_EDIT_FONT)
  #define FONT_MENU_EDIT_NAME u8g_font_9x18
  #define DOG_CHAR_WIDTH_EDIT  9
  #define DOG_CHAR_HEIGHT_EDIT 18
  #define LCD_WIDTH_EDIT       14
#else
  #define FONT_MENU_EDIT_NAME FONT_MENU_NAME
  #define DOG_CHAR_WIDTH_EDIT  6
  #define DOG_CHAR_HEIGHT_EDIT 12
  #define LCD_WIDTH_EDIT       22
#endif

#ifndef TALL_FONT_CORRECTION
  #define TALL_FONT_CORRECTION 0
#endif

#define START_COL              0

// LCD selection
#if ENABLED(U8GLIB_ST7920)
  //U8GLIB_ST7920_128X64_RRD u8g(0,0,0);
  U8GLIB_ST7920_128X64_RRD u8g(0);
#else
  // for regular DOGM128 display with HW-SPI
  U8GLIB_DOGM128 u8g(DOGLCD_CS, DOGLCD_A0);  // HW-SPI Com: CS, A0
#endif

#ifndef LCD_PIXEL_WIDTH
  #define LCD_PIXEL_WIDTH 128
#endif
#ifndef LCD_PIXEL_HEIGHT
  #define LCD_PIXEL_HEIGHT 64
#endif

#include "utf_mapper.h"

int lcd_contrast;
static char currentfont = 0;

static void lcd_setFont(char font_nr) {
  switch(font_nr) {
    case FONT_STATUSMENU : {u8g.setFont(FONT_STATUSMENU_NAME); currentfont = FONT_STATUSMENU;}; break;
    case FONT_MENU       : {u8g.setFont(FONT_MENU_NAME); currentfont = FONT_MENU;}; break;
    case FONT_SPECIAL    : {u8g.setFont(FONT_SPECIAL_NAME); currentfont = FONT_SPECIAL;}; break;
    case FONT_MENU_EDIT  : {u8g.setFont(FONT_MENU_EDIT_NAME); currentfont = FONT_MENU_EDIT;}; break;
    break;
  }
}

char lcd_print(char c) {
  if ((c > 0) && (c <= LCD_STR_SPECIAL_MAX)) {
    u8g.setFont(FONT_SPECIAL_NAME);
    u8g.print(c);
    lcd_setFont(currentfont);
    return 1;
  } else {
    return charset_mapper(c);
  }
}

char lcd_print(const char* str) {
  char c;
  int i = 0;
  char n = 0;
  while ((c = str[i++])) {
    n += lcd_print(c);
  }
  return n;
}

/* Arduino < 1.0.0 is missing a function to print PROGMEM strings, so we need to implement our own */
char lcd_printPGM(const char* str) {
  char c;
  char n = 0;
  while ((c = pgm_read_byte(str++))) {
    n += lcd_print(c);
  }
  return n;
}

/* Warning: This function is called from interrupt context */
static void lcd_implementation_init() {

  #if defined(LCD_PIN_BL) && LCD_PIN_BL > -1 // Enable LCD backlight
    pinMode(LCD_PIN_BL, OUTPUT);
    digitalWrite(LCD_PIN_BL, HIGH);
  #endif

  #if defined(LCD_PIN_RESET) && LCD_PIN_RESET > -1
    pinMode(LCD_PIN_RESET, OUTPUT);
    digitalWrite(LCD_PIN_RESET, HIGH);
  #endif

  #if DISABLED(MINIPANEL) // setContrast not working for Mini Panel
    u8g.setContrast(lcd_contrast);
  #endif

  #if ENABLED(LCD_SCREEN_ROT_90)
    u8g.setRot90();   // Rotate screen by 90°
  #elif ENABLED(LCD_SCREEN_ROT_180)
    u8g.setRot180();  // Rotate screen by 180°
  #elif ENABLED(LCD_SCREEN_ROT_270)
    u8g.setRot270();  // Rotate screen by 270°
  #endif

  #if ENABLED(SHOW_BOOTSCREEN)
    static bool show_bootscreen = true;

    #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
      if (show_bootscreen) {
        u8g.firstPage();
        do {
          u8g.drawBitmapP(
            (128 - (CUSTOM_BOOTSCREEN_BMPWIDTH))  /2,
            ( 64 - (CUSTOM_BOOTSCREEN_BMPHEIGHT)) /2,
            CEILING(CUSTOM_BOOTSCREEN_BMPWIDTH, 8), CUSTOM_BOOTSCREEN_BMPHEIGHT, custom_start_bmp);
        } while (u8g.nextPage());
        safe_delay(CUSTOM_BOOTSCREEN_TIMEOUT);
      }
    #endif // SHOW_CUSTOM_BOOTSCREEN

    int offx = (u8g.getWidth() - (START_BMPWIDTH)) / 2;

    #if ENABLED(START_BMPHIGH)
      int offy = 0;
    #else
      int offy = DOG_CHAR_HEIGHT;
    #endif

    int txt1X = (u8g.getWidth() - (sizeof(STRING_SPLASH_LINE1) - 1) * (DOG_CHAR_WIDTH)) / 2;

    if (show_bootscreen) {
      u8g.firstPage();
      do {
        u8g.drawBitmapP(offx, offy, START_BMPBYTEWIDTH, START_BMPHEIGHT, start_bmp);
        lcd_setFont(FONT_MENU);
        #ifndef STRING_SPLASH_LINE2
          u8g.drawStr(txt1X, u8g.getHeight() - (DOG_CHAR_HEIGHT), STRING_SPLASH_LINE1);
        #else
          int txt2X = (u8g.getWidth() - (sizeof(STRING_SPLASH_LINE2) - 1) * (DOG_CHAR_WIDTH)) / 2;
          u8g.drawStr(txt1X, u8g.getHeight() - (DOG_CHAR_HEIGHT) * 3 / 2, STRING_SPLASH_LINE1);
          u8g.drawStr(txt2X, u8g.getHeight() - (DOG_CHAR_HEIGHT) * 1 / 2, STRING_SPLASH_LINE2);
        #endif
      } while (u8g.nextPage());
    }

    show_bootscreen = false;

  #endif // SHOW_BOOTSCREEN
}

void lcd_kill_screen() {
  lcd_setFont(FONT_MENU);
  u8g.setPrintPos(0, u8g.getHeight()/4*1);
  lcd_print(lcd_status_message);
  u8g.setPrintPos(0, u8g.getHeight()/4*2);
  lcd_printPGM(PSTR(MSG_HALTED));
  u8g.setPrintPos(0, u8g.getHeight()/4*3);
  lcd_printPGM(PSTR(MSG_PLEASE_RESET));
}

static void lcd_implementation_clear() { } // Automatically cleared by Picture Loop

FORCE_INLINE void _draw_centered_temp(int temp, int x, int y) {
  int degsize = 6 * (temp >= 100 ? 3 : temp >= 10 ? 2 : 1); // number's pixel width
  u8g.setPrintPos(x - (18 - degsize) / 2, y); // move left if shorter
  lcd_print(itostr3(temp));
  lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
}
FORCE_INLINE void _draw_quemador(int x, int y){  
  u8g.drawBox(x + 2, y, 1, 1);
  u8g.drawBox(x + 6, y, 1, 1);
  u8g.drawBox(x + 11, y, 1, 1);
  u8g.drawBox(x + 15, y, 1, 1);
  u8g.drawBox(x + 1, y + 1, 3, 1);
  u8g.drawBox(x + 5, y + 1, 3, 1);
  u8g.drawBox(x + 10, y + 1, 3, 1);
  u8g.drawBox(x + 14, y + 1, 3, 1);
  u8g.drawBox(x, y + 2, 18,1);  
}

FORCE_INLINE void _draw_heater_status(int x, int heater) {
  #if HAS_TEMP_BED
    bool isBed = heater < 0;
  #else
    const bool isBed = false;
  #endif
  _draw_centered_temp((isBed ? thermalManager.degTargetBed() : thermalManager.degTargetHotend(heater)) + 0.5, x, 7);
  _draw_centered_temp((isBed ? thermalManager.degBed() : thermalManager.degHotend(heater)) + 0.5, x, 40);// MODIFICADO 28 -> 50
  int h = 0,
      y =29;
  if (isBed ? thermalManager.isHeatingBed() : thermalManager.isHeatingHotend(heater)) {
    _draw_quemador(x + h, y);    
  }
  else {    
    u8g.setColorIndex(0); // black on white
    u8g.drawBox(x + h, y, 18, 3);
    u8g.setColorIndex(1); // white on black
  }
}

#define OLLA_WIDTH 20
#define OLLA_HEIGTH 20
//  DRAW OLLA 
static void drawOlla(u8g_uint_t x, u8g_uint_t y, char etiqueta){
  u8g_uint_t ollaAlto = OLLA_HEIGTH;
  u8g_uint_t ollaAncho = OLLA_WIDTH;
  u8g_uint_t ollaIntAncho = ollaAncho - 2;
  u8g_uint_t ollaIntAlto = ollaAlto - 1;
  u8g.drawBox(x,y,ollaAncho,ollaAlto);
  u8g.setColorIndex(0); //  white on black
  u8g.drawBox(x + 1,y,ollaIntAncho,ollaIntAlto);
  u8g.setColorIndex(1); // black on white 
  bool updown = false;
  for(int i = x; i< (ollaAncho + x); i++){
    int y1 = (updown)? (y + 3): (y + 4);
    updown = !updown;
    u8g.drawBox(i,y1,1,1);
  }
  u8g.setPrintPos(x + 8, y + 14);
  lcd_print(etiqueta);
}// DRAW OLLA
// DRAW RECIRCULADO
volatile millis_t tRecirculadoAnt = 0;
static void drawRecirculado(u8g_uint_t x, u8g_uint_t y, char etiqueta){
  int radio = OLLA_HEIGTH / 4;
  u8g.drawCircle(x,y, radio);
  u8g.setPrintPos(x -2 , y + 4);
  lcd_print(etiqueta);
  millis_t now = millis();
  if(now > tRecirculadoAnt + 1000){
    u8g.drawHLine(x + radio, y - radio , radio*2 + 1);
    u8g.drawHLine(x - radio * 3, y + radio , radio*2 + 1);
    tRecirculadoAnt = now;
  }else{
    u8g.drawHLine(x + radio, y + radio , radio * 2 + 1);
    u8g.drawHLine(x - radio * 3, y - radio , radio * 2 + 1);
  }
}// DRAW RECIRCULADO

static void lcd_implementation_status_screen() {
  u8g_uint_t anchoPantalla = u8g.getWidth();
  u8g.setColorIndex(1); // black on white 
  bool blink = lcd_blink();
  //LICOR
  drawOlla(3, 8, 'L');
  //RECIRCULADO
  if(isResirculando){
      drawRecirculado((OLLA_WIDTH + 3) + (((anchoPantalla/2) - (OLLA_WIDTH / 2)) - (OLLA_WIDTH + 3))/2, 20, 'R');
  }
   //MACERADOR
  drawOlla((anchoPantalla / 2) -(OLLA_WIDTH / 2) , 8, 'M');
  //HERVIDO
  drawOlla(anchoPantalla - (OLLA_WIDTH + 3), 8, 'H');
  // Status Menu Font for SD info, Heater status, Fan, XYZ
  lcd_setFont(FONT_STATUSMENU);
  _draw_heater_status(4 , LICOR);
  _draw_heater_status((anchoPantalla/2)-(OLLA_WIDTH / 2) , MACERADOR);
  // Heated bed
  #if HOTENDS < 4 && HAS_TEMP_BED
    _draw_heater_status(anchoPantalla - (OLLA_WIDTH + 2), HERVIDO);
  #endif
  u8g.setColorIndex(1); // black on white
  // Status line
  u8g.setPrintPos(0, 63);
  lcd_print(lcd_status_message);
 
}

static void lcd_implementation_mark_as_selected(uint8_t row, bool isSelected) {
  if (isSelected) {
    u8g.setColorIndex(1);  // black on white
    u8g.drawBox(0, row * (DOG_CHAR_HEIGHT) + 3 - (TALL_FONT_CORRECTION), LCD_PIXEL_WIDTH, DOG_CHAR_HEIGHT);
    u8g.setColorIndex(0);  // following text must be white on black
  }
  else {
    u8g.setColorIndex(1); // unmarked text is black on white
  }
  u8g.setPrintPos((START_COL) * (DOG_CHAR_WIDTH), (row + 1) * (DOG_CHAR_HEIGHT));
}

static void lcd_implementation_drawmenu_generic(bool isSelected, uint8_t row, const char* pstr, char pre_char, char post_char) {
  UNUSED(pre_char);

  char c;
  uint8_t n = LCD_WIDTH - (START_COL) - 2;

  lcd_implementation_mark_as_selected(row, isSelected);

  while (c = pgm_read_byte(pstr)) {
    n -= lcd_print(c);
    pstr++;
  }
  while (n--) lcd_print(' ');
  u8g.setPrintPos(LCD_PIXEL_WIDTH - (DOG_CHAR_WIDTH), (row + 1) * (DOG_CHAR_HEIGHT));
  lcd_print(post_char);
  lcd_print(' ');
}

static void _drawmenu_setting_edit_generic(bool isSelected, uint8_t row, const char* pstr, const char* data, bool pgm) {
  char c;
  uint8_t vallen = (pgm ? lcd_strlen_P(data) : (lcd_strlen((char*)data)));
  uint8_t n = LCD_WIDTH - (START_COL) - 2 - vallen;

  lcd_implementation_mark_as_selected(row, isSelected);

  while (c = pgm_read_byte(pstr)) {
    n -= lcd_print(c);
    pstr++;
  }
  lcd_print(':');
  while (n--) lcd_print(' ');
  u8g.setPrintPos(LCD_PIXEL_WIDTH - (DOG_CHAR_WIDTH) * vallen, (row + 1) * (DOG_CHAR_HEIGHT));
  if (pgm)  lcd_printPGM(data);  else  lcd_print((char*)data);
}

#define lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, data) _drawmenu_setting_edit_generic(sel, row, pstr, data, false)
#define lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, data) _drawmenu_setting_edit_generic(sel, row, pstr, data, true)

#define lcd_implementation_drawmenu_setting_edit_int3(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float3(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float32(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float43(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr43sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float5(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr5rj(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float52(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr52sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float51(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr51sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_long5(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr5rj(*(data)))
#define lcd_implementation_drawmenu_setting_edit_bool(sel, row, pstr, pstr2, data) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))

//Add version for callback functions
#define lcd_implementation_drawmenu_setting_edit_callback_int3(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float3(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float32(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float43(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr43sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float5(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr5rj(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float52(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr52sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float51(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr51sign(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_long5(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr5rj(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_bool(sel, row, pstr, pstr2, data, callback) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))

void lcd_implementation_drawedit(const char* pstr, const char* value=NULL) {
  uint8_t rows = 1;
  uint8_t lcd_width = LCD_WIDTH - (START_COL), char_width = DOG_CHAR_WIDTH;
  uint8_t vallen = lcd_strlen(value);

  #if ENABLED(USE_BIG_EDIT_FONT)
    if (lcd_strlen_P(pstr) <= LCD_WIDTH_EDIT - 1) {
      lcd_setFont(FONT_MENU_EDIT);
      lcd_width = LCD_WIDTH_EDIT + 1;
      char_width = DOG_CHAR_WIDTH_EDIT;
      if (lcd_strlen_P(pstr) >= LCD_WIDTH_EDIT - vallen) rows = 2;
    }
    else {
      lcd_setFont(FONT_MENU);
    }
  #endif

  if (lcd_strlen_P(pstr) > LCD_WIDTH - 2 - vallen) rows = 2;

  const float kHalfChar = (DOG_CHAR_HEIGHT_EDIT) / 2;
  float rowHeight = u8g.getHeight() / (rows + 1); // 1/(rows+1) = 1/2 or 1/3

  u8g.setPrintPos(0, rowHeight + kHalfChar);
  lcd_printPGM(pstr);
  if (value != NULL) {
    lcd_print(':');
    u8g.setPrintPos((lcd_width - 1 - vallen) * char_width, rows * rowHeight + kHalfChar);
    lcd_print(value);
  }
}

#if ENABLED(SDSUPPORT)

  static void _drawmenu_sd(bool isSelected, uint8_t row, const char* pstr, const char* filename, char* const longFilename, bool isDir) {
    UNUSED(pstr);
    char c;
    uint8_t n = LCD_WIDTH - (START_COL) - 1;

    if (longFilename[0]) {
      filename = longFilename;
      longFilename[n] = '\0';
    }

    lcd_implementation_mark_as_selected(row, isSelected);

    if (isDir) lcd_print(LCD_STR_FOLDER[0]);
    while ((c = *filename)) {
      n -= lcd_print(c);
      filename++;
    }
    while (n--) lcd_print(' ');
  }

  #define lcd_implementation_drawmenu_sdfile(sel, row, pstr, filename, longFilename) _drawmenu_sd(sel, row, pstr, filename, longFilename, false)
  #define lcd_implementation_drawmenu_sddirectory(sel, row, pstr, filename, longFilename) _drawmenu_sd(sel, row, pstr, filename, longFilename, true)

#endif //SDSUPPORT

#define lcd_implementation_drawmenu_back(sel, row, pstr) lcd_implementation_drawmenu_generic(sel, row, pstr, LCD_STR_UPLEVEL[0], LCD_STR_UPLEVEL[0])
#define lcd_implementation_drawmenu_submenu(sel, row, pstr, data) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', LCD_STR_ARROW_RIGHT[0])
#define lcd_implementation_drawmenu_gcode(sel, row, pstr, gcode) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', ' ')
#define lcd_implementation_drawmenu_function(sel, row, pstr, data) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', ' ')

#endif //__ULTRALCD_IMPL_DOGM_H
