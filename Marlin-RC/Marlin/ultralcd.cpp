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

#include "ultralcd.h"
#if ENABLED(ULTRA_LCD)
#include "Marlin.h"
#include "language.h"
#include "temperature.h"
#include "configuration_store.h"

int preheatHotendTemp1, preheatBedTemp1, preheatFanSpeed1,
    preheatHotendTemp2, preheatBedTemp2, preheatFanSpeed2;
uint8_t lcd_status_message_level;
char lcd_status_message[3 * (LCD_WIDTH) + 1] = WELCOME_MSG; // worst case is kana with up to 3*LCD_WIDTH+1
#include "ultralcd_impl_DOGM.h"


// The main status screen
static void lcd_status_screen();

millis_t next_lcd_update_ms;

uint8_t lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; // Set when the LCD needs to draw, decrements after every draw. Set to 2 in LCD routines so the LCD gets at least 1 full redraw (first redraw is partial)

#if ENABLED(ULTIPANEL)

  // place-holders for Ki and Kd edits
  float raw_Ki, raw_Kd;

  /**
   * REVERSE_MENU_DIRECTION
   *
   * To reverse the menu direction we need a general way to reverse
   * the direction of the encoder everywhere. So encoderDirection is
   * added to allow the encoder to go the other way.
   *
   * This behavior is limited to scrolling Menus and SD card listings,
   * and is disabled in other contexts.
   */
  #if ENABLED(REVERSE_MENU_DIRECTION)
    int8_t encoderDirection = 1;
    #define ENCODER_DIRECTION_NORMAL() (encoderDirection = 1)
    #define ENCODER_DIRECTION_MENUS() (encoderDirection = -1)
  #else
    #define ENCODER_DIRECTION_NORMAL() ;
    #define ENCODER_DIRECTION_MENUS() ;
  #endif

  int8_t encoderDiff; // updated from interrupt context and added to encoderPosition every LCD update

  millis_t manual_move_start_time = 0;
  int8_t manual_move_axis = (int8_t)NO_AXIS;
  #if EXTRUDERS > 1
    int8_t manual_move_e_index = 0;
  #else
    #define manual_move_e_index 0
  #endif

  bool encoderRateMultiplierEnabled;
  int32_t lastEncoderMovementMillis;

  #if HAS_POWER_SWITCH
    extern bool powersupply;
  #endif
  static void lcd_main_menu();
  static void lcd_tune_menu();
  static void lcd_prepare_menu();
  static void lcd_move_menu();
  static void lcd_control_menu();
  static void lcd_control_temperature_menu();
  static void lcd_control_temperature_preheat_pla_settings_menu();
  static void lcd_control_temperature_preheat_abs_settings_menu();
  static void lcd_control_motion_menu();
  static void lcd_control_volumetric_menu();
  #if HAS_LCD_CONTRAST
    static void lcd_set_contrast();
  #endif
  // Function pointer to menu functions.
  typedef void (*screenFunc_t)();

  // Different types of actions that can be used in menu items.
  static void menu_action_back();
  static void menu_action_submenu(screenFunc_t data);
  static void menu_action_gcode(const char* pgcode);
  static void menu_action_function(screenFunc_t data);
  static void menu_action_setting_edit_bool(const char* pstr, bool* ptr);
  static void menu_action_setting_edit_int3(const char* pstr, int* ptr, int minValue, int maxValue);
  static void menu_action_setting_edit_float3(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float32(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float43(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float5(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float51(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float52(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_long5(const char* pstr, unsigned long* ptr, unsigned long minValue, unsigned long maxValue);
  static void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, screenFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_int3(const char* pstr, int* ptr, int minValue, int maxValue, screenFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float3(const char* pstr, float* ptr, float minValue, float maxValue, screenFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float32(const char* pstr, float* ptr, float minValue, float maxValue, screenFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float43(const char* pstr, float* ptr, float minValue, float maxValue, screenFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float5(const char* pstr, float* ptr, float minValue, float maxValue, screenFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float51(const char* pstr, float* ptr, float minValue, float maxValue, screenFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float52(const char* pstr, float* ptr, float minValue, float maxValue, screenFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_long5(const char* pstr, unsigned long* ptr, unsigned long minValue, unsigned long maxValue, screenFunc_t callbackFunc);
  /* Helper macros for menus */

  #ifndef ENCODER_FEEDRATE_DEADZONE
    #define ENCODER_FEEDRATE_DEADZONE 10
  #endif
  #ifndef ENCODER_STEPS_PER_MENU_ITEM
    #define ENCODER_STEPS_PER_MENU_ITEM 5
  #endif
  #ifndef ENCODER_PULSES_PER_STEP
    #define ENCODER_PULSES_PER_STEP 1
  #endif

  /**
   * START_SCREEN_OR_MENU generates init code for a screen or menu
   *
   *   encoderLine is the position based on the encoder
   *   encoderTopLine is the top menu line to display
   *   _lcdLineNr is the index of the LCD line (e.g., 0-3)
   *   _menuLineNr is the menu item to draw and process
   *   _thisItemNr is the index of each MENU_ITEM or STATIC_ITEM
   *   _countedItems is the total number of items in the menu (after one call)
   */
  #define START_SCREEN_OR_MENU(LIMIT) \
    ENCODER_DIRECTION_MENUS(); \
    encoderRateMultiplierEnabled = false; \
    if (encoderPosition > 0x8000) encoderPosition = 0; \
    static int8_t _countedItems = 0; \
    int8_t encoderLine = encoderPosition / ENCODER_STEPS_PER_MENU_ITEM; \
    if (_countedItems > 0 && encoderLine >= _countedItems - LIMIT) { \
      encoderLine = _countedItems - LIMIT; \
      encoderPosition = encoderLine * (ENCODER_STEPS_PER_MENU_ITEM); \
    }

  #define SCREEN_OR_MENU_LOOP() \
    int8_t _menuLineNr = encoderTopLine, _thisItemNr; \
    for (int8_t _lcdLineNr = 0; _lcdLineNr < LCD_HEIGHT; _lcdLineNr++, _menuLineNr++) { \
      _thisItemNr = 0

  /**
   * START_SCREEN  Opening code for a screen having only static items.
   *               Do simplified scrolling of the entire screen.
   *
   * START_MENU    Opening code for a screen with menu items.
   *               Scroll as-needed to keep the selected line in view.
   *               'wasClicked' indicates the controller was clicked.
   */
  #define START_SCREEN() \
    START_SCREEN_OR_MENU(LCD_HEIGHT); \
    encoderTopLine = encoderLine; \
    bool _skipStatic = false; \
    SCREEN_OR_MENU_LOOP()

  #define START_MENU() \
    START_SCREEN_OR_MENU(1); \
    NOMORE(encoderTopLine, encoderLine); \
    if (encoderLine >= encoderTopLine + LCD_HEIGHT) { \
      encoderTopLine = encoderLine - (LCD_HEIGHT - 1); \
    } \
    bool wasClicked = LCD_CLICKED; \
    bool _skipStatic = true; \
    SCREEN_OR_MENU_LOOP()

  /**
   * MENU_ITEM generates draw & handler code for a menu item, potentially calling:
   *
   *   lcd_implementation_drawmenu_[type](sel, row, label, arg3...)
   *   menu_action_[type](arg3...)
   *
   * Examples:
   *   MENU_ITEM(back, MSG_WATCH)
   *     lcd_implementation_drawmenu_back(sel, row, PSTR(MSG_WATCH))
   *     menu_action_back()
   *
   *   MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause)
   *     lcd_implementation_drawmenu_function(sel, row, PSTR(MSG_PAUSE_PRINT), lcd_sdcard_pause)
   *     menu_action_function(lcd_sdcard_pause)
   *
   *   MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
   *   MENU_ITEM(setting_edit_int3, MSG_SPEED, PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *     lcd_implementation_drawmenu_setting_edit_int3(sel, row, PSTR(MSG_SPEED), PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *     menu_action_setting_edit_int3(PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *
   */
  #define _MENU_ITEM_PART_1(TYPE, LABEL, ARGS...) \
    if (_menuLineNr == _thisItemNr) { \
      if (lcdDrawUpdate) \
        lcd_implementation_drawmenu_ ## TYPE(encoderLine == _thisItemNr, _lcdLineNr, PSTR(LABEL), ## ARGS); \
      if (wasClicked && encoderLine == _thisItemNr) { \
        lcd_quick_feedback()

  #define _MENU_ITEM_PART_2(TYPE, ARGS...) \
        menu_action_ ## TYPE(ARGS); \
        return; \
      } \
    } \
    ++_thisItemNr

  #define MENU_ITEM(TYPE, LABEL, ARGS...) do { \
      _skipStatic = false; \
      _MENU_ITEM_PART_1(TYPE, LABEL, ## ARGS); \
      _MENU_ITEM_PART_2(TYPE, ## ARGS); \
    } while(0)

  // Used to print static text with no visible cursor.
  #define STATIC_ITEM(LABEL, ARGS...) \
    if (_menuLineNr == _thisItemNr) { \
      if (_skipStatic && encoderLine <= _thisItemNr) { \
        encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
        lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT; \
      } \
      if (lcdDrawUpdate) \
        lcd_implementation_drawmenu_static(_lcdLineNr, PSTR(LABEL), ## ARGS); \
    } \
    ++_thisItemNr

  #define END_SCREEN() \
    } \
    _countedItems = _thisItemNr

  #define END_MENU() \
    } \
    _countedItems = _thisItemNr; \
    UNUSED(_skipStatic)

  #if ENABLED(ENCODER_RATE_MULTIPLIER)

    //#define ENCODER_RATE_MULTIPLIER_DEBUG  // If defined, output the encoder steps per second value

    /**
     * MENU_MULTIPLIER_ITEM generates drawing and handling code for a multiplier menu item
     */
    #define MENU_MULTIPLIER_ITEM(type, label, args...) do { \
        _MENU_ITEM_PART_1(type, label, ## args); \
        encoderRateMultiplierEnabled = true; \
        lastEncoderMovementMillis = 0; \
        _MENU_ITEM_PART_2(type, ## args); \
      } while(0)

  #endif //ENCODER_RATE_MULTIPLIER

  #define MENU_ITEM_DUMMY() do { _thisItemNr++; } while(0)
  #define MENU_ITEM_EDIT(type, label, args...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label), ## args)
  #define MENU_ITEM_EDIT_CALLBACK(type, label, args...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## args)
  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    #define MENU_MULTIPLIER_ITEM_EDIT(type, label, args...) MENU_MULTIPLIER_ITEM(setting_edit_ ## type, label, PSTR(label), ## args)
    #define MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(type, label, args...) MENU_MULTIPLIER_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## args)
  #else //!ENCODER_RATE_MULTIPLIER
    #define MENU_MULTIPLIER_ITEM_EDIT(type, label, args...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label), ## args)
    #define MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(type, label, args...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## args)
  #endif //!ENCODER_RATE_MULTIPLIER

  /** Used variables to keep track of the menu */
  volatile uint8_t buttons;  //the last checked buttons in a bit array.
  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    volatile uint8_t slow_buttons; // Bits of the pressed buttons.
  #endif
  int8_t encoderTopLine;              /* scroll offset in the current menu */
  millis_t next_button_update_ms;
  uint8_t lastEncoderBits;
  uint32_t encoderPosition;
  #if PIN_EXISTS(SD_DETECT)
    uint8_t lcd_sd_status;
  #endif

  typedef struct {
    screenFunc_t menu_function;
    uint32_t encoder_position;
  } menuPosition;

  screenFunc_t currentScreen = lcd_status_screen; // pointer to the currently active menu handler

  menuPosition screen_history[10];
  uint8_t screen_history_depth = 0;

  bool ignore_click = false;
  bool wait_for_unclick;
  bool defer_return_to_status = false;

  // Variables used when editing values.
  const char* editLabel;
  void* editValue;
  int32_t minEditValue, maxEditValue;
  screenFunc_t callbackFunc;              // call this after editing

  /**
   * General function to go directly to a menu
   * Remembers the previous position
   */
  static void lcd_goto_screen(screenFunc_t screen, const bool feedback = false, const uint32_t encoder = 0) {
    if (currentScreen != screen) {
      currentScreen = screen;
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
      encoderPosition = encoder;
      if (feedback) lcd_quick_feedback();
      if (screen == lcd_status_screen) {
        defer_return_to_status = false;
        screen_history_depth = 0;
      }
      #if ENABLED(LCD_PROGRESS_BAR)
        // For LCD_PROGRESS_BAR re-initialize custom characters
        lcd_set_custom_characters(screen == lcd_status_screen);
      #endif
    }
  }

  static void lcd_return_to_status() { lcd_goto_screen(lcd_status_screen); }

  inline void lcd_save_previous_menu() {
    if (screen_history_depth < COUNT(screen_history)) {
      screen_history[screen_history_depth].menu_function = currentScreen;
      screen_history[screen_history_depth].encoder_position = encoderPosition;
      ++screen_history_depth;
    }
  }

  static void lcd_goto_previous_menu(bool feedback=false) {
    if (screen_history_depth > 0) {
      --screen_history_depth;
      lcd_goto_screen(
        screen_history[screen_history_depth].menu_function,
        feedback,
        screen_history[screen_history_depth].encoder_position
      );
    }
    else
      lcd_return_to_status();
  }

  void lcd_ignore_click(bool b) {
    ignore_click = b;
    wait_for_unclick = false;
  }

#endif // ULTIPANEL

/**
 *
 * "Info Screen"
 *
 * This is very display-dependent, so the lcd implementation draws this.
 */

static void lcd_status_screen() {

  #if ENABLED(ULTIPANEL)
    ENCODER_DIRECTION_NORMAL();
    encoderRateMultiplierEnabled = false;
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)
    millis_t ms = millis();
    #if DISABLED(PROGRESS_MSG_ONCE)
      if (ELAPSED(ms, progress_bar_ms + PROGRESS_BAR_MSG_TIME + PROGRESS_BAR_BAR_TIME)) {
        progress_bar_ms = ms;
      }
    #endif
    #if PROGRESS_MSG_EXPIRE > 0
      // Handle message expire
      if (expire_status_ms > 0) {
        expire_status_ms = 0;
      }
    #endif
  #endif //LCD_PROGRESS_BAR

  lcd_implementation_status_screen();

  #if ENABLED(ULTIPANEL)

    bool current_click = LCD_CLICKED;

    if (ignore_click) {
      if (wait_for_unclick) {
        if (!current_click)
          ignore_click = wait_for_unclick = false;
        else
          current_click = false;
      }
      else if (current_click) {
        lcd_quick_feedback();
        wait_for_unclick = true;
        current_click = false;
      }
    }

    if (current_click) {
      lcd_goto_screen(lcd_main_menu, true);
      lcd_implementation_init( // to maybe revive the LCD if static electricity killed it.
        #if ENABLED(LCD_PROGRESS_BAR) && ENABLED(ULTIPANEL)
          currentScreen == lcd_status_screen
        #endif
      );
    }
  #endif //ULTIPANEL
}

/**
 *
 * draw the kill screen
 *
 */
void kill_screen(const char* lcd_msg) {
  lcd_init();
  lcd_setalertstatuspgm(lcd_msg);
  #if ENABLED(DOGLCD)
    u8g.firstPage();
    do {
      lcd_kill_screen();
    } while (u8g.nextPage());
  #else
    lcd_kill_screen();
  #endif
}

#if ENABLED(ULTIPANEL)
   /**
   *
   * "Main" menu
   *
   */

  static void lcd_main_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_WATCH);
    MENU_ITEM(submenu, MSG_PREPARE, lcd_prepare_menu);
    MENU_ITEM(submenu, MSG_CONTROL, lcd_control_menu);
    END_MENU();
  }

  /**
   *
   * "Tune" submenu items
   *
   */
  /**
  * Watch temperature callbacks
  */
  #if TEMP_SENSOR_0 != 0
    void watch_temp_callback_E0() {}
  #endif
  #if HOTENDS > 1 && TEMP_SENSOR_1 != 0
    void watch_temp_callback_E1() {}
  #endif // HOTENDS > 1
  #if HOTENDS > 2 && TEMP_SENSOR_2 != 0
    void watch_temp_callback_E2() {}
  #endif // HOTENDS > 2
  #if HOTENDS > 3 && TEMP_SENSOR_3 != 0
    void watch_temp_callback_E3() {}
  #endif // HOTENDS > 3
  #if TEMP_SENSOR_BED != 0
    void watch_temp_callback_bed() {}
  #endif 

  /**
   *
   * "Tune" submenu
   *
   */
  static void lcd_tune_menu() {
    START_MENU();
    //
    // ^ Main
    //
    MENU_ITEM(back, MSG_MAIN);
    
    //
    // Nozzle:
    // Nozzle [1-4]:
    //
    #if HOTENDS == 1
      #if TEMP_SENSOR_0 != 0
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
      #endif
    #else //HOTENDS > 1
      #if TEMP_SENSOR_0 != 0
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
      #endif
      #if TEMP_SENSOR_1 != 0
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &thermalManager.target_temperature[1], 0, HEATER_1_MAXTEMP - 15, watch_temp_callback_E1);
      #endif
      #if HOTENDS > 2
        #if TEMP_SENSOR_2 != 0
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &thermalManager.target_temperature[2], 0, HEATER_2_MAXTEMP - 15, watch_temp_callback_E2);
        #endif
        #if HOTENDS > 3
          #if TEMP_SENSOR_3 != 0
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &thermalManager.target_temperature[3], 0, HEATER_3_MAXTEMP - 15, watch_temp_callback_E3);
          #endif
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1

    //
    // Bed:
    //
    #if TEMP_SENSOR_BED != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED, &thermalManager.target_temperature_bed, 0, BED_MAXTEMP - 15, watch_temp_callback_bed);
    #endif

    //
    // Fan Speed:
    //
    #if FAN_COUNT > 0
      #if HAS_FAN0
        #if FAN_COUNT > 1
          #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED " 1"
        #else
          #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED
        #endif
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_1ST_FAN_SPEED, &fanSpeeds[0], 0, 255);
      #endif
      #if HAS_FAN1
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanSpeeds[1], 0, 255);
      #endif
      #if HAS_FAN2
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 3", &fanSpeeds[2], 0, 255);
      #endif
    #endif // FAN_COUNT > 0
    END_MENU();
  }

  /**
   *
   * "Prepare" submenu items
   *
   */
  void _lcd_preheat(int endnum, const float temph, const float tempb, const int fan) {
    if (temph > 0) thermalManager.setTargetHotend(temph, endnum);
    #if TEMP_SENSOR_BED != 0
      thermalManager.setTargetBed(tempb);
    #else
      UNUSED(tempb);
    #endif
    #if FAN_COUNT > 0
      #if FAN_COUNT > 1
        fanSpeeds[active_extruder < FAN_COUNT ? active_extruder : 0] = fan;
      #else
        fanSpeeds[0] = fan;
      #endif
    #else
      UNUSED(fan);
    #endif
    lcd_return_to_status();
  }

  #if TEMP_SENSOR_0 != 0
    void lcd_preheat_pla0() { _lcd_preheat(0, preheatHotendTemp1, preheatBedTemp1, preheatFanSpeed1); }
    void lcd_preheat_abs0() { _lcd_preheat(0, preheatHotendTemp2, preheatBedTemp2, preheatFanSpeed2); }
  #endif

  #if HOTENDS > 1
    void lcd_preheat_pla1() { _lcd_preheat(1, preheatHotendTemp1, preheatBedTemp1, preheatFanSpeed1); }
    void lcd_preheat_abs1() { _lcd_preheat(1, preheatHotendTemp2, preheatBedTemp2, preheatFanSpeed2); }
    #if HOTENDS > 2
      void lcd_preheat_pla2() { _lcd_preheat(2, preheatHotendTemp1, preheatBedTemp1, preheatFanSpeed1); }
      void lcd_preheat_abs2() { _lcd_preheat(2, preheatHotendTemp2, preheatBedTemp2, preheatFanSpeed2); }
      #if HOTENDS > 3
        void lcd_preheat_pla3() { _lcd_preheat(3, preheatHotendTemp1, preheatBedTemp1, preheatFanSpeed1); }
        void lcd_preheat_abs3() { _lcd_preheat(3, preheatHotendTemp2, preheatBedTemp2, preheatFanSpeed2); }
      #endif
    #endif

    void lcd_preheat_pla0123() {
      #if HOTENDS > 1
        thermalManager.setTargetHotend(preheatHotendTemp1, 1);
        #if HOTENDS > 2
          thermalManager.setTargetHotend(preheatHotendTemp1, 2);
          #if HOTENDS > 3
            thermalManager.setTargetHotend(preheatHotendTemp1, 3);
          #endif
        #endif
      #endif
      lcd_preheat_pla0();
    }
    void lcd_preheat_abs0123() {
      #if HOTENDS > 1
        thermalManager.setTargetHotend(preheatHotendTemp2, 1);
        #if HOTENDS > 2
          thermalManager.setTargetHotend(preheatHotendTemp2, 2);
          #if HOTENDS > 3
            thermalManager.setTargetHotend(preheatHotendTemp2, 3);
          #endif
        #endif
      #endif
      lcd_preheat_abs0();
    }

  #endif // HOTENDS > 1

  #if TEMP_SENSOR_BED != 0
    void lcd_preheat_pla_bedonly() { _lcd_preheat(0, 0, preheatBedTemp1, preheatFanSpeed1); }
    void lcd_preheat_abs_bedonly() { _lcd_preheat(0, 0, preheatBedTemp2, preheatFanSpeed2); }
  #endif

  #if TEMP_SENSOR_0 != 0 && (TEMP_SENSOR_1 != 0 || TEMP_SENSOR_2 != 0 || TEMP_SENSOR_3 != 0 || TEMP_SENSOR_BED != 0)

    static void lcd_preheat_pla_menu() {
      START_MENU();
      MENU_ITEM(back, MSG_PREPARE);
      #if HOTENDS == 1
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_pla0);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H1, lcd_preheat_pla0);
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H2, lcd_preheat_pla1);
        #if HOTENDS > 2
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_pla2);
          #if HOTENDS > 3
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_pla3);
          #endif
        #endif
        MENU_ITEM(function, MSG_PREHEAT_1_ALL, lcd_preheat_pla0123);
      #endif
      #if TEMP_SENSOR_BED != 0
        MENU_ITEM(function, MSG_PREHEAT_1_BEDONLY, lcd_preheat_pla_bedonly);
      #endif
      END_MENU();
    }

    static void lcd_preheat_abs_menu() {
      START_MENU();
      MENU_ITEM(back, MSG_PREPARE);
      #if HOTENDS == 1
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_abs0);
      #else
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H1, lcd_preheat_abs0);
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H2, lcd_preheat_abs1);
        #if HOTENDS > 2
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_abs2);
          #if HOTENDS > 3
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_abs3);
          #endif
        #endif
        MENU_ITEM(function, MSG_PREHEAT_2_ALL, lcd_preheat_abs0123);
      #endif
      #if TEMP_SENSOR_BED != 0
        MENU_ITEM(function, MSG_PREHEAT_2_BEDONLY, lcd_preheat_abs_bedonly);
      #endif
      END_MENU();
    }

  #endif // TEMP_SENSOR_0 && (TEMP_SENSOR_1 || TEMP_SENSOR_2 || TEMP_SENSOR_3 || TEMP_SENSOR_BED)

  void lcd_cooldown() {
    #if FAN_COUNT > 0
      for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
    #endif
    thermalManager.disable_all_heaters();
    lcd_return_to_status();
  }
  /**
   *
   * "Prepare" submenu
   *
   */

  static void lcd_prepare_menu() {
    START_MENU();

    //
    // ^ Main
    //
    MENU_ITEM(back, MSG_MAIN);

    //
    // Auto Home
    //
    MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
    #if ENABLED(INDIVIDUAL_AXIS_HOMING_MENU)
      MENU_ITEM(gcode, MSG_AUTO_HOME_X, PSTR("G28 X"));
      MENU_ITEM(gcode, MSG_AUTO_HOME_Y, PSTR("G28 Y"));
      MENU_ITEM(gcode, MSG_AUTO_HOME_Z, PSTR("G28 Z"));
    #endif
    // Preheat PLA
    // Preheat ABS
    //
    #if TEMP_SENSOR_0 != 0
      #if TEMP_SENSOR_1 != 0 || TEMP_SENSOR_2 != 0 || TEMP_SENSOR_3 != 0 || TEMP_SENSOR_BED != 0
        MENU_ITEM(submenu, MSG_PREHEAT_1, lcd_preheat_pla_menu);
        MENU_ITEM(submenu, MSG_PREHEAT_2, lcd_preheat_abs_menu);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_pla0);
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_abs0);
      #endif
    #endif

    //
    // Cooldown
    //
    MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);

    //
    // Switch power on/off
    //
    #if HAS_POWER_SWITCH
      if (powersupply)
        MENU_ITEM(gcode, MSG_SWITCH_PS_OFF, PSTR("M81"));
      else
        MENU_ITEM(gcode, MSG_SWITCH_PS_ON, PSTR("M80"));
    #endif
    END_MENU();
  }



  /**
   *
   * "Control" submenu
   *
   */

  static void lcd_control_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_MAIN);
    MENU_ITEM(submenu, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM(submenu, MSG_MOTION, lcd_control_motion_menu);
    MENU_ITEM(submenu, MSG_VOLUMETRIC, lcd_control_volumetric_menu);

    #if HAS_LCD_CONTRAST
      //MENU_ITEM_EDIT(int3, MSG_CONTRAST, &lcd_contrast, 0, 63);
      MENU_ITEM(submenu, MSG_CONTRAST, lcd_set_contrast);
    #endif
    MENU_ITEM(function, MSG_RESTORE_FAILSAFE, Config_ResetDefault);
    END_MENU();
  }

  /**
   *
   * "Temperature" submenu
   *
   */

  #if ENABLED(PID_AUTOTUNE_MENU)

    #if ENABLED(PIDTEMP)
      int autotune_temp[HOTENDS] = ARRAY_BY_HOTENDS1(150);
      const int heater_maxtemp[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP);
    #endif

    static void _lcd_autotune(int e) {
      char cmd[30];
      sprintf_P(cmd, PSTR("M303 U1 E%i S%i"), e,
        #if HAS_PID_FOR_BOTH
          e < 0 ? autotune_temp_bed : autotune_temp[e]
        #else
          autotune_temp[e]
        #endif
      );
      enqueue_and_echo_command(cmd);
    }

  #endif //PID_AUTOTUNE_MENU

  #if ENABLED(PIDTEMP)

    // Helpers for editing PID Ki & Kd values
    // grab the PID value out of the temp variable; scale it; then update the PID driver
    void copy_and_scalePID_i(int e) {
      #if DISABLED(PID_PARAMS_PER_HOTEND) || HOTENDS == 1
        UNUSED(e);
      #endif
      PID_PARAM(Ki, e) = scalePID_i(raw_Ki);
      thermalManager.updatePID();
    }
    void copy_and_scalePID_d(int e) {
      #if DISABLED(PID_PARAMS_PER_HOTEND) || HOTENDS == 1
        UNUSED(e);
      #endif
      PID_PARAM(Kd, e) = scalePID_d(raw_Kd);
      thermalManager.updatePID();
    }
    #define _PIDTEMP_BASE_FUNCTIONS(eindex) \
      void copy_and_scalePID_i_E ## eindex() { copy_and_scalePID_i(eindex); } \
      void copy_and_scalePID_d_E ## eindex() { copy_and_scalePID_d(eindex); }

    #if ENABLED(PID_AUTOTUNE_MENU)
      #define _PIDTEMP_FUNCTIONS(eindex) \
        _PIDTEMP_BASE_FUNCTIONS(eindex); \
        void lcd_autotune_callback_E ## eindex() { _lcd_autotune(eindex); }
    #else
      #define _PIDTEMP_FUNCTIONS(eindex) _PIDTEMP_BASE_FUNCTIONS(eindex)
    #endif

    _PIDTEMP_FUNCTIONS(0);
    #if ENABLED(PID_PARAMS_PER_HOTEND)
      #if HOTENDS > 1
        _PIDTEMP_FUNCTIONS(1);
        #if HOTENDS > 2
          _PIDTEMP_FUNCTIONS(2);
          #if HOTENDS > 3
            _PIDTEMP_FUNCTIONS(3);
          #endif //HOTENDS > 3
        #endif //HOTENDS > 2
      #endif //HOTENDS > 1
    #endif //PID_PARAMS_PER_HOTEND

  #endif //PIDTEMP

  /**
   *
   * "Control" > "Temperature" submenu
   *
   */
  static void lcd_control_temperature_menu() {
    START_MENU();

    //
    // ^ Control
    //
    MENU_ITEM(back, MSG_CONTROL);

    //
    // Nozzle:
    // Nozzle [1-4]:
    //
    #if HOTENDS == 1
      #if TEMP_SENSOR_0 != 0
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
      #endif
    #else //HOTENDS > 1
      #if TEMP_SENSOR_0 != 0
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &thermalManager.target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
      #endif
      #if TEMP_SENSOR_1 != 0
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &thermalManager.target_temperature[1], 0, HEATER_1_MAXTEMP - 15, watch_temp_callback_E1);
      #endif
      #if HOTENDS > 2
        #if TEMP_SENSOR_2 != 0
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &thermalManager.target_temperature[2], 0, HEATER_2_MAXTEMP - 15, watch_temp_callback_E2);
        #endif
        #if HOTENDS > 3
          #if TEMP_SENSOR_3 != 0
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &thermalManager.target_temperature[3], 0, HEATER_3_MAXTEMP - 15, watch_temp_callback_E3);
          #endif
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1

    //
    // Bed:
    //
    #if TEMP_SENSOR_BED != 0
      MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_BED, &thermalManager.target_temperature_bed, 0, BED_MAXTEMP - 15);
    #endif

    //
    // Fan Speed:
    //
    #if FAN_COUNT > 0
      #if HAS_FAN0
        #if FAN_COUNT > 1
          #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED " 1"
        #else
          #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED
        #endif
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_1ST_FAN_SPEED, &fanSpeeds[0], 0, 255);
      #endif
      #if HAS_FAN1
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanSpeeds[1], 0, 255);
      #endif
      #if HAS_FAN2
        MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 3", &fanSpeeds[2], 0, 255);
      #endif
    #endif // FAN_COUNT > 0

    //
    // PID-P, PID-I, PID-D, PID-C, PID Autotune
    // PID-P E1, PID-I E1, PID-D E1, PID-C E1, PID Autotune E1
    // PID-P E2, PID-I E2, PID-D E2, PID-C E2, PID Autotune E2
    // PID-P E3, PID-I E3, PID-D E3, PID-C E3, PID Autotune E3
    // PID-P E4, PID-I E4, PID-D E4, PID-C E4, PID Autotune E4
    //
    #if ENABLED(PIDTEMP)

      #define _PID_BASE_MENU_ITEMS(ELABEL, eindex) \
        raw_Ki = unscalePID_i(PID_PARAM(Ki, eindex)); \
        raw_Kd = unscalePID_d(PID_PARAM(Kd, eindex)); \
        MENU_ITEM_EDIT(float52, MSG_PID_P ELABEL, &PID_PARAM(Kp, eindex), 1, 9990); \
        MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I ELABEL, &raw_Ki, 0.01, 9990, copy_and_scalePID_i_E ## eindex); \
        MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_D ELABEL, &raw_Kd, 1, 9990, copy_and_scalePID_d_E ## eindex)
        #define _PID_MENU_ITEMS(ELABEL, eindex) _PID_BASE_MENU_ITEMS(ELABEL, eindex)
      #if ENABLED(PID_AUTOTUNE_MENU)
        #define PID_MENU_ITEMS(ELABEL, eindex) \
          _PID_MENU_ITEMS(ELABEL, eindex); \
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE ELABEL, &autotune_temp[eindex], 150, heater_maxtemp[eindex] - 15, lcd_autotune_callback_E ## eindex)
      #else
        #define PID_MENU_ITEMS(ELABEL, eindex) _PID_MENU_ITEMS(ELABEL, eindex)
      #endif

      #if ENABLED(PID_PARAMS_PER_HOTEND) && HOTENDS > 1
        PID_MENU_ITEMS(MSG_E1, 0);
        PID_MENU_ITEMS(MSG_E2, 1);
        #if HOTENDS > 2
          PID_MENU_ITEMS(MSG_E3, 2);
          #if HOTENDS > 3
            PID_MENU_ITEMS(MSG_E4, 3);
          #endif //HOTENDS > 3
        #endif //HOTENDS > 2
      #else //!PID_PARAMS_PER_HOTEND || HOTENDS == 1
        PID_MENU_ITEMS("", 0);
      #endif //!PID_PARAMS_PER_HOTEND || HOTENDS == 1

    #endif //PIDTEMP

    //
    // Preheat PLA conf
    //
    MENU_ITEM(submenu, MSG_PREHEAT_1_SETTINGS, lcd_control_temperature_preheat_pla_settings_menu);

    //
    // Preheat ABS conf
    //
    MENU_ITEM(submenu, MSG_PREHEAT_2_SETTINGS, lcd_control_temperature_preheat_abs_settings_menu);
    END_MENU();
  }

  /**
   *
   * "Temperature" > "Preheat PLA conf" submenu
   *
   */
  static void lcd_control_temperature_preheat_pla_settings_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_TEMPERATURE);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &preheatFanSpeed1, 0, 255);
    #if TEMP_SENSOR_0 != 0
      MENU_ITEM_EDIT(int3, MSG_NOZZLE, &preheatHotendTemp1, HEATER_0_MINTEMP, HEATER_0_MAXTEMP - 15);
    #endif
    #if TEMP_SENSOR_BED != 0
      MENU_ITEM_EDIT(int3, MSG_BED, &preheatBedTemp1, BED_MINTEMP, BED_MAXTEMP - 15);
    #endif
    END_MENU();
  }

  /**
   *
   * "Temperature" > "Preheat ABS conf" submenu
   *
   */
  static void lcd_control_temperature_preheat_abs_settings_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_TEMPERATURE);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &preheatFanSpeed2, 0, 255);
    #if TEMP_SENSOR_0 != 0
      MENU_ITEM_EDIT(int3, MSG_NOZZLE, &preheatHotendTemp2, HEATER_0_MINTEMP, HEATER_0_MAXTEMP - 15);
    #endif
    #if TEMP_SENSOR_BED != 0
      MENU_ITEM_EDIT(int3, MSG_BED, &preheatBedTemp2, BED_MINTEMP, BED_MAXTEMP - 15);
    #endif
    END_MENU();
  }

  /**
   *
   * "Control" > "Motion" submenu
   *
   */
  static void lcd_control_motion_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_CONTROL);   
    END_MENU();
  }

  /**
   *
   * "Control" > "Filament" submenu
   *
   */
  static void lcd_control_volumetric_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_CONTROL);
    END_MENU();
  }

  /**
   *
   * "Control" > "Contrast" submenu
   *
   */
  #if HAS_LCD_CONTRAST
    static void lcd_set_contrast() {
      if (LCD_CLICKED) { lcd_goto_previous_menu(true); return; }
      ENCODER_DIRECTION_NORMAL();
      if (encoderPosition) {
        set_lcd_contrast(lcd_contrast + encoderPosition);
        encoderPosition = 0;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      }
      if (lcdDrawUpdate) {
        lcd_implementation_drawedit(PSTR(MSG_CONTRAST),
          #if LCD_CONTRAST_MAX >= 100
            itostr3(lcd_contrast)
          #else
            itostr2(lcd_contrast)
          #endif
        );
      }
    }
  #endif // HAS_LCD_CONTRAST
  /**
   *
   * Functions for editing single values
   *
   * The "menu_edit_type" macro generates the functions needed to edit a numerical value.
   *
   * For example, menu_edit_type(int, int3, itostr3, 1) expands into these functions:
   *
   *   bool _menu_edit_int3();
   *   void menu_edit_int3(); // edit int (interactively)
   *   void menu_edit_callback_int3(); // edit int (interactively) with callback on completion
   *   static void _menu_action_setting_edit_int3(const char* pstr, int* ptr, int minValue, int maxValue);
   *   static void menu_action_setting_edit_int3(const char* pstr, int* ptr, int minValue, int maxValue);
   *   static void menu_action_setting_edit_callback_int3(const char* pstr, int* ptr, int minValue, int maxValue, screenFunc_t callback); // edit int with callback
   *
   * You can then use one of the menu macros to present the edit interface:
   *   MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
   *
   * This expands into a more primitive menu item:
   *   MENU_ITEM(setting_edit_int3, MSG_SPEED, PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *
   *
   * Also: MENU_MULTIPLIER_ITEM_EDIT, MENU_ITEM_EDIT_CALLBACK, and MENU_MULTIPLIER_ITEM_EDIT_CALLBACK
   *
   *       menu_action_setting_edit_int3(PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   */
  #define menu_edit_type(_type, _name, _strFunc, scale) \
    bool _menu_edit_ ## _name () { \
      ENCODER_DIRECTION_NORMAL(); \
      bool isClicked = LCD_CLICKED; \
      if ((int32_t)encoderPosition < 0) encoderPosition = 0; \
      if ((int32_t)encoderPosition > maxEditValue) encoderPosition = maxEditValue; \
      if (lcdDrawUpdate) \
        lcd_implementation_drawedit(editLabel, _strFunc(((_type)((int32_t)encoderPosition + minEditValue)) / scale)); \
      if (isClicked) { \
        *((_type*)editValue) = ((_type)((int32_t)encoderPosition + minEditValue)) / scale; \
        lcd_goto_previous_menu(true); \
      } \
      return isClicked; \
    } \
    void menu_edit_ ## _name () { _menu_edit_ ## _name(); } \
    void menu_edit_callback_ ## _name () { if (_menu_edit_ ## _name ()) (*callbackFunc)(); } \
    static void _menu_action_setting_edit_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue) { \
      lcd_save_previous_menu(); \
      \
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; \
      \
      editLabel = pstr; \
      editValue = ptr; \
      minEditValue = minValue * scale; \
      maxEditValue = maxValue * scale - minEditValue; \
      encoderPosition = (*ptr) * scale - minEditValue; \
    } \
    static void menu_action_setting_edit_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue) { \
      _menu_action_setting_edit_ ## _name(pstr, ptr, minValue, maxValue); \
      currentScreen = menu_edit_ ## _name; \
    }\
    static void menu_action_setting_edit_callback_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue, screenFunc_t callback) { \
      _menu_action_setting_edit_ ## _name(pstr, ptr, minValue, maxValue); \
      currentScreen = menu_edit_callback_ ## _name; \
      callbackFunc = callback; \
    }

  menu_edit_type(int, int3, itostr3, 1);
  menu_edit_type(float, float3, ftostr3, 1);
  menu_edit_type(float, float32, ftostr32, 100);
  menu_edit_type(float, float43, ftostr43sign, 1000);
  menu_edit_type(float, float5, ftostr5rj, 0.01);
  menu_edit_type(float, float51, ftostr51sign, 10);
  menu_edit_type(float, float52, ftostr52sign, 100);
  menu_edit_type(unsigned long, long5, ftostr5rj, 0.01);
  /**
   *
   * Audio feedback for controller clicks
   *
   */
  void lcd_buzz(long duration, uint16_t freq) {
    #if PIN_EXISTS(BEEPER)
      buzzer.tone(duration, freq);
    #endif
  }

  void lcd_quick_feedback() {
    lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
    next_button_update_ms = millis() + 500;

    // Buzz and wait. The delay is needed for buttons to settle!
    lcd_buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
    #if PIN_EXISTS(BEEPER)
      for (int8_t i = 5; i--;) { buzzer.tick(); delay(2); }
    #endif
  }

  /**
   *
   * Menu actions
   *
   */
  static void menu_action_back() { lcd_goto_previous_menu(); }
  static void menu_action_submenu(screenFunc_t func) { lcd_save_previous_menu(); lcd_goto_screen(func); }
  static void menu_action_gcode(const char* pgcode) { enqueue_and_echo_commands_P(pgcode); }
  static void menu_action_function(screenFunc_t func) { (*func)(); }
  static void menu_action_setting_edit_bool(const char* pstr, bool* ptr) {UNUSED(pstr); *ptr = !(*ptr); }
  static void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, screenFunc_t callback) {
    menu_action_setting_edit_bool(pstr, ptr);
    (*callback)();
  }

#endif // ULTIPANEL
void lcd_init() {
  lcd_implementation_init();
  #if ENABLED(NEWPANEL)
    #if BUTTON_EXISTS(EN1)
      SET_INPUT(BTN_EN1);
      WRITE(BTN_EN1, HIGH);
    #endif
    #if BUTTON_EXISTS(EN2)
      SET_INPUT(BTN_EN2);
      WRITE(BTN_EN2, HIGH);
    #endif
    #if BUTTON_EXISTS(ENC)
      SET_INPUT(BTN_ENC);
      WRITE(BTN_ENC, HIGH);
    #endif
    #if BUTTON_EXISTS(UP)
      SET_INPUT(BTN_UP);
    #endif
    #if BUTTON_EXISTS(DWN)
      SET_INPUT(BTN_DWN);
    #endif
    #if BUTTON_EXISTS(LFT)
      SET_INPUT(BTN_LFT);
    #endif
    #if BUTTON_EXISTS(RT)
      SET_INPUT(BTN_RT);
    #endif
  #else // !NEWPANEL

    #if ENABLED(SR_LCD_2W_NL) // Non latching 2 wire shift register
      pinMode(SR_DATA_PIN, OUTPUT);
      pinMode(SR_CLK_PIN, OUTPUT);
    #elif defined(SHIFT_CLK)
      pinMode(SHIFT_CLK, OUTPUT);
      pinMode(SHIFT_LD, OUTPUT);
      pinMode(SHIFT_EN, OUTPUT);
      pinMode(SHIFT_OUT, INPUT);
      WRITE(SHIFT_OUT, HIGH);
      WRITE(SHIFT_LD, HIGH);
      WRITE(SHIFT_EN, LOW);
    #endif // SR_LCD_2W_NL

  #endif // !NEWPANEL
  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    slow_buttons = 0;
  #endif

  lcd_buttons_update();

  #if ENABLED(ULTIPANEL)
    encoderDiff = 0;
  #endif
}

int lcd_strlen(const char* s) {
  int i = 0, j = 0;
  while (s[i]) {
    #if ENABLED(MAPPER_NON)
      j++;
    #else
      if ((s[i] & 0xC0u) != 0x80u) j++;
    #endif
    i++;
  }
  return j;
}

int lcd_strlen_P(const char* s) {
  int j = 0;
  while (pgm_read_byte(s)) {
    #if ENABLED(MAPPER_NON)
      j++;
    #else
      if ((pgm_read_byte(s) & 0xC0u) != 0x80u) j++;
    #endif
    s++;
  }
  return j;
}

bool lcd_blink() {
  static uint8_t blink = 0;
  static millis_t next_blink_ms = 0;
  millis_t ms = millis();
  if (ELAPSED(ms, next_blink_ms)) {
    blink ^= 0xFF;
    next_blink_ms = ms + 1000 - LCD_UPDATE_INTERVAL / 2;
  }
  return blink != 0;
}

/**
 * Update the LCD, read encoder buttons, etc.
 *   - Read button states
 *   - Check the SD Card slot state
 *   - Act on RepRap World keypad input
 *   - Update the encoder position
 *   - Apply acceleration to the encoder position
 *   - Set lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NOW on controller events
 *   - Reset the Info Screen timeout if there's any input
 *   - Update status indicators, if any
 *
 *   Run the current LCD menu handler callback function:
 *   - Call the handler only if lcdDrawUpdate != LCDVIEW_NONE
 *   - Before calling the handler, LCDVIEW_CALL_NO_REDRAW => LCDVIEW_NONE
 *   - Call the menu handler. Menu handlers should do the following:
 *     - If a value changes, set lcdDrawUpdate to LCDVIEW_REDRAW_NOW and draw the value
 *       (Encoder events automatically set lcdDrawUpdate for you.)
 *     - if (lcdDrawUpdate) { redraw }
 *     - Before exiting the handler set lcdDrawUpdate to:
 *       - LCDVIEW_CLEAR_CALL_REDRAW to clear screen and set LCDVIEW_CALL_REDRAW_NEXT.
 *       - LCDVIEW_REDRAW_NOW or LCDVIEW_NONE to keep drawingm but only in this loop.
 *       - LCDVIEW_REDRAW_NEXT to keep drawing and draw on the next loop also.
 *       - LCDVIEW_CALL_NO_REDRAW to keep drawing (or start drawing) with no redraw on the next loop.
 *     - NOTE: For graphical displays menu handlers may be called 2 or more times per loop,
 *             so don't change lcdDrawUpdate without considering this.
 *
 *   After the menu handler callback runs (or not):
 *   - Clear the LCD if lcdDrawUpdate == LCDVIEW_CLEAR_CALL_REDRAW
 *   - Update lcdDrawUpdate for the next loop (i.e., move one state down, usually)
 *
 * No worries. This function is only called from the main thread.
 */
void lcd_update() {
  lcd_buttons_update();
  int return_to_status_ms;
  millis_t ms = millis();
  if (ELAPSED(ms, next_lcd_update_ms)) {
    next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;
    #if ENABLED(LCD_HAS_STATUS_INDICATORS)
      lcd_implementation_update_indicators();
    #endif

    #if ENABLED(LCD_HAS_SLOW_BUTTONS)
      slow_buttons = lcd_implementation_read_slow_buttons(); // buttons which take too long to read in interrupt context
    #endif

    #if ENABLED(ULTIPANEL)
      bool encoderPastThreshold = (abs(encoderDiff) >= ENCODER_PULSES_PER_STEP);
      if (encoderPastThreshold || LCD_CLICKED) {
        if (encoderPastThreshold) {
          int32_t encoderMultiplier = 1;

          #if ENABLED(ENCODER_RATE_MULTIPLIER)

            if (encoderRateMultiplierEnabled) {
              int32_t encoderMovementSteps = abs(encoderDiff) / ENCODER_PULSES_PER_STEP;

              if (lastEncoderMovementMillis != 0) {
                // Note that the rate is always calculated between to passes through the
                // loop and that the abs of the encoderDiff value is tracked.
                float encoderStepRate = (float)(encoderMovementSteps) / ((float)(ms - lastEncoderMovementMillis)) * 1000.0;

                if (encoderStepRate >= ENCODER_100X_STEPS_PER_SEC)     encoderMultiplier = 100;
                else if (encoderStepRate >= ENCODER_10X_STEPS_PER_SEC) encoderMultiplier = 10;

                #if ENABLED(ENCODER_RATE_MULTIPLIER_DEBUG)
                  SERIAL_ECHO_START;
                  SERIAL_ECHOPAIR("Enc Step Rate: ", encoderStepRate);
                  SERIAL_ECHOPAIR("  Multiplier: ", encoderMultiplier);
                  SERIAL_ECHOPAIR("  ENCODER_10X_STEPS_PER_SEC: ", ENCODER_10X_STEPS_PER_SEC);
                  SERIAL_ECHOPAIR("  ENCODER_100X_STEPS_PER_SEC: ", ENCODER_100X_STEPS_PER_SEC);
                  SERIAL_EOL;
                #endif //ENCODER_RATE_MULTIPLIER_DEBUG
              }

              lastEncoderMovementMillis = ms;
            } // encoderRateMultiplierEnabled
          #endif //ENCODER_RATE_MULTIPLIER

          encoderPosition += (encoderDiff * encoderMultiplier) / ENCODER_PULSES_PER_STEP;
          encoderDiff = 0;
        }
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      }
    #endif // ULTIPANEL

    // We arrive here every ~100ms when idling often enough.
    // Instead of tracking the changes simply redraw the Info Screen ~1 time a second.
    static int8_t lcd_status_update_delay = 1; // first update one loop delayed
    if (
      #if ENABLED(ULTIPANEL)
        currentScreen == lcd_status_screen &&
      #endif
        !lcd_status_update_delay--) {
      lcd_status_update_delay = 9;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }

    if (lcdDrawUpdate) {

      switch (lcdDrawUpdate) {
        case LCDVIEW_CALL_NO_REDRAW:
          lcdDrawUpdate = LCDVIEW_NONE;
          break;
        case LCDVIEW_CLEAR_CALL_REDRAW: // set by handlers, then altered after (rarely occurs here)
        case LCDVIEW_CALL_REDRAW_NEXT:  // set by handlers, then altered after (never occurs here?)
          lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        case LCDVIEW_REDRAW_NOW:        // set above, or by a handler through LCDVIEW_CALL_REDRAW_NEXT
        case LCDVIEW_NONE:
          break;
      }

      #if ENABLED(DOGLCD)  // Changes due to different driver architecture of the DOGM display
        static int8_t dot_color = 0;
        dot_color = 1 - dot_color;
        u8g.firstPage();
        do {
          lcd_setFont(FONT_MENU);
          u8g.setPrintPos(125, 0);
          u8g.setColorIndex(dot_color); // Set color for the alive dot
          u8g.drawPixel(127, 63); // draw alive dot
          u8g.setColorIndex(1); // black on white
          (*currentScreen)();
        } while (u8g.nextPage());
      #elif ENABLED(ULTIPANEL)
        (*currentScreen)();
      #else
        lcd_status_screen();
      #endif
    }

    #if ENABLED(ULTIPANEL)

      // Return to Status Screen after a timeout
      if (currentScreen == lcd_status_screen || defer_return_to_status)
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
      else if (ELAPSED(ms, return_to_status_ms))
        lcd_return_to_status();

    #endif // ULTIPANEL

    switch (lcdDrawUpdate) {
      case LCDVIEW_CLEAR_CALL_REDRAW:
        lcd_implementation_clear();
      case LCDVIEW_CALL_REDRAW_NEXT:
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        break;
      case LCDVIEW_REDRAW_NOW:
        lcdDrawUpdate = LCDVIEW_NONE;
        break;
      case LCDVIEW_NONE:
        break;
    }

  }
}

void set_utf_strlen(char* s, uint8_t n) {
  uint8_t i = 0, j = 0;
  while (s[i] && (j < n)) {
    #if ENABLED(MAPPER_NON)
      j++;
    #else
      if ((s[i] & 0xC0u) != 0x80u) j++;
    #endif
    i++;
  }
  while (j++ < n) s[i++] = ' ';
  s[i] = '\0';
}

void lcd_finishstatus(bool persist=false) {
  set_utf_strlen(lcd_status_message, LCD_WIDTH);
  #if !(ENABLED(LCD_PROGRESS_BAR) && (PROGRESS_MSG_EXPIRE > 0))
    UNUSED(persist);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)
    progress_bar_ms = millis();
    #if PROGRESS_MSG_EXPIRE > 0
      expire_status_ms = persist ? 0 : progress_bar_ms + PROGRESS_MSG_EXPIRE;
    #endif
  #endif
  lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
}

#if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
  void dontExpireStatus() { expire_status_ms = 0; }
#endif

bool lcd_hasstatus() { return (lcd_status_message[0] != '\0'); }

void lcd_setstatus(const char* message, bool persist) {
  if (lcd_status_message_level > 0) return;
  strncpy(lcd_status_message, message, 3 * (LCD_WIDTH));
  lcd_finishstatus(persist);
}

void lcd_setstatuspgm(const char* message, uint8_t level) {
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;
  strncpy_P(lcd_status_message, message, 3 * (LCD_WIDTH));
  lcd_finishstatus(level > 0);
}

void lcd_setalertstatuspgm(const char* message) {
  lcd_setstatuspgm(message, 1);
  #if ENABLED(ULTIPANEL)
    lcd_return_to_status();
  #endif
}

void lcd_reset_alert_level() { lcd_status_message_level = 0; }

#if HAS_LCD_CONTRAST
  void set_lcd_contrast(int value) {
    lcd_contrast = constrain(value, LCD_CONTRAST_MIN, LCD_CONTRAST_MAX);
    u8g.setContrast(lcd_contrast);
  }
#endif

#if ENABLED(ULTIPANEL)

  /**
   * Setup Rotary Encoder Bit Values (for two pin encoders to indicate movement)
   * These values are independent of which pins are used for EN_A and EN_B indications
   * The rotary encoder part is also independent to the chipset used for the LCD
   */
  #if defined(EN_A) && defined(EN_B)
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #endif

  #define GET_BUTTON_STATES(DST) \
    uint8_t new_##DST = 0; \
    WRITE(SHIFT_LD, LOW); \
    WRITE(SHIFT_LD, HIGH); \
    for (int8_t i = 0; i < 8; i++) { \
      new_##DST >>= 1; \
      if (READ(SHIFT_OUT)) SBI(new_##DST, 7); \
      WRITE(SHIFT_CLK, HIGH); \
      WRITE(SHIFT_CLK, LOW); \
    } \
    DST = ~new_##DST; //invert it, because a pressed switch produces a logical 0


  /**
   * Read encoder buttons from the hardware registers
   * Warning: This function is called from interrupt context!
   */
  void lcd_buttons_update() {
    #if ENABLED(NEWPANEL)
      uint8_t newbutton = 0;
      #if BUTTON_EXISTS(EN1)
        if (BUTTON_PRESSED(EN1)) newbutton |= EN_A;
      #endif
      #if BUTTON_EXISTS(EN2)
        if (BUTTON_PRESSED(EN2)) newbutton |= EN_B;
      #endif
      #if LCD_HAS_DIRECTIONAL_BUTTONS || BUTTON_EXISTS(ENC)
        millis_t now = millis();
      #endif

      #if LCD_HAS_DIRECTIONAL_BUTTONS
        if (ELAPSED(now, next_button_update_ms)) {
          if (false) {
            // for the else-ifs below
          }
          #if BUTTON_EXISTS(UP)
            else if (BUTTON_PRESSED(UP)) {
              encoderDiff = -(ENCODER_STEPS_PER_MENU_ITEM);
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(DWN)
            else if (BUTTON_PRESSED(DWN)) {
              encoderDiff = ENCODER_STEPS_PER_MENU_ITEM;
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(LFT)
            else if (BUTTON_PRESSED(LFT)) {
              encoderDiff = -(ENCODER_PULSES_PER_STEP);
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(RT)
            else if (BUTTON_PRESSED(RT)) {
              encoderDiff = ENCODER_PULSES_PER_STEP;
              next_button_update_ms = now + 300;
            }
          #endif
        }
      #endif

      #if BUTTON_EXISTS(ENC)
        if (ELAPSED(now, next_button_update_ms) && BUTTON_PRESSED(ENC)) newbutton |= EN_C;
      #endif

      buttons = newbutton;
      #if ENABLED(LCD_HAS_SLOW_BUTTONS)
        buttons |= slow_buttons;
      #endif
     #else
      GET_BUTTON_STATES(buttons);
    #endif //!NEWPANEL

    // Manage encoder rotation
    #if ENABLED(REVERSE_MENU_DIRECTION) && ENABLED(REVERSE_ENCODER_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff -= encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff += encoderDirection)
    #elif ENABLED(REVERSE_MENU_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff += encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff -= encoderDirection)
    #elif ENABLED(REVERSE_ENCODER_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff--)
      #define ENCODER_DIFF_CCW (encoderDiff++)
    #else
      #define ENCODER_DIFF_CW  (encoderDiff++)
      #define ENCODER_DIFF_CCW (encoderDiff--)
    #endif
    #define ENCODER_SPIN(_E1, _E2) switch (lastEncoderBits) { case _E1: ENCODER_DIFF_CW; break; case _E2: ENCODER_DIFF_CCW; }

    uint8_t enc = 0;
    if (buttons & EN_A) enc |= B01;
    if (buttons & EN_B) enc |= B10;
    if (enc != lastEncoderBits) {
      switch (enc) {
        case encrot0: ENCODER_SPIN(encrot3, encrot1); break;
        case encrot1: ENCODER_SPIN(encrot0, encrot2); break;
        case encrot2: ENCODER_SPIN(encrot1, encrot3); break;
        case encrot3: ENCODER_SPIN(encrot2, encrot0); break;
      }
    }
    lastEncoderBits = enc;
  }

  bool lcd_detected(void) {
      return true;
  }

  bool lcd_clicked() { return LCD_CLICKED; }

#endif // ULTIPANEL

/*********************************/
/** Number to string conversion **/
/*********************************/

#define DIGIT(n) ('0' + (n))
#define DIGIMOD(n) DIGIT((n) % 10)

char conv[8];

// Convert float to rj string with 123 or -12 format
char *ftostr3(const float& x) { return itostr3((int)x); }

// Convert float to rj string with _123, -123, _-12, or __-1 format
char *ftostr4sign(const float& x) { return itostr4sign((int)x); }

// Convert unsigned int to string with 12 format
char* itostr2(const uint8_t& x) {
  int xx = x;
  conv[0] = DIGIMOD(xx / 10);
  conv[1] = DIGIMOD(xx);
  conv[2] = '\0';
  return conv;
}

// Convert float to string with +123.4 / -123.4 format
char* ftostr41sign(const float& x) {
  int xx = int(abs(x * 10)) % 10000;
  conv[0] = x >= 0 ? '+' : '-';
  conv[1] = DIGIMOD(xx / 1000);
  conv[2] = DIGIMOD(xx / 100);
  conv[3] = DIGIMOD(xx / 10);
  conv[4] = '.';
  conv[5] = DIGIMOD(xx);
  conv[6] = '\0';
  return conv;
}

// Convert signed float to string with 023.45 / -23.45 format
char *ftostr32(const float& x) {
  long xx = abs(x * 100);
  conv[0] = x >= 0 ? DIGIMOD(xx / 10000) : '-';
  conv[1] = DIGIMOD(xx / 1000);
  conv[2] = DIGIMOD(xx / 100);
  conv[3] = '.';
  conv[4] = DIGIMOD(xx / 10);
  conv[5] = DIGIMOD(xx);
  conv[6] = '\0';
  return conv;
}

// Convert signed float to string (6 digit) with -1.234 / _0.000 / +1.234 format
char* ftostr43sign(const float& x, char plus/*=' '*/) {
  long xx = x * 1000;
  if (xx == 0)
    conv[0] = ' ';
  else if (xx > 0)
    conv[0] = plus;
  else {
    xx = -xx;
    conv[0] = '-';
  }
  conv[1] = DIGIMOD(xx / 1000);
  conv[2] = '.';
  conv[3] = DIGIMOD(xx / 100);
  conv[4] = DIGIMOD(xx / 10);
  conv[5] = DIGIMOD(xx);
  conv[6] = '\0';
  return conv;
}

// Convert unsigned float to string with 1.23 format
char* ftostr12ns(const float& x) {
  long xx = x * 100;
  xx = abs(xx);
  conv[0] = DIGIMOD(xx / 100);
  conv[1] = '.';
  conv[2] = DIGIMOD(xx / 10);
  conv[3] = DIGIMOD(xx);
  conv[4] = '\0';
  return conv;
}

// Convert signed int to lj string with +012 / -012 format
char* itostr3sign(const int& x) {
  int xx;
  if (x >= 0) {
    conv[0] = '+';
    xx = x;
  }
  else {
    conv[0] = '-';
    xx = -x;
  }
  conv[1] = DIGIMOD(xx / 100);
  conv[2] = DIGIMOD(xx / 10);
  conv[3] = DIGIMOD(xx);
  conv[4] = '.';
  conv[5] = '0';
  conv[6] = '\0';
  return conv;
}

// Convert signed int to rj string with 123 or -12 format
char* itostr3(const int& x) {
  int xx = x;
  if (xx < 0) {
    conv[0] = '-';
    xx = -xx;
  }
  else
    conv[0] = xx >= 100 ? DIGIMOD(xx / 100) : ' ';

  conv[1] = xx >= 10 ? DIGIMOD(xx / 10) : ' ';
  conv[2] = DIGIMOD(xx);
  conv[3] = '\0';
  return conv;
}

// Convert unsigned int to lj string with 123 format
char* itostr3left(const int& xx) {
  if (xx >= 100) {
    conv[0] = DIGIMOD(xx / 100);
    conv[1] = DIGIMOD(xx / 10);
    conv[2] = DIGIMOD(xx);
    conv[3] = '\0';
  }
  else if (xx >= 10) {
    conv[0] = DIGIMOD(xx / 10);
    conv[1] = DIGIMOD(xx);
    conv[2] = '\0';
  }
  else {
    conv[0] = DIGIMOD(xx);
    conv[1] = '\0';
  }
  return conv;
}

// Convert signed int to rj string with _123, -123, _-12, or __-1 format
char *itostr4sign(const int& x) {
  int xx = abs(x);
  int sign = 0;
  if (xx >= 100) {
    conv[1] = DIGIMOD(xx / 100);
    conv[2] = DIGIMOD(xx / 10);
  }
  else if (xx >= 10) {
    conv[0] = ' ';
    sign = 1;
    conv[2] = DIGIMOD(xx / 10);
  }
  else {
    conv[0] = ' ';
    conv[1] = ' ';
    sign = 2;
  }
  conv[sign] = x < 0 ? '-' : ' ';
  conv[3] = DIGIMOD(xx);
  conv[4] = '\0';
  return conv;
}

// Convert unsigned float to rj string with 12345 format
char* ftostr5rj(const float& x) {
  long xx = abs(x);
  conv[0] = xx >= 10000 ? DIGIMOD(xx / 10000) : ' ';
  conv[1] = xx >= 1000 ? DIGIMOD(xx / 1000) : ' ';
  conv[2] = xx >= 100 ? DIGIMOD(xx / 100) : ' ';
  conv[3] = xx >= 10 ? DIGIMOD(xx / 10) : ' ';
  conv[4] = DIGIMOD(xx);
  conv[5] = '\0';
  return conv;
}

// Convert signed float to string with +1234.5 format
char* ftostr51sign(const float& x) {
  long xx = abs(x * 10);
  conv[0] = (x >= 0) ? '+' : '-';
  conv[1] = DIGIMOD(xx / 10000);
  conv[2] = DIGIMOD(xx / 1000);
  conv[3] = DIGIMOD(xx / 100);
  conv[4] = DIGIMOD(xx / 10);
  conv[5] = '.';
  conv[6] = DIGIMOD(xx);
  conv[7] = '\0';
  return conv;
}

// Convert signed float to string with +123.45 format
char* ftostr52sign(const float& x) {
  long xx = abs(x * 100);
  conv[0] = (x >= 0) ? '+' : '-';
  conv[1] = DIGIMOD(xx / 10000);
  conv[2] = DIGIMOD(xx / 1000);
  conv[3] = DIGIMOD(xx / 100);
  conv[4] = '.';
  conv[5] = DIGIMOD(xx / 10);
  conv[6] = DIGIMOD(xx);
  conv[7] = '\0';
  return conv;
}

// Convert signed float to space-padded string with -_23.4_ format
char* ftostr52sp(const float& x) {
  long xx = x * 100;
  uint8_t dig;
  if (xx < 0) { // negative val = -_0
    xx = -xx;
    conv[0] = '-';
    dig = (xx / 1000) % 10;
    conv[1] = dig ? DIGIT(dig) : ' ';
  }
  else { // positive val = __0
    dig = (xx / 10000) % 10;
    if (dig) {
      conv[0] = DIGIT(dig);
      conv[1] = DIGIMOD(xx / 1000);
    }
    else {
      conv[0] = ' ';
      dig = (xx / 1000) % 10;
      conv[1] = dig ? DIGIT(dig) : ' ';
    }
  }

  conv[2] = DIGIMOD(xx / 100); // lsd always

  dig = xx % 10;
  if (dig) { // 2 decimal places
    conv[5] = DIGIT(dig);
    conv[4] = DIGIMOD(xx / 10);
    conv[3] = '.';
  }
  else { // 1 or 0 decimal place
    dig = (xx / 10) % 10;
    if (dig) {
      conv[4] = DIGIT(dig);
      conv[3] = '.';
    }
    else {
      conv[3] = conv[4] = ' ';
    }
    conv[5] = ' ';
  }
  conv[6] = '\0';
  return conv;
}

#endif // ULTRA_LCD
