
#ifndef ULTRALCD_H
#define ULTRALCD_H

#include "Marlin.h"

#if ENABLED(ULTRA_LCD)
  #define BUTTON_EXISTS(BN) (defined(BTN_## BN) && BTN_## BN >= 0)
  #define BUTTON_PRESSED(BN) !READ(BTN_## BN)
  int lcd_strlen(const char* s);
  int lcd_strlen_P(const char* s);
  void lcd_update();
  void lcd_init();
  bool lcd_hasstatus();
  void lcd_setstatus(const char* message, const bool persist=false);
  void lcd_setstatuspgm(const char* message, const uint8_t level=0);
  void lcd_setalertstatuspgm(const char* message);
  void lcd_reset_alert_level();
  bool lcd_detected(void);
  void lcd_kill_screen();
  void kill_screen(const char* lcd_msg);

  #if HAS_BUZZER
    void lcd_buzz(long duration, uint16_t freq);
  #endif
  #if ENABLED(DOGLCD)
    extern int lcd_contrast;
    void set_lcd_contrast(int value);
  #elif ENABLED(SHOW_BOOTSCREEN)
    void bootscreen();
  #endif

  #define LCD_MESSAGEPGM(x) lcd_setstatuspgm(PSTR(x))
  #define LCD_ALERTMESSAGEPGM(x) lcd_setalertstatuspgm(PSTR(x))

  #define LCD_UPDATE_INTERVAL 100
  #define LCD_TIMEOUT_TO_STATUS 15000

  #if ENABLED(ULTIPANEL)
    extern volatile uint8_t buttons;  //the last checked buttons in a bit array.
    void lcd_buttons_update();
    void lcd_quick_feedback(); // Audible feedback for a button click - could also be visual
    bool lcd_clicked();
    void lcd_ignore_click(bool b=true);
  #else
    FORCE_INLINE void lcd_buttons_update() {}
  #endif

    extern int preheatHotendTemp1;
    extern int preheatBedTemp1;
    extern int preheatMacerador;
    extern int preheatLicor;
    extern int preheatFanSpeed1;
    extern int preheatHotendTemp2;
    extern int preheatBedTemp2;
    extern int preheatFanSpeed2;
    extern const int etapas;
    extern int MaceradorTemp  [] ;//= {60, 65, 71, 75};
    extern int LicorTemp      [] ;//= {65, 70, 75, 78};
    extern int Inicio         [] ;//= {0, 30, 60, 90};
    extern int Duracion       [] ;//= {30, 30, 30, 30};
    extern bool E1Recircula;
    extern bool E2Recircula;
    extern bool E3Recircula;
    extern bool E4Recircula;
    bool lcd_blink();

#if ENABLED(ULTIPANEL)
#define BLEN_A 0
#define BLEN_B 1
    // Encoder click is directly connected
    #if BUTTON_EXISTS(ENC)
      #define BLEN_C 2
      #define EN_C (_BV(BLEN_C))
    #endif
    #define EN_A (_BV(BLEN_A))
    #define EN_B (_BV(BLEN_B))
    #define EN_C (_BV(BLEN_C))
  #endif
  #define LCD_CLICKED (buttons & EN_C)
#else //no LCD
  FORCE_INLINE void lcd_update() {}
  FORCE_INLINE void lcd_init() {}
  FORCE_INLINE bool lcd_hasstatus() { return false; }
  FORCE_INLINE void lcd_setstatus(const char* message, const bool persist=false) {UNUSED(message); UNUSED(persist);}
  FORCE_INLINE void lcd_setstatuspgm(const char* message, const uint8_t level=0) {UNUSED(message); UNUSED(level);}
  FORCE_INLINE void lcd_buttons_update() {}
  FORCE_INLINE void lcd_reset_alert_level() {}
  FORCE_INLINE bool lcd_detected(void) { return true; }

  #define LCD_MESSAGEPGM(x) NOOP
  #define LCD_ALERTMESSAGEPGM(x) NOOP

#endif //ULTRA_LCD

char* itostr2(const uint8_t& x);
char* itostr3sign(const int& x);
char* itostr3(const int& x);
char* itostr3left(const int& x);
char* itostr4sign(const int& x);

char* ftostr3(const float& x);
char* ftostr4sign(const float& x);
char* ftostr41sign(const float& x);
char* ftostr32(const float& x);
char* ftostr43sign(const float& x, char plus=' ');
char* ftostr12ns(const float& x);
char* ftostr5rj(const float& x);
char* ftostr51sign(const float& x);
char* ftostr52sign(const float& x);
char* ftostr52sp(const float& x); // remove zero-padding from ftostr32

#endif //ULTRALCD_H
