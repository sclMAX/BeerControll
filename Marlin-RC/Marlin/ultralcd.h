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

#ifndef ULTRALCD_H
#define ULTRALCD_H

#include "Marlin.h"

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
void lcd_buzz(long duration, uint16_t freq);
extern int lcd_contrast;
void set_lcd_contrast(int value);
#define LCD_MESSAGEPGM(x) lcd_setstatuspgm(PSTR(x))
#define LCD_ALERTMESSAGEPGM(x) lcd_setalertstatuspgm(PSTR(x))
#define LCD_UPDATE_INTERVAL 100
#define LCD_TIMEOUT_TO_STATUS 15000
extern volatile uint8_t buttons;  //the last checked buttons in a bit array.
void lcd_buttons_update();
void lcd_quick_feedback(); // Audible feedback for a button click - could also be visual
bool lcd_clicked();
void lcd_ignore_click(bool b=true);
extern int preheatHotendTemp1;
extern int preheatBedTemp1;
extern int preheatFanSpeed1;
extern int preheatHotendTemp2;
extern int preheatBedTemp2;
extern int preheatFanSpeed2;
bool lcd_blink();
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
#define LCD_CLICKED (buttons & EN_C)

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
