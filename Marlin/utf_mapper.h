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

#ifndef UTF_MAPPER_H
#define UTF_MAPPER_H

#include  "language.h"

#if ENABLED(DOGLCD)
  #define HARDWARE_CHAR_OUT u8g.print
#else
  #define HARDWARE_CHAR_OUT lcd.write
#endif

#if ENABLED(DISPLAY_CHARSET_ISO10646_1)
  #define MAPPER_ONE_TO_ONE
#endif

#if ENABLED(MAPPER_C2C3)

  char charset_mapper(char c) {
    static uint8_t utf_hi_char; // UTF-8 high part
    static bool seen_c2 = false;
    uint8_t d = c;
    if ( d >= 0x80u ) { // UTF-8 handling
      if ( (d >= 0xc0u) && (!seen_c2) ) {
        utf_hi_char = d - 0xc2u;
        seen_c2 = true;
        return 0;
      }
      else if (seen_c2) {
        d &= 0x3fu;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80u + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_c2 = false;
    return 1;
  }

#elif ENABLED(MAPPER_CECF)

  char charset_mapper(char c) {
    static uint8_t utf_hi_char; // UTF-8 high part
    static bool seen_ce = false;
    uint8_t d = c;
    if ( d >= 0x80 ) { // UTF-8 handling
      if ( (d >= 0xc0) && (!seen_ce) ) {
        utf_hi_char = d - 0xce;
        seen_ce = true;
        return 0;
      }
      else if (seen_ce) {
        d &= 0x3f;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80 + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_ce = false;
    return 1;
  }

#elif ENABLED(MAPPER_CECF)

  char charset_mapper(char c) {
    static uint8_t utf_hi_char; // UTF-8 high part
    static bool seen_ce = false;
    uint8_t d = c;
    if ( d >= 0x80 ) { // UTF-8 handling
      if ( (d >= 0xc0) && (!seen_ce) ) {
        utf_hi_char = d - 0xce;
        seen_ce = true;
        return 0;
      }
      else if (seen_ce) {
        d &= 0x3f;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80 + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_ce = false;
    return 1;
  }

#elif ENABLED(MAPPER_D0D1_MOD)

  char charset_mapper(char c) {
    // it is a Russian alphabet translation
    // except 0401 --> 0xa2 = Ё, 0451 --> 0xb5 = ё
    static uint8_t utf_hi_char; // UTF-8 high part
    static bool seen_d5 = false;
    uint8_t d = c;
    if (d >= 0x80) { // UTF-8 handling
      if (d >= 0xd0 && !seen_d5) {
        utf_hi_char = d - 0xd0;
        seen_d5 = true;
        return 0;
      }
      else if (seen_d5) {
        d &= 0x3f;
        if (!utf_hi_char && d == 1) {
          HARDWARE_CHAR_OUT((char) 0xa2); // Ё
        }
        else if (utf_hi_char == 1 && d == 0x11) {
          HARDWARE_CHAR_OUT((char)0xb5); // ё
        }
        else {
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x10));
        }
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_d5 = false;
    return 1;
  }

#elif ENABLED(MAPPER_D0D1)

  char charset_mapper(char c) {
    static uint8_t utf_hi_char; // UTF-8 high part
    static bool seen_d5 = false;
    uint8_t d = c;
    if (d >= 0x80u) { // UTF-8 handling
      if (d >= 0xd0u && !seen_d5) {
        utf_hi_char = d - 0xd0u;
        seen_d5 = true;
        return 0;
      }
      else if (seen_d5) {
        d &= 0x3fu;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0xa0u + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT('?');
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_d5 = false;
    return 1;
  }

#elif ENABLED(MAPPER_E382E383)

  char charset_mapper(char c) {
    static uint8_t utf_hi_char; // UTF-8 high part
    static bool seen_e3 = false;
    static bool seen_82_83 = false;
    uint8_t d = c;
    if (d >= 0x80) { // UTF-8 handling
      if (d == 0xe3 && !seen_e3) {
        seen_e3 = true;
        return 0;      // eat 0xe3
      }
      else if (d >= 0x82 && seen_e3 && !seen_82_83) {
        utf_hi_char = d - 0x82;
        seen_82_83 = true;
        return 0;
      }
      else if (seen_e3 && seen_82_83) {
        d &= 0x3f;
        #ifndef MAPPER_ONE_TO_ONE
          HARDWARE_CHAR_OUT((char)pgm_read_byte_near(utf_recode + d + (utf_hi_char << 6) - 0x20));
        #else
          HARDWARE_CHAR_OUT((char)(0x80 + (utf_hi_char << 6) + d)) ;
        #endif
      }
      else {
        HARDWARE_CHAR_OUT((char) '?' );
      }
    }
    else {
      HARDWARE_CHAR_OUT((char) c );
    }
    seen_e3 = false;
    seen_82_83 = false;
    return 1;
  }

#else

  #define MAPPER_NON

  char charset_mapper(char c) {
    HARDWARE_CHAR_OUT( c );
    return 1;
  }

  #endif // code mappers

#endif // UTF_MAPPER_H
