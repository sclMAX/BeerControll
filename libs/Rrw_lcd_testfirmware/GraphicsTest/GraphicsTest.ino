/* 

this software will test the graphical CLD v1.0 
Uncomment the desired board below before compiling 
Download and install the U8glib library from https://github.com/olikraus/u8glib/wiki
*/

//#define ULTRATRONICS
//#define MEGATRONICS
//#define MINITRONICS
//#define RAMPS


#include "SPI.h"
#include "SD.h"
#include "U8glib.h"
#include <Arduino.h>



/* ultratronics */

#ifdef ULTRATRONICS
  #define SDSC A5
  #define SW_BACK A9
  #define SW_ENC A10
  #define ENCA 20
  #define ENCB 21
  #define LCDSC A8

  U8GLIB_ST7920_128X64_4X u8g(SCK, MOSI, A8);
  //SCK,MOSI,

  #define SERIALTYPE SerialUSB

#endif

#ifdef RAMPS

  /* ramps */
  #define SDSC 53
  #define SW_BACK 40
  #define SW_ENC 63
  #define ENCA 64
  #define ENCB 59
  #define LCDSC 49

  U8GLIB_ST7920_128X64_4X u8g(LCDSC);
  #define SERIALTYPE Serial
#endif


#ifdef MEGATRONICS
  /* megatronics */
  #define SDSC 53
  #define SW_BACK 34
  #define SW_ENC 33
  #define ENCA 44
  #define ENCB 45
  #define LCDSC 56

  U8GLIB_ST7920_128X64_4X u8g(LCDSC);
  #define SERIALTYPE Serial
#endif


#ifdef MINITRONICS
/* minitronics */
  #define SDSC SS
  #define SW_BACK 25
  #define SW_ENC 26
  #define ENCA 17
  #define ENCB 18
  #define LCDSC 15
  #define SERIALTYPE Serial
  U8GLIB_ST7920_128X64_4X u8g(LCDSC);	
#endif
  

void u8g_prepare(void) {
  u8g.setFont(u8g_font_6x10);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();
}

void u8g_box_frame(uint8_t a) {
  u8g.drawStr( 0, 0, "drawBox");
  u8g.drawBox(5, 10, 20, 10);
  u8g.drawBox(10 + a, 15, 30, 7);
  u8g.drawStr( 0, 30, "drawFrame");
  u8g.drawFrame(5, 10 + 30, 20, 10);
  u8g.drawFrame(10 + a, 15 + 30, 30, 7);
}

void u8g_disc_circle(uint8_t a) {
  u8g.drawStr( 0, 0, "drawDisc");
  u8g.drawDisc(10, 18, 9);
  u8g.drawDisc(24 + a, 16, 7);
  u8g.drawStr( 0, 30, "drawCircle");
  u8g.drawCircle(10, 18 + 30, 9);
  u8g.drawCircle(24 + a, 16 + 30, 7);
}

void u8g_r_frame(uint8_t a) {
  u8g.drawStr( 0, 0, "drawRFrame/Box");
  u8g.drawRFrame(5, 10, 40, 30, a + 1);
  u8g.drawRBox(50, 10, 25, 40, a + 1);
}

void u8g_string(uint8_t a) {
  u8g.drawStr(30 + a, 31, " 0");
  u8g.drawStr90(30, 31 + a, " 90");
  u8g.drawStr180(30 - a, 31, " 180");
  u8g.drawStr270(30, 31 - a, " 270");
}

void u8g_line(uint8_t a) {
  u8g.drawStr( 0, 0, "drawLine");
  u8g.drawLine(7 + a, 10, 40, 55);
  u8g.drawLine(7 + a * 2, 10, 60, 55);
  u8g.drawLine(7 + a * 3, 10, 80, 55);
  u8g.drawLine(7 + a * 4, 10, 100, 55);
}

void u8g_triangle(uint8_t a) {
  uint16_t offset = a;
  u8g.drawStr( 0, 0, "drawTriangle");
  u8g.drawTriangle(14, 7, 45, 30, 10, 40);
  u8g.drawTriangle(14 + offset, 7 - offset, 45 + offset, 30 - offset, 57 + offset, 10 - offset);
  u8g.drawTriangle(57 + offset * 2, 10, 45 + offset * 2, 30, 86 + offset * 2, 53);
  u8g.drawTriangle(10 + offset, 40 + offset, 45 + offset, 30 + offset, 86 + offset, 53 + offset);
}

void u8g_ascii_1() {
  char s[2] = " ";
  uint8_t x, y;
  u8g.drawStr( 0, 0, "ASCII page 1");
  for ( y = 0; y < 6; y++ ) {
    for ( x = 0; x < 16; x++ ) {
      s[0] = y * 16 + x + 32;
      u8g.drawStr(x * 7, y * 10 + 10, s);
    }
  }
}

void u8g_ascii_2() {
  char s[2] = " ";
  uint8_t x, y;
  u8g.drawStr( 0, 0, "ASCII page 2");
  for ( y = 0; y < 6; y++ ) {
    for ( x = 0; x < 16; x++ ) {
      s[0] = y * 16 + x + 160;
      u8g.drawStr(x * 7, y * 10 + 10, s);
    }
  }
}

void u8g_extra_page(uint8_t a)
{
  if ( u8g.getMode() == U8G_MODE_HICOLOR || u8g.getMode() == U8G_MODE_R3G3B2) {
    /* draw background (area is 128x128) */
    u8g_uint_t r, g, b;
    b = a << 5;
    for ( g = 0; g < 64; g++ )
    {
      for ( r = 0; r < 64; r++ )
      {
        u8g.setRGB(r << 2, g << 2, b );
        u8g.drawPixel(g, r);
      }
    }
    u8g.setRGB(255, 255, 255);
    u8g.drawStr( 66, 0, "Color Page");
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT )
  {
    u8g.drawStr( 66, 0, "Gray Level");
    u8g.setColorIndex(1);
    u8g.drawBox(0, 4, 64, 32);
    u8g.drawBox(70, 20, 4, 12);
    u8g.setColorIndex(2);
    u8g.drawBox(0 + 1 * a, 4 + 1 * a, 64 - 2 * a, 32 - 2 * a);
    u8g.drawBox(74, 20, 4, 12);
    u8g.setColorIndex(3);
    u8g.drawBox(0 + 2 * a, 4 + 2 * a, 64 - 4 * a, 32 - 4 * a);
    u8g.drawBox(78, 20, 4, 12);
  }
  else
  {
    u8g.drawStr( 0, 12, "setScale2x2");
    u8g.setScale2x2();
    u8g.drawStr( 0, 6 + a, "setScale2x2");
    u8g.undoScale();
  }
}


bool SDOK = false;

uint8_t draw_state = 0;

void draw(void) {
  u8g_prepare();

  char s[2] = " ";
  uint8_t x, y;


  if (SDOK) {
    u8g.drawStr( 0, 0, "SD OK!");
  }
  else
    u8g.drawStr( 0, 0, "SD NOT READY!");

  u8g.drawStr( 0, 10, "Back:");
  if (digitalRead(SW_BACK) == 1)
    u8g.drawStr( 100, 10, "Down");
  else
    u8g.drawStr( 100, 10, "Up");

  u8g.drawStr( 0, 20, "Enc sw:");
  if (digitalRead(SW_ENC) == 1)
    u8g.drawStr( 100, 20, "Down");
  else
    u8g.drawStr( 100, 20, "Up");

  u8g.drawStr( 0, 30, "Enc A:");
  if (digitalRead(ENCA) == 1)
    u8g.drawStr( 100, 30, "Down");
  else
    u8g.drawStr( 100, 30, "Up");

  u8g.drawStr( 0, 40, "Enc B:");
  if (digitalRead(ENCB) == 1)
    u8g.drawStr( 100, 40, "Down");
  else
    u8g.drawStr( 100, 40, "Up");

}

Sd2Card card;
SdVolume volume;
SdFile root;




void setup(void) {

  // flip screen, if required
  //u8g.setRot180();

#if defined(ARDUINO)
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
#endif

#if defined(ULTRATRONICS)
  pinMode(65, OUTPUT);
  pinMode(52, OUTPUT);
  pinMode(51, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(59, OUTPUT);
  pinMode(61, OUTPUT);
  digitalWrite(65, HIGH);
  digitalWrite(51, HIGH);
  digitalWrite(52, HIGH);
  digitalWrite(50, HIGH);
  digitalWrite(59, HIGH);
  digitalWrite(61, HIGH);
#endif

  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(LCDSC, OUTPUT);
  pinMode(SDSC, OUTPUT);
  pinMode(LCDSC, OUTPUT);
  pinMode(SDSC, HIGH);

  pinMode(SW_BACK, INPUT_PULLUP);
  pinMode(SW_ENC, INPUT_PULLUP);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  digitalWrite(LCDSC, HIGH);

  digitalWrite(SW_BACK, HIGH);
  digitalWrite(SW_ENC, HIGH);
  //digitalWrite(17,HIGH);
  //digitalWrite(17,HIGH);


  // digitalWrite(SDSC,HIGH);
  SERIALTYPE.begin(115200);
  SERIALTYPE.print("\nInitializing SD card...");
//u8g.setHardwareBackup(u8g_backup_avr_spi);

}


unsigned long nextMillis = 0;
void loop(void) {

  if (nextMillis < millis()) {

    nextMillis = millis() + 5000;
    if (!card.init(SPI_FULL_SPEED, SDSC)) {
      SERIALTYPE.println("initialization failed. Things to check:");
      SERIALTYPE.println("* is a card is inserted?");
      SERIALTYPE.println("* Is your wiring correct?");
      SERIALTYPE.println("* did you change the chipSelect pin to match your shield or module?");
      SDOK = false;
    } else {
      SDOK = true;
      SERIALTYPE.println("Wiring is correct and a card is present.");

      if (!volume.init(card)) {
        SERIALTYPE.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
        return;
      }

      uint32_t volumesize;
      SERIALTYPE.print("\nVolume type is FAT");
      SERIALTYPE.println(volume.fatType(), DEC);
      SERIALTYPE.println();

      volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
      volumesize *= volume.clusterCount();       // we'll have a lot of clusters
      volumesize *= 512;                            // SD card blocks are always 512 bytes
      SERIALTYPE.print("Volume size (bytes): ");
      SERIALTYPE.println(volumesize);
    }

  }

  // picture loop
  u8g.firstPage();
  do {
    draw();
  } while ( u8g.nextPage() );





}


